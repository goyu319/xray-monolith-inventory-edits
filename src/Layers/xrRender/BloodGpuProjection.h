#pragma once

#include "BloodGpuSkinning.h"

namespace BloodGpuProjection
{
using Rows = BloodGpuSkinning::Rows;
using InputVertex = BloodGpuSkinning::InputVertex;

struct SourceHandle
{
    std::uint64_t epoch = 0;
    // Logical triangle-stream entries, NOT unique stored vertices. Indexed
    // sources address the original ordered index stream; identity sources
    // address consecutive vertices. Value copies do not create owner leases.
    std::uint32_t slot = 0, generation = 0, vertex_count = 0;
};
struct SourceRange
{
    SourceHandle source;
    // Offsets/counts in the source's logical triangle stream (index entries
    // for indexed registration). Repeated triangles and caller order survive.
    std::uint32_t first_vertex = 0, vertex_count = 0;
};
struct MarkHandle
{
    std::uint64_t epoch = 0;
    std::uint32_t slot = 0, generation = 0, capacity_vertices = 0;
};
struct Projection
{
    Rows view;
    float contact[3], size;
    float normal[3], unused;
};
struct Output
{
    static constexpr unsigned vertex_stride=44; // first24bytes remain FVF::LIT, then float3 restP
    ID3D11Buffer* vertices = nullptr;
    ID3D11Buffer* arguments = nullptr;
    std::uint32_t arguments_byte_offset = 0;
    // Diagnostic/reservation upper bound. NEVER the accepted vertex count.
    std::uint32_t capacity_vertices = 0;
};
struct Counters
{
    std::uint64_t source_uploads = 0, source_vertices_uploaded = 0;
    std::uint64_t source_indices_uploaded = 0, source_bytes_uploaded = 0;
    std::uint64_t marks_created = 0, marks_released = 0, candidate_vertices = 0;
    std::uint64_t projection_dispatches = 0, render_dispatches = 0;
    std::uint64_t fallbacks = 0, resource_failures = 0, resets = 0;
    std::uint64_t render_batches = 0, allocation_polls = 0, allocation_busy = 0;
    std::uint64_t allocation_copies = 0, allocation_reads = 0, reclaimed_vertices = 0;
    std::uint64_t allocation_moves = 0, moved_vertices = 0, moved_payload_bytes = 0;
    std::uint32_t live_sources = 0, live_source_vertices = 0;
    std::uint32_t live_source_indices = 0, live_source_bytes = 0;
    std::uint32_t live_marks = 0, reserved_mark_vertices = 0;
    std::uint64_t gpu_allocated_bytes = 0;
};

// Render thread only. Sources are immutable unique mesh vertices plus ordered
// bone-face indices, uploaded once per registration into one bounded byte arena.
// InputVertex::job MUST contain the original creation
// link count (1..4), independently of repeated IDs. Ranges retain CPU order and
// duplicate triangles. Caller retains immutable hit inputs for any CPU fallback.
// Geometry and draw/dispatch counts remain GPU-authoritative. Optional allocator
// maintenance reads a completed count-only copy without waiting; see Poll below.
class Backend
{
public:
    static constexpr std::uint32_t MaxSources = 2048;
    static constexpr std::uint32_t MaxSourceBytes = 256u * 1024u * 1024u;
    // Theoretical individual bounds; indexed admission checks their combined
    // byte cost. Unique vertex counts need not be a multiple of three.
    static constexpr std::uint32_t MaxSourceVertices = 5162220;
    static constexpr std::uint32_t MaxSourceIndices = MaxSourceBytes / sizeof(std::uint32_t);
    static constexpr std::uint32_t MaxMarks = 4096;
    static constexpr std::uint32_t MaxGeneratedVertices = 3145728;
    static constexpr std::uint32_t MaxPaletteBones = 1024;
    static constexpr std::uint32_t MaxGroups = 2048;
    static constexpr std::uint32_t TrianglesPerGroup = 128;
    static constexpr std::uint32_t MaxCandidateVertices = MaxGroups * TrianglesPerGroup * 3;
    static constexpr std::uint32_t MaxBatchJobs = 4096;
    static constexpr std::uint32_t MaxBatchBones = 65536;
    static constexpr std::uint32_t MaxMovesPerPoll = 8;
    static constexpr std::uint32_t MaxMoveBytesPerPoll = 8u * 1024u * 1024u;

    Backend() noexcept;
    ~Backend();
    Backend(const Backend&) = delete;
    Backend& operator=(const Backend&) = delete;
    bool EnsureReady() noexcept;
    // Identity triangle stream: no index storage or expansion is allocated.
    bool CanRegisterSource(std::uint32_t count) const noexcept;
    bool RegisterSource(const InputVertex* vertices, std::uint32_t count, SourceHandle& handle) noexcept;
    // Indexed triangle stream: vertices uploaded once, indices retain exact
    // finalized per-bone face order/duplicates. All validation precedes either
    // upload and the single atomic arena allocation; failure preserves handle.
    bool CanRegisterSource(std::uint32_t vertexCount, std::uint32_t indexCount) const noexcept;
    bool RegisterSource(const InputVertex* vertices, std::uint32_t vertexCount,
        const std::uint32_t* indices, std::uint32_t indexCount, SourceHandle& handle) noexcept;
    bool IsValid(const SourceHandle& handle) const noexcept;
    void ReleaseSource(SourceHandle& handle) noexcept;
    bool CanCreateMark(const SourceRange* ranges, std::uint32_t rangeCount, std::uint32_t paletteCount) const noexcept;
    bool CreateMark(const SourceRange* ranges, std::uint32_t rangeCount,
        const Rows* hitPalette, std::uint32_t paletteCount, const Projection& projection, MarkHandle& handle) noexcept;
    bool IsValid(const MarkHandle& handle) const noexcept;
    void ReleaseMark(MarkHandle& handle) noexcept;
    // Successful creation retains source slots independently of caller cache lifetime.
    // Skin dispatch is indirect, sized from GPU accepted count. Output occupies
    // stable shared buffers/argument slot until ReleaseMark/Reset; maintenance
    // may move the vertex span and update GPU StartVertex. Never cache a first
    // vertex offset. Draw using the supplied indirect args with IA offset ZERO.
    bool RenderMark(const MarkHandle& handle, const Rows* palette, std::uint32_t paletteCount,
        const Rows& world, std::uint32_t color, Output& output) noexcept;
    bool BeginRenderBatch() noexcept;
    bool CanAppendRender(std::uint32_t paletteCount) const noexcept;
    // melee (value 3): dents / dentCount = the body's dents (8 floats each, rest space): the mark's vertices move with them
    bool AppendRender(const MarkHandle& handle, const Rows* palette, std::uint32_t paletteCount,
        const Rows& world, std::uint32_t color, const float* dents = nullptr, std::uint32_t dentCount = 0) noexcept;
    bool DispatchRenderBatch() noexcept;
    bool GetOutput(const MarkHandle& handle, Output& output) const noexcept;
    // Caller invokes at most once/frame, before admission. At most one READ /
    // DO_NOT_WAIT staging Map; busy returns immediately. A completed count-only
    // copy may free unused candidate reservation tails. Epoch/generation checks
    // reject stale copies; no output/indirect count depends on CPU completion.
    // No Flush, polling loop, GetData or geometry readback occurs here.
    // Up to 8 trimmed prefixes / 8 MiB may then move into earlier free holes
    // using GPU byte copies; cached VB contents and indirect draw offsets follow.
    void PollCompletedAllocations() noexcept;
    Counters GetCounters() const noexcept { return m_Counters; }
    void Reset() noexcept;

private:
    struct State;
    State* m_State;
    std::uint64_t m_Epoch;
    Counters m_Counters;
    bool m_InitializationFailed;
    bool RegisterSourceInternal(const InputVertex* vertices, std::uint32_t vertexCount,
        const std::uint32_t* indices, std::uint32_t indexCount, bool indexed, SourceHandle& handle) noexcept;
};
static_assert(sizeof(Projection) == 80, "GPU projection ABI");
}
