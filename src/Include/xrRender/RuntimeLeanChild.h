#pragma once
// Single-pass runtime child build shared by the renderer's off-thread
// publication stage (Layers/xrRender/RuntimeSkinnedChild_impl.h) and the
// offline equivalence harness. Pure CPU: no engine globals, allocators,
// resource manager, D3D or owner-model access, so it may run on any thread.
//
// For one packed child it reproduces, in one pass over the worker rows:
//  - ValidateRuntimeSkinChild (RuntimeSkinValidation.h): same reason, first
//    failing vertex and influence (error.influence is carried across rows there);
//  - CSkeletonX_ST::BuildRuntime (RuntimeSkinnedChild_impl.h): BonesUsed
//    (first-seen order of positive-weight bones) and the maximal bone index;
//    the per-row vertBoned4W / vertHW_4W / box work is delegated to the
//    caller's Sink so the engine keeps using its own vertHW_4W::set and
//    Fbox::modify;
//  - BuildRuntimeFaceLists (RuntimeFaceLists.h): per bone, ascending face ids
//    of faces with a positive-weight corner on that bone (exact pre-count);
//  - CSkeletonX::CacheProjectionSource(iBase 0, iCount, face_table, force) for
//    a child whose only software replica is Vertices4W (SkeletonX.cpp:597-712).
#include <cmath>
#include "RuntimeSignedSkin.h"
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <intrin.h>

namespace runtime_lean {

struct ValidationError { const char* reason = "none"; std::size_t vertex = 0; unsigned influence = 0; };

// Row accessors for gfy_packed::Vertex-like rows: surface[14], bone[4], weight[4].
template <class Row> inline float surf(const Row& v, unsigned k) { return v.surface[k]; }
template <class Row> inline unsigned bone(const Row& v, unsigned k) { return v.bone[k]; }
template <class Row> inline float weight(const Row& v, unsigned k) { return v.weight[k]; }

// One row of ValidateRuntimeSkinChild's loop, in the same order and with the
// same float expressions. `influence` mirrors error.influence, which that
// function does not reset between rows.
template <class Row>
inline const char* validate_row(const Row& v, unsigned bone_count, unsigned& influence, unsigned& max_bone)
{
    for (unsigned k = 0; k < 14; ++k) if (!std::isfinite(surf(v, k))) return "nonfinite_surface";
    const bool signed4=weight(v,3)<0.f&&gfy_signed_hw::valid4(v.weight,true);float sum = 0.f;
    for (unsigned k = 0; k < 4; ++k)
    {
        influence = k;
        if (bone(v, k) >= bone_count) return "bone_index";
        if (!std::isfinite(weight(v, k)) || (weight(v, k) < 0.f&&!signed4) || weight(v, k) > 1.f) return "weight_range";
        sum += weight(v, k); if (bone(v, k) > max_bone) max_bone = bone(v, k);
    }
    if (std::fabs(sum - 1.f) > 1e-5f) return "weight_sum";
    const float left = ((1.f - weight(v, 0)) - weight(v, 1)) - weight(v, 2);
    const float grouped = 1.f - ((weight(v, 0) + weight(v, 1)) + weight(v, 2));
    if ((!signed4&&(left < 0.f || grouped < 0.f)) || std::fabs(left - weight(v, 3)) > 1e-5f) return "reconstructed_fourth_weight";
    unsigned quantized = 0;
    for (unsigned k = 0; k < 3; ++k) quantized += unsigned(std::floor(weight(v, k) * 255.f + .5f));
    if (!signed4 && quantized > 255) return "gpu_quantized_fourth_weight";
    return nullptr;
}

// The two size checks ValidateRuntimeSkinChild performs before any row.
inline const char* validate_counts(std::size_t count, std::size_t index_count)
{
    if (!count || count > 65535) return "vertex_count";
    if (!index_count || index_count % 3 || index_count / 3 > 65535) return "index_count";
    return nullptr;
}

// Validates and converts one child. On success `used` and `max_bone_out` are
// filled, `masks[0..count)` holds each row's positive-weight bone bits (only
// when bone_count <= 64) and sink(i,row) has been called once per row in
// order. `canceled()` is polled every 4096 rows. `seen_wide` is scratch.
template <class Row, class Used, class Scratch, class Sink, class Cancel>
inline bool build_rows(const Row* rows, std::size_t count, const std::uint16_t* indices, std::size_t index_count,
    unsigned bone_count, unsigned registers, std::uint64_t* masks, Used& used, Scratch& seen_wide,
    unsigned& max_bone_out, ValidationError& error, Sink&& sink, Cancel&& canceled)
{
    error = ValidationError{};
    if (const char* why = validate_counts(count, index_count)) { error.reason = why; return false; }
    unsigned influence = 0, max_bone = 0;
    std::uint64_t seen_mask = 0;
    const bool narrow = bone_count <= 64;
    if (!narrow) seen_wide.assign(bone_count, 0);
    used.clear();
    for (std::size_t i = 0; i < count; ++i)
    {
        if (!(i & 4095) && canceled()) { error.reason = "canceled"; error.vertex = i; return false; }
        const Row& v = rows[i];
        if (const char* why = validate_row(v, bone_count, influence, max_bone))
        {
            error.reason = why; error.vertex = i; error.influence = influence; return false;
        }
        std::uint64_t mask = 0;
        for (unsigned k = 0; k < 4; ++k)
        {
            if (weight(v, k) == 0.f) continue;
            const unsigned b = bone(v, k);
            if (narrow)
            {
                const std::uint64_t bit = std::uint64_t(1) << b;
                mask |= bit;
                if (!(seen_mask & bit)) { seen_mask |= bit; used.push_back(std::uint16_t(b)); }
            }
            else if (!seen_wide[b]) { seen_wide[b] = 1; used.push_back(std::uint16_t(b)); }
        }
        if (narrow) masks[i] = mask;
        sink(i, v);
    }
    for (std::size_t k = 0; k < index_count; ++k)
        if (indices[k] >= count) { error.reason = "index_range"; error.vertex = count - 1; error.influence = influence; return false; }
    if (registers < 25 || max_bone > (registers - 25) / 3)
    {
        error.reason = "hardware_palette"; error.vertex = count - 1; error.influence = influence; return false;
    }
    max_bone_out = max_bone;
    return true;
}

// Face rows from per-row bone masks (bone_count <= 64). row(b) returns the
// child's (cleared) face row for bone b. counts has bone_count slots.
template <class RowOf>
inline void face_rows_from_masks(const std::uint64_t* masks, const std::uint16_t* indices, std::size_t index_count,
    unsigned bone_count, std::uint32_t* counts, RowOf&& row)
{
    const std::uint32_t faces = std::uint32_t(index_count / 3);
    for (unsigned b = 0; b < bone_count; ++b) counts[b] = 0;
    for (std::uint32_t f = 0; f < faces; ++f)
    {
        std::uint64_t m = masks[indices[3 * f]] | masks[indices[3 * f + 1]] | masks[indices[3 * f + 2]];
        while (m) { unsigned long b; _BitScanForward64(&b, m); m &= m - 1; ++counts[b]; }
    }
    for (unsigned b = 0; b < bone_count; ++b) { auto& r = row(b); r.clear(); if (counts[b]) r.reserve(counts[b]); }
    for (std::uint32_t f = 0; f < faces; ++f)
    {
        std::uint64_t m = masks[indices[3 * f]] | masks[indices[3 * f + 1]] | masks[indices[3 * f + 2]];
        while (m) { unsigned long b; _BitScanForward64(&b, m); m &= m - 1; row(b).push_back(std::uint16_t(f)); }
    }
}

enum class ProjectionResult { built, empty, invalid, budget };

// CacheProjectionSource(iBase=0, iCount=index_count, face_table, force=true)
// for a child whose only software replica is Vertices4W (vertBoned4W rows):
// same checks in the same order, same remap size (min(65536, vertex_count)),
// same unique-vertex order, same budget request and the same outputs.
// faces_for(b) returns the child's face row for bone b. source provides
// vertices/indices/bones vectors (InputVertex, u32, {first_vertex,vertex_count}).
// reserve(bytes) is ProjectionSource::ReserveBudget on the new source.
template <class Backup, class Source, class FaceRow, class Remap, class Reserve>
inline ProjectionResult build_projection(const Backup* backup, std::size_t vertex_count, const std::uint16_t* indices,
    std::size_t index_count, unsigned bone_count, unsigned max_palette_bones, std::size_t max_source_indices,
    FaceRow&& faces_for, Remap& remap, Source& source, Reserve&& reserve)
{
    if (!bone_count || bone_count > max_palette_bones) return ProjectionResult::invalid;
    std::size_t count = 0;
    for (unsigned b = 0; b < bone_count; ++b)
    {
        count += faces_for(b).size() * std::size_t(3);
        if (count > max_source_indices) return ProjectionResult::invalid;
    }
    if (!count) return ProjectionResult::empty;
    const std::size_t remap_size = vertex_count < (std::size_t(1) << 16) ? vertex_count : (std::size_t(1) << 16);
    remap.assign(remap_size, std::uint32_t(-1));
    std::uint32_t unique_count = 0;
    for (unsigned b = 0; b < bone_count; ++b)
        for (std::uint16_t face : faces_for(b))
        {
            if (std::uint32_t(face) * 3 + 2 >= index_count) return ProjectionResult::invalid;
            for (std::uint32_t k = 0; k < 3; ++k)
            {
                const std::uint32_t index = indices[std::uint32_t(face) * 3 + k];
                if (index >= remap.size()) return ProjectionResult::invalid;
                if (remap[index] == std::uint32_t(-1)) remap[index] = unique_count++;
            }
        }
    using VertexT = typename std::decay<decltype(source.vertices[0])>::type;
    using RangeT = typename std::decay<decltype(source.bones[0])>::type;
    if (!reserve(unique_count * sizeof(VertexT) + count * sizeof(std::uint32_t) + bone_count * sizeof(RangeT)))
        return ProjectionResult::budget;
    source.vertices.resize(unique_count);
    source.indices.resize(count);
    source.bones.resize(bone_count);
    for (std::uint32_t index = 0; index < remap.size(); ++index)
    {
        if (remap[index] == std::uint32_t(-1)) continue;
        auto& out = source.vertices[remap[index]];
        const auto& v = backup[index];
        out.job = 4;
        for (std::uint32_t j = 0; j < 4; ++j) out.ids[j] = v.m[j];
        for (std::uint32_t j = 0; j < 3; ++j) out.weights[j] = v.w[j];
        for (std::uint32_t id : out.ids) if (id >= bone_count) return ProjectionResult::invalid;
        out.p[0] = v.P.x; out.p[1] = v.P.y; out.p[2] = v.P.z;
    }
    std::uint32_t cursor = 0;
    for (unsigned b = 0; b < bone_count; ++b)
    {
        auto& range = source.bones[b];
        range.first_vertex = cursor;
        for (std::uint16_t face : faces_for(b))
            for (std::uint32_t k = 0; k < 3; ++k)
                source.indices[cursor++] = remap[indices[std::uint32_t(face) * 3 + k]];
        range.vertex_count = cursor - range.first_vertex;
    }
    return ProjectionResult::built;
}

} // namespace runtime_lean
