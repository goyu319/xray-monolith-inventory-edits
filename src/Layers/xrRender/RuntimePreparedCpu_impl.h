// Included only by RuntimeSkinnedChild_impl.h, after vertHW_4W is defined.
// Entries are private to the single RuntimeStageExecutor thread. Shutdown
// clears them only after that thread joins. No visual, shader, device or GPU
// buffer is owned here. ref_smem payloads are write-once and atomically owned.
namespace {
struct RuntimePreparedCpuEntry
{
    const void* owner = nullptr;
    u64 cookie = 0, last_use = 0;
    unsigned registers = 0, bone_count = 0, material = 0;
    u16 slot = 0, material_source = 0;
    bool receives_wallmarks = false;
    std::vector<u8> rig;
    std::vector<gfy_packed::Vertex> input;
    std::vector<u16> input_indices;
    std::vector<u32> original_faces;
    std::vector<vertHW_4W> hw;
    ref_smem<vertBoned4W> vertices;
    ref_smem<u16> indices, bones_used;
    std::vector<CBoneData::FacesVec> faces; // bone order; face IDs remain child-local
    Fbox box;
    u32 rms_bonecount = 0;
    // A cache must not pin the independent projection/GPU budget after the
    // active visual and its marks release a source. Expiry means rebuild it.
    std::shared_ptr<const void> footprint_identity;
    std::weak_ptr<const CSkeletonWallmark::ProjectionSource> projection;
    size_t bytes = 0;
};
struct RuntimePreparedCpuCache
{
    static constexpr size_t budget = 32u * 1024u * 1024u;
    static constexpr size_t max_entry = 8u * 1024u * 1024u;
    static constexpr size_t max_entries = 128;
    std::vector<std::unique_ptr<RuntimePreparedCpuEntry>> entries;
    size_t bytes = 0;
    void erase(size_t i) { bytes -= entries[i]->bytes; entries.erase(entries.begin() + i); }
    void expire(u64 now)
    {
        for (size_t i = entries.size(); i--;) if (now - entries[i]->last_use > 30000) erase(i);
    }
    void oldest()
    {
        size_t n = 0;
        for (size_t i = 1; i < entries.size(); ++i) if (entries[i]->last_use < entries[n]->last_use) n = i;
        erase(n);
    }
};
RuntimePreparedCpuCache runtime_prepared_cpu;
template<class T> bool runtime_exact_rows(const std::vector<T>& a, const std::vector<T>& b)
{
    return a.size() == b.size() && (a.empty() || !memcmp(a.data(), b.data(), a.size() * sizeof(T)));
}
bool runtime_prepared_matches(const RuntimePreparedCpuEntry& e, const RuntimeGeometryStage& stage,
    const RuntimeStagedChild& child)
{
    const auto& src = *child.source;
    return e.owner == stage.owner_model && e.cookie == stage.owner_cookie && e.slot == child.child_index &&
        e.material_source == child.material_source_child && e.material == src.material &&
        e.bone_count == stage.bone_count && e.registers == stage.registers &&
        e.receives_wallmarks == child.receives_wallmarks && e.rig == stage.prepared_rig &&
        runtime_exact_rows(e.input, src.vertices) && runtime_exact_rows(e.input_indices, src.indices) &&
        runtime_exact_rows(e.original_faces, src.original_faces);
}
}
void RuntimePreparedCpuCaptureRig(RuntimeGeometryStage& stage, const RuntimeSkinUpdate& header) noexcept
{
    // Build-time rollback keeps the existing uncached staging path intact.
#if defined(GFY_RUNTIME_PREPARED_CPU_CACHE) && !GFY_RUNTIME_PREPARED_CPU_CACHE
    return;
#endif
    // Capture before queueing: shared_str must never cross into the cache.
    try {
        auto append = [&](const void* p, size_t n) {
            const auto* b = static_cast<const u8*>(p); stage.prepared_rig.insert(stage.prepared_rig.end(), b, b + n);
        };
        for (const auto& bone : header.exact_rig) {
            const u32 length = bone.name.size(); append(&length, sizeof(length));
            if (length) append(bone.name.c_str(), length);
            append(&bone.parent, sizeof(bone.parent)); append(&bone.model_to_bone, sizeof(bone.model_to_bone));
        }
    } catch (...) { stage.prepared_rig.clear(); }
}
bool RuntimePreparedCpuRestore(RuntimeGeometryStage& stage, RuntimeStagedChild& child, RuntimeStageScratch& scratch) noexcept
{
    // Exterior only: rapidly changing cavity material groups must not evict
    // stable authored children. Disabled capture or cancellation is a miss.
    if (stage.prepared_rig.empty() || child.source->material || stage.canceled.load(std::memory_order_relaxed)) return false;
    LARGE_INTEGER start; QueryPerformanceCounter(&start);
    const u64 now = GetTickCount64(); runtime_prepared_cpu.expire(now);
    for (auto& ptr : runtime_prepared_cpu.entries) {
        auto& e = *ptr;
        if (!runtime_prepared_matches(e, stage, child)) continue;
        if (!scratch.hw || !scratch.indices || size_t(child.vbase) + e.input.size() > scratch.rows ||
            size_t(child.ibase) + e.input_indices.size() > scratch.index_rows) break;
        try {
            for (size_t b = 0; b < e.faces.size(); ++b) stage.faces[b][child.child_index] = e.faces[b];
            child.vertices = e.vertices; child.indices = e.indices; child.bones_used = e.bones_used;
            child.box = e.box; child.rms_bonecount = e.rms_bonecount;
            child.vcount = u32(e.input.size()); child.icount = u32(e.input_indices.size());
            CopyMemory(static_cast<vertHW_4W*>(scratch.hw) + child.vbase, e.hw.data(), e.hw.size() * sizeof(vertHW_4W));
            CopyMemory(scratch.indices + child.ibase, e.input_indices.data(), e.input_indices.size() * sizeof(u16));
            child.footprint_identity = e.footprint_identity;
            child.projection = e.projection.lock();
            // Geometry can be identical while the recipient region changes.
            // Rebuild only projection metadata; keep the prepared geometry hit.
            if(child.projection && child.projection->ownership !=
                (child.required_fields?child.required_fields->wallmark_ownership:nullptr))
                child.projection.reset();
            child.projection_attempted = bool(child.projection);
            child.built = true; e.last_use = now;
            ++stage.prepared_hits; stage.prepared_vertices += child.vcount;
            stage.prepared_cache_ms += RuntimeStageMs(start);
            return true;
        } catch (...) { break; } // ordinary BuildChild overwrites all partial outputs
    }
    stage.prepared_cache_ms += RuntimeStageMs(start);
    return false;
}
void RuntimePreparedCpuStore(RuntimeGeometryStage& stage, const RuntimeStagedChild& child, const RuntimeStageScratch& scratch) noexcept
{
    if (stage.prepared_rig.empty() || !child.built || child.source->material || stage.canceled.load(std::memory_order_relaxed)) return;
    LARGE_INTEGER start; QueryPerformanceCounter(&start);
    struct Finish { RuntimeGeometryStage& s; LARGE_INTEGER start; ~Finish() { s.prepared_cache_ms += RuntimeStageMs(start); s.prepared_cache_bytes = runtime_prepared_cpu.bytes; } } finish{stage,start};
    const auto& src = *child.source;
    for (auto& ptr : runtime_prepared_cpu.entries) if (runtime_prepared_matches(*ptr, stage, child)) {
        ptr->footprint_identity = child.footprint_identity;
        ptr->projection = child.projection; ptr->last_use = GetTickCount64(); return;
    }
    // Conservative admission estimate, including owned ref_smem rows. The
    // retained charge below uses actual capacities; one candidate is <=8 MiB.
    size_t estimate = sizeof(RuntimePreparedCpuEntry) + 256 + stage.prepared_rig.size() +
        src.vertices.size() * (sizeof(gfy_packed::Vertex) + sizeof(vertHW_4W) + sizeof(vertBoned4W)) +
        src.indices.size() * sizeof(u16) * 2 + src.original_faces.size() * sizeof(u32) +
        (child.bones_used._get() ? child.bones_used._get()->dwLength : 0) + stage.bone_count * sizeof(CBoneData::FacesVec);
    for (const auto& row : stage.faces) estimate += row[child.child_index].size() * sizeof(u16);
    if (estimate > RuntimePreparedCpuCache::max_entry) return;
    try {
        // At most one generation per owner/slot, including rejected attempts.
        // Exact content checks make rollback safe even if its cached row changed.
        for (size_t i = runtime_prepared_cpu.entries.size(); i--;) {
            const auto& e = *runtime_prepared_cpu.entries[i];
            if (e.owner == stage.owner_model && e.cookie == stage.owner_cookie && e.slot == child.child_index) runtime_prepared_cpu.erase(i);
        }
        while (!runtime_prepared_cpu.entries.empty() &&
            (runtime_prepared_cpu.bytes + estimate > RuntimePreparedCpuCache::budget || runtime_prepared_cpu.entries.size() >= RuntimePreparedCpuCache::max_entries)) runtime_prepared_cpu.oldest();
        std::unique_ptr<RuntimePreparedCpuEntry> e(new RuntimePreparedCpuEntry());
        e->owner = stage.owner_model; e->cookie = stage.owner_cookie; e->last_use = GetTickCount64();
        e->registers = stage.registers; e->bone_count = stage.bone_count; e->material = src.material;
        e->slot = child.child_index; e->material_source = child.material_source_child; e->receives_wallmarks = child.receives_wallmarks;
        e->rig = stage.prepared_rig; e->input = src.vertices; e->input_indices = src.indices; e->original_faces = src.original_faces;
        const auto* hw = static_cast<const vertHW_4W*>(scratch.hw) + child.vbase;
        e->hw.assign(hw, hw + child.vcount);
        e->vertices = child.vertices; e->indices = child.indices; e->bones_used = child.bones_used;
        e->box = child.box; e->rms_bonecount = child.rms_bonecount; e->projection = child.projection;
        e->footprint_identity = child.footprint_identity;
        e->faces.resize(stage.bone_count);
        for (size_t b = 0; b < e->faces.size(); ++b) e->faces[b] = stage.faces[b][child.child_index];
        e->bytes = sizeof(*e) + 256 + e->rig.capacity() + e->input.capacity() * sizeof(gfy_packed::Vertex) +
            e->hw.capacity() * sizeof(vertHW_4W) + e->input_indices.capacity() * sizeof(u16) + e->original_faces.capacity() * sizeof(u32) +
            e->vertices.size() * sizeof(vertBoned4W) + (e->indices.size() + e->bones_used.size()) * sizeof(u16) + e->faces.capacity() * sizeof(CBoneData::FacesVec);
        for (const auto& f : e->faces) e->bytes += f.capacity() * sizeof(u16);
        if (e->bytes > RuntimePreparedCpuCache::max_entry) return;
        while (!runtime_prepared_cpu.entries.empty() && runtime_prepared_cpu.bytes + e->bytes > RuntimePreparedCpuCache::budget) runtime_prepared_cpu.oldest();
        const size_t charge = e->bytes;
        runtime_prepared_cpu.entries.push_back(std::move(e)); runtime_prepared_cpu.bytes += charge;
    } catch (...) { /* optional cache admission must not refuse a wound */ }
    stage.prepared_cache_bytes = runtime_prepared_cpu.bytes;
}
void RuntimePreparedCpuClear() noexcept
{
    runtime_prepared_cpu.entries.clear(); runtime_prepared_cpu.bytes = 0;
}
