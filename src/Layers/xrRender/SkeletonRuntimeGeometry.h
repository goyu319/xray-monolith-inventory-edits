#pragma once
#include "SkeletonCustom.h"
#include "../../Include/xrRender/RuntimeSkinGeometry.h"
struct RuntimeRequiredFields;
struct RuntimeGeometryPacket
{
    xr_vector<IRenderVisual*> children; // Includes borrowed unchanged originals.
    xr_vector<dxRender_Visual*> owned;
    xr_vector<xr_vector<CBoneData::FacesVec>> faces;
    u64 owner_cookie = 0, damage_revision = 0, geometry_generation = 0;
    bool separation_packet = false;
    std::shared_ptr<RuntimeRequiredFields> required_fields;
#ifdef USE_DX11
    std::shared_ptr<PainterBloodInstance> blood_cells; // shared rest-cell lineage, committed with geometry
#endif
    RuntimeGeometryPacket* next_retired = nullptr;
#ifdef USE_DX11
    ID3DQuery* fence = nullptr;
#endif
    ~RuntimeGeometryPacket();
};
struct RuntimeGeometryPrepared
{
    RuntimeGeometryPacket* packet = nullptr;
    RuntimeGeometryPacket* prior_packet = nullptr;
    SkeletonWMVec wallmarks;
    xr_vector<IRenderVisual*> published;
    ~RuntimeGeometryPrepared() { xr_delete(packet); }
};
void CollectRuntimeGeometryRetired(bool shutdown);
// Off-thread publication executor: lazily started by the first stage, stopped
// (queued stages canceled, thread joined) when the model pool is destroyed.
void RuntimeGeometryStageShutdown();
class CSkeletonX_ST;
#ifdef USE_DX11
#include "../../Include/xrRender/RuntimeGeometryStage.h"
#include "../../Include/xrRender/RuntimePackedMesh.h"
#include "../../Include/xrRender/RuntimeLeanChild.h"
#include "RuntimeShaderCache.h"
#include "PainterTissueField.h"
#include "RuntimeRequiredFields.h"
#include <atomic>
#include <memory>
// 0 = synchronous publication only, 1 = off-thread prepare (default),
// 2 = off-thread prepare + install-time comparison against BuildRuntime.
extern int r_painter_publish;
// One child prepared off the publish barrier (RuntimeGeometryStage.h). Holds no
// shared_str (str_value's counter is not atomic, xrstring.h:92-93) and nothing
// registered with the resource manager or stats_manager, so it may be
// destroyed on any thread. Unadopted D3D buffers were never bound.
struct RuntimeStagedChild
{
    std::shared_ptr<PainterTissueGpu> painter_tissue;
    std::shared_ptr<RuntimeRequiredFields> required_fields;
    bool skin_clip=false,skin_owned=false,analytic=false;
    // Captured at stage creation (publish barrier).
    u16 child_index = 0, material_source_child = 0;
    bool receives_wallmarks = true, remove_surface = false;
    bool material_valid = true, removed_valid = true; // admission checks evaluated in packed order by Run()
    xr_string shader, texture; // non-empty overrides the original material
    const gfy_packed::Child* source = nullptr; // owned by RuntimeGeometryStage::packed
    // Produced by Run().
    bool built = false, projection_attempted = false;
    ref_smem<vertBoned4W> vertices;
    ref_smem<u16> bones_used, indices;
    // One VB and one IB per publication (RuntimeGeometryStage::vb/ib): every
    // built child holds its own reference to the packet buffers and draws its
    // rows at vbase/ibase (Fvisual::vBase/iBase, DrawIndexed base vertex and
    // start index). The CPU copies (vertices, indices) stay per child.
    ID3D11Buffer* vb = nullptr;
    ID3D11Buffer* ib = nullptr;
    u32 vbase = 0, ibase = 0;
    Fbox box;
    u32 rms_bonecount = 0, vcount = 0, icount = 0;
    std::shared_ptr<const void> footprint_identity;
    std::shared_ptr<const CSkeletonWallmark::ProjectionSource> projection;
    RuntimeStagedChild() { box.invalidate(); }
    ~RuntimeStagedChild() { _RELEASE(vb); _RELEASE(ib); }
    RuntimeStagedChild(const RuntimeStagedChild&) = delete;
    RuntimeStagedChild& operator=(const RuntimeStagedChild&) = delete;
};
class RuntimeGeometryStage final : public IRuntimeGeometryStage
{
public:
    std::atomic<long> refs{1};
    bool tissue_only=false;std::shared_ptr<PainterTissueGpu> painter_tissue;
    bool fields_only=false,build_channels=false,build_removal=false,build_skin_ownership=false,build_wallmark_ownership=false;
    std::shared_ptr<RuntimeRequiredFields> required_fields;
    RuntimeGeometryStage* queue_next = nullptr; // executor FIFO (intrusive, no allocation)
    LARGE_INTEGER submitted{};
    const void* owner_model = nullptr; // identity only, never dereferenced off the barrier
    u64 owner_cookie = 0, expected_generation = 0, damage_revision = 0;
    bool separation_packet = false, published_gap = false;
    u16 bone_count = 0;
    unsigned registers = 0;
    size_t originals_count = 0, count = 0;
    ID3D11Device* device = nullptr; // AddRef'd at creation; Run() never reads HW
    // Packet buffers: all built children's vertHW_4W rows / u16 indices in
    // packed order, created with two CreateBuffer calls by Run(). The stage
    // releases its references once every child has adopted its own (install)
    // or in its destructor.
    ID3D11Buffer* vb = nullptr;
    ID3D11Buffer* ib = nullptr;
    size_t total_vertices = 0, total_indices = 0;
    std::shared_ptr<const gfy_packed::Mesh> packed; // worker output rows, kept alive until the stage dies
    std::vector<std::unique_ptr<RuntimeStagedChild>> children; // std::vector: xalloc cannot hold move-only types; packed order (== ascending slot order)
    xr_vector<xr_vector<CBoneData::FacesVec>> faces; // [bone][slot]; replaced rows built by Run()
    std::atomic<bool> canceled{false}, ready{false};
    bool ok = false, installed = false;
    const char* reason = "stage_not_run"; // owner-level reason when !ok
    string512 child_message{}; // child-level rejection line, logged by install
    RuntimeStageTiming timing;
    std::vector<u8> prepared_rig; // exact serialized rig, captured at the barrier
    unsigned prepared_hits = 0, prepared_vertices = 0;
    size_t prepared_cache_bytes = 0;
    float prepared_cache_ms = 0;
    void Run() noexcept; // executor thread only, exactly once
    void AddRef() noexcept override { refs.fetch_add(1, std::memory_order_relaxed); }
    void Release() noexcept override { if (refs.fetch_sub(1, std::memory_order_acq_rel) == 1) delete this; }
    bool Ready() const noexcept override { return ready.load(std::memory_order_acquire) && (required_fields ? required_fields->DecisionReady() : (!painter_tissue || tissue_only || painter_tissue->native.load(std::memory_order_acquire)==3 || painter_tissue->warmed.load(std::memory_order_acquire))); }
    void Cancel() noexcept override { canceled.store(true, std::memory_order_relaxed); }
    const RuntimeStageTiming& Timing() const noexcept override { return timing; }
private:
    ~RuntimeGeometryStage() override { children.clear(); _RELEASE(vb); _RELEASE(ib); _RELEASE(device); }
};
float RuntimeStageMs(const LARGE_INTEGER& from) noexcept;
// Per-stage scratch: the packet's vertHW_4W rows and u16 indices (every built
// child at its vbase/ibase; the initial data of the two packet buffers) and
// the bone masks of the largest child (<= 65535 rows). Uninitialized: every
// row is written before it is read.
struct RuntimeStageScratch
{
    void* hw = nullptr;
    u16* indices = nullptr;
    u64* masks = nullptr;
    size_t rows = 0, index_rows = 0, mask_rows = 0;
    bool reserve_packet(size_t vertices, size_t indices) noexcept;
    bool reserve_masks(size_t count) noexcept;
    void release() noexcept;
    ~RuntimeStageScratch() { release(); }
};
// Worker thread; implemented next to vertHW_4W (RuntimeSkinnedChild_impl.h).
// Writes the child's rows at child.vbase/ibase of the packet scratch.
bool RuntimeStageBuildChild(RuntimeStagedChild& child, unsigned bone_count, unsigned registers,
    xr_vector<xr_vector<CBoneData::FacesVec>>& faces, const std::atomic<bool>& canceled, RuntimeStageTiming& timing,
    RuntimeStageScratch& scratch, char* message, size_t message_size) noexcept;
// Executor-thread-only optional immutable CPU cache; clear only after executor join.
void RuntimePreparedCpuCaptureRig(RuntimeGeometryStage& stage, const RuntimeSkinUpdate& header) noexcept;
bool RuntimePreparedCpuRestore(RuntimeGeometryStage& stage, RuntimeStagedChild& child, RuntimeStageScratch& scratch) noexcept;
void RuntimePreparedCpuStore(RuntimeGeometryStage& stage, const RuntimeStagedChild& child, const RuntimeStageScratch& scratch) noexcept;
void RuntimePreparedCpuClear() noexcept;
// Two CreateBuffer calls (DEFAULT usage, initial data, the description
// dx10BufferUtils::CreateBuffer uses) for the whole packet. Worker thread.
bool RuntimeStagePackBuffers(ID3D11Device* device, const RuntimeStageScratch& scratch, size_t vertices, size_t indices,
    ID3D11Buffer** vb, ID3D11Buffer** ib, char* message, size_t message_size) noexcept;
// Barrier-only shader handle resolution for runtime children, backed by the
// renderer-wide RuntimeShaderCache (RuntimeShaderCache.h). seed() records the
// handles the OGF originals and the active packet's children already hold, so
// exterior pairs never compile; a cavity pair compiles once per game and is
// then reused by every NPC and revision.
struct RuntimeShaderReuse
{
    unsigned reused = 0, compiled = 0;
    bool enabled = true;
    void seed(const RuntimeGeometryPacket* active, const xr_vector<IRenderVisual*>* originals);
};
// Resolve `shader` for a runtime child whose dbg_shader_def / dbg_texture_def /
// hud / skinning are set: a cache hit copies the handle and the names exactly
// as SetShaderTexture would leave them; a miss compiles through
// ResetShaderTexture (skinning mode restored afterwards) and records the
// handle. Publish barrier only.
void RuntimeResolveChildShader(dxRender_Visual& child, RuntimeShaderReuse* reuse);
#endif
