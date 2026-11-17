#pragma once
// Off-thread runtime geometry publication (renderer side).
//
// IKinematics::LL_StageRuntimeGeometry (publish barrier, main thread) admits a
// worker output with the same checks and reasons as LL_PrepareRuntimeGeometry,
// captures what the heavy build needs and queues it on ONE renderer-owned
// worker thread. That thread performs validation, vertBoned4W/vertHW_4W
// conversion, D3D11 buffer creation (the device is free-threaded), face lists
// and exterior projection sources. It never touches the owner model, the
// resource manager, stats_manager, shared_str or wallmarks.
// IKinematics::LL_InstallStagedRuntimeGeometry (a LATER publish barrier, once
// Ready()) admits again, wraps the prepared children (resource-manager geometry
// and shader handles, stats), snapshots wallmarks and commits atomically in
// that frame, exactly like LL_InstallRuntimeGeometry.
// Dropping a stage (Cancel + Release) is allowed at any time on any thread; an
// unadopted stage owns only unbound D3D buffers and CPU memory.
struct RuntimeStageTiming
{
    // Barrier A (stage creation), milliseconds
    float stage_ms = 0;
    // Worker thread, milliseconds
    float queue_ms = 0, run_ms = 0, build_ms = 0, buffers_ms = 0, faces_ms = 0, projection_ms = 0;
    // Barrier B (install), milliseconds
    float install_ms = 0, admit_ms = 0, wrap_ms = 0, shader_ms = 0, wallmarks_ms = 0, commit_ms = 0, verify_ms = 0;
    unsigned children = 0, vertices = 0, triangles = 0, shader_reused = 0, shader_compiled = 0, marks = 0;
    int verify_result = -1; // -1 not run, 0 mismatch (logged), 1 identical
};

namespace RuntimeStageOutcome {
enum : int
{
    staged = 0,      // stage returned; install it at a later barrier once Ready()
    rejected = 1,    // admission rejected (logged); acknowledge as not accepted
    unpack = 2,      // packed mesh names a missing tissue texture (AppendPackedRuntimeChildren false)
    unavailable = 3, // off-thread publication disabled/unavailable: use LL_InstallRuntimeGeometry
};
}

class IRuntimeGeometryStage
{
public:
    virtual void AddRef() noexcept = 0;
    // The last reference destroys the stage on the calling thread (safe on any thread).
    virtual void Release() noexcept = 0;
    // True once the worker finished (success, failure or cancellation). Acquire.
    virtual bool Ready() const noexcept = 0;
    // Cooperative cancellation; the worker stops at the next 4096-row boundary.
    virtual void Cancel() noexcept = 0;
    // Valid after Ready() on the owner thread.
    virtual const RuntimeStageTiming& Timing() const noexcept = 0;
protected:
    virtual ~IRuntimeGeometryStage() {}
};
