#ifndef _XR_POOL_H
#define _XR_POOL_H

#include <stddef.h>

/* LuaJIT 2.0 x64 low-address allocator.
** The engine reserves up to 1664 MiB in the lower 2 GB during WinMain and
** commits/decommits 64 KiB chunks on demand. This preserves the address-space
** requirement without committing the entire pool to the system pagefile.
*/
typedef struct xr_luajit_pool_stats
{
    void* base;
    size_t requested_bytes;
    size_t reserved_bytes;
    size_t committed_bytes;
    size_t peak_committed_bytes;
    size_t fallback_bytes;
    size_t peak_fallback_bytes;
    size_t fallback_active_allocations;
    size_t fallback_total_allocations;
    size_t allocation_failures;
} xr_luajit_pool_stats;

void XR_INIT(void);
void* XR_MMAP(size_t size);
int XR_DESTROY(void* p, size_t size);
void XR_EARLY_INIT(void);
void XR_GET_POOL_STATS(xr_luajit_pool_stats* stats);

/* BloodEngine (Painter) compatibility. Painter installs a release hook and reads pool numbers.
** This allocator falls back to the normal heap instead of failing, so the hook is never called;
** it is kept so Painter's code links unchanged. XR_POOL_STATS maps onto XR_GET_POOL_STATS. */
#ifdef __cplusplus
extern "C" {
#endif
typedef int (*xr_alloc_oom_hook_fn)(size_t bytes);
extern xr_alloc_oom_hook_fn volatile xr_alloc_oom_hook;
typedef struct xr_pool_stats {
	size_t reserved_bytes;
	size_t floor_bytes;
	size_t committed_bytes;
	size_t used_bytes;
	unsigned commit_failures;
	unsigned hook_releases;
} xr_pool_stats;
void XR_POOL_STATS(xr_pool_stats* out);
#ifdef __cplusplus
}
#endif

#endif
