#pragma once
#include "CombatCore.h"
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>
namespace painter { namespace combat { namespace core {
// No runtime policy defaults: the caller must supply the allocated source
// basis, sight policy, storage capacities and memory allowance explicitly.
// ExposureCachePolicy is allocated in ../OutdoorConfig.h.
struct ExposureGeometryStamp {
    std::uint64_t level_epoch=0,map_hash=0,geometry_revision=0,data_revision=0,source_basis=0;
};
struct ExposureSource {
    std::uint64_t id=0; // unique source identity, never an opening or observer id
    std::uint32_t building=invalid_room,room=invalid_room,node=invalid_node;
    Vec3 point; // already validated, including the explicitly chosen sample height
};
struct ExposureOpening {
    std::uint32_t id=0,building=invalid_room,near_room=invalid_room,far_room=invalid_room;
    Vec3 centre,normal;float half_width=0,y0=0,y1=0;
};
struct ExposureSourceOpening {
    static constexpr std::size_t no_second=static_cast<std::size_t>(-1);
    std::size_t source=0,opening=0,second_opening=no_second;
};
struct ExposureGeometryInput {
    ExposureGeometryStamp stamp;
    std::vector<ExposureSource> sources; // sorted strictly by source id
    std::vector<ExposureOpening> openings;
    // Sorted source, opening, second_opening. Duplicates are permitted. A pair
    // means source.room -> opening.near_room -> second_opening.near_room.
    std::vector<ExposureSourceOpening> links;
    // Explicit caller geometry policy. Required positive when pairs are
    // supplied or the complete (possibly empty) pair basis is declared.
    float pair_max_m=0;
    bool paired_links_complete=false;
};
struct ExposureObserver {
    ActorKey actor;std::uint64_t owner=0,snapshot_seq=0;TimeMs observed_at_ms=0;
    std::uint32_t faction=0,building=invalid_room,room=invalid_room;
    Vec3 eye,head;bool eligible=false;
};
struct ExposureCapture {
    ExposureGeometryStamp stamp;
    std::uint32_t side=0,faction=0;std::uint64_t group=0,roster_revision=0,snapshot_seq=0;
    TimeMs now_ms=0;bool contact_active=false;
    std::vector<ExposureObserver> observers; // complete sorted side roster/leases; copied, never borrowed
};
struct ExposureDestination {
    std::uint32_t building=invalid_room,near_room=invalid_room,node=invalid_node;
    Vec3 point; // exact projected position AND height participate in the key
    // Include the validated direct + two-opening source union. Without an
    // explicitly complete pair basis the result remains unsupported/unknown.
    bool paired_openings=false;
};
struct ExposureWorkQuota {
    std::size_t work_left=0,queries_left=0,work_used=0,queries_used=0;
};
enum class ExposureCacheStatus : std::uint8_t { invalid,disabled,memory_limit,queued,pending,complete,unsupported,cancelled,query_failure,transaction_busy,transaction_exhausted };
struct ExposureCacheTransaction {
    ExposureCacheStatus status=ExposureCacheStatus::invalid;
    std::uint64_t token=0;
    bool active=false,committed=false,rolled_back=false;
    std::size_t restored=0;
};
struct ExposureCacheQuote {
    ExposureCacheStatus status=ExposureCacheStatus::invalid;
    bool complete=false,pending=true;
    std::size_t total_sources=0,seen_sources=0,seen_clear_sources=0,red_sources=0,blocked_sources=0,unknown_sources=0;
    std::uint64_t source_basis=0,observation_revision=0,priced_snapshot_seq=0;
    std::uint64_t transaction_token=0; // nonzero means speculative, not published authority
    TimeMs priced_at_ms=0,seen_valid_until_ms=0;
    // Raw unique-source counts only: no normalization, multiplier, room-clear
    // assertion, movement restriction proof or personal firing permission.
};
struct ExposureCacheAdvance {
    ExposureCacheStatus status=ExposureCacheStatus::invalid;
    bool committed=false,cancelled=false,journal_exhausted=false;
    std::size_t work=0,queries=0,columns_completed=0;
};
struct ExposureCacheStats {
    std::size_t charged_bytes=0,payload_bytes=0,column_capacity=0,columns=0,sources=0;
    std::uint64_t evictions=0,observation_revision=0;
    std::size_t transaction_storage_bytes=0,transaction_capacity=0,transaction_records=0;
    std::uint64_t transaction_token=0;
    bool transaction_active=false,transaction_failed=false,rolling_back=false;
};
struct ExposureCacheSight {
    bool valid=false;std::uint64_t source=0;ActorKey observer;
    std::uint64_t owner=0,snapshot_seq=0;TimeMs observed_at_ms=0,valid_until_ms=0;
    std::uint64_t transaction_token=0;
};
// Serialized worker-owned store. Bind/capture/enqueue are explicit, query-free
// intake operations. Advance journals only touched values; cancellation restores
// them, but NEVER refunds caller work/native-query quota. No hidden growing STL
// container is owned: metadata, geometry, columns, sights and undo journal all
// reside in one charged backing allocation. Caller input vectors are not owned.
// An optional outer transaction spans captures/enqueues/multiple advances. It
// restores side observations and capture provenance; successful static line/
// destination work may survive. Only one serialized owner may use the active
// token. It is not safe for concurrent readers or shared mutable RoomSessions.
// Rollback is resumable when cancelled: quotes/sights stay closed until it is
// complete. A failed transaction cannot commit. Every native/work quota remains
// charged. Rebind refuses while active; reset destroys the entire store.
// The caller supplies an audited allocation-overhead allowance; charged_bytes
// includes every owned payload byte and the cache object, not caller input or
// runtime thread stack/allocator-wide bookkeeping. Quotes are current only for
// the last successful capture time. One cache holds one active side's history;
// rebinding another side invalidates sight authority while retaining static lines.
// Each advance makes at most one observer/source sweep. Idle columns and
// ray-starved items yield remaining caller work instead of spinning; cached
// cells and finite-aperture checks can still progress without query allowance.
// Unknown completed attempts retry on the next advance, never become safe.
class RoomExposureCache {
public:
    explicit RoomExposureCache(ExposureCachePolicy);
    ~RoomExposureCache();
    RoomExposureCache(const RoomExposureCache&)=delete;
    RoomExposureCache& operator=(const RoomExposureCache&)=delete;
    ExposureCacheStatus bind(const ExposureGeometryInput&);
    ExposureCacheTransaction begin_transaction(const std::function<bool()>& cancelled={});
    ExposureCacheTransaction commit_transaction(std::uint64_t token,const std::function<bool()>& cancelled={});
    ExposureCacheTransaction rollback_transaction(std::uint64_t token,const std::function<bool()>& cancelled={});
    bool begin_capture(const ExposureCapture&,const std::function<bool()>& cancelled={},bool reset_observations=false);
    bool matches_capture(const ExposureCapture&) const;
    ExposureCacheQuote quote(const ExposureDestination&) const;
    ExposureCacheStatus enqueue(const ExposureDestination&,const std::function<bool()>& cancelled={});
    ExposureCacheAdvance advance(const EngineServices&,ExposureWorkQuota&,const std::function<bool()>& cancelled={});
    ExposureCacheSight sight(std::uint64_t source_id) const;
    ExposureCacheStats stats() const;
    void reset();
private:
    struct Impl;
    ExposureCachePolicy policy_;
    std::unique_ptr<unsigned char[]> storage_;
    Impl* impl_=nullptr;
    ExposureCacheStatus status_=ExposureCacheStatus::invalid;
    std::uint64_t transaction_serial_=0; // never reused, including across reset/rebind
};
}}}
