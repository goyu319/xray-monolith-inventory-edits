#pragma once
#include "CombatContract.h"
#include <functional>
#include <map>
#include <utility>
namespace painter { namespace combat { namespace engine {
// Value-only authority-thread intake. Compile without engine PCH, with the pure
// bridge exception boundary. Lookup is the existing native mission_of store.
using MissionLookup = bool (*)(void*,std::uint64_t,Mission&);
enum class MissionIntakeStatus {ready,invalid,stale,cancelled,exhausted,failed};
struct MissionIntakeResult {
    MissionIntakeStatus status=MissionIntakeStatus::invalid;
    std::size_t lookup_count=0,mission_sides=0,retained_absent=0;
};
class MissionIntake {
public:
    MissionIntakeResult update(WorldSnapshot&,MissionLookup,TimeMs retain_ms,void* context=nullptr,
        const std::function<bool()>& cancelled={});
    void reset();
private:
    using Group=std::pair<std::uint32_t,std::uint64_t>;
    struct KnownMember {ActorKey key;TimeMs known_at_ms=0;};
    struct Entry {std::uint32_t side=0;Mission mission;bool have_mission=false;std::vector<KnownMember> roster;};
    std::map<Group,Entry> groups_;
    std::uint64_t epoch_=0,map_=0,sequence_=0;TimeMs now_=0;
    // Registry-generated identities start at 0x80000000. No truncation of a
    // 64-bit native group into the explicit identity namespace.
    std::uint32_t next_=1;
};
}}}
