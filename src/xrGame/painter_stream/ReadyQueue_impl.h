#include "ReadyQueue.h"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace sss24_stream {
namespace {
constexpr std::size_t none=std::numeric_limits<std::size_t>::max();
bool valid_position(Position p) noexcept {
    return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);
}
double distance_squared(Position a,Position b) noexcept {
    const double x=a.x-b.x,y=a.y-b.y,z=a.z-b.z;
    return x*x+y*y+z*z;
}
bool callbacks_valid(const Callbacks& c) noexcept {
    return c.clock_us&&c.inspect&&c.dependency&&c.dispatch&&c.release;
}
std::size_t owner_capacity(std::size_t slots) {
    if(!slots||slots>65536)throw std::invalid_argument("invalid ready queue capacity");
    std::size_t n=2;while(n<slots*2)n*=2;return n;
}
std::uint64_t key_hash(SourceKey k) noexcept {
    std::uint64_t v=k.world^(k.incarnation+0x9e3779b97f4a7c15ULL+(k.world<<6)+(k.world>>2));
    v^=std::uint64_t(k.id)*0xbf58476d1ce4e5b9ULL;
    v^=v>>30;v*=0xbf58476d1ce4e5b9ULL;v^=v>>27;v*=0x94d049bb133111ebULL;return v^(v>>31);
}
}
bool SourceKey::valid() const noexcept { return world&&incarnation&&id!=0xffff; }
bool operator==(SourceKey a,SourceKey b) noexcept {
    return a.world==b.world&&a.incarnation==b.incarnation&&a.id==b.id;
}
bool operator!=(SourceKey a,SourceKey b) noexcept { return !(a==b); }
ReadyQueue::ReadyQueue(Limits l):limits_(l),slots_(l.capacity),owners_(owner_capacity(l.capacity)) {
    if(!l.capacity||l.capacity>65536||!l.spawns_per_frame||!l.packets_per_frame||
       !l.probes_per_frame||!l.slice_us||!l.age_priority_us)
        throw std::invalid_argument("invalid internal ready queue limits");
}
std::size_t ReadyQueue::FindOwner(SourceKey key) const noexcept {
    const auto mask=owners_.size()-1;auto at=std::size_t(key_hash(key))&mask;
    for(std::size_t n=0;n<owners_.size();++n,at=(at+1)&mask){
        const auto& item=owners_[at];
        if(item.state==0)return none;
        if(item.state==1&&item.key==key)return at;
    }
    return none;
}
std::size_t ReadyQueue::Owner(SourceKey key) noexcept {
    const auto existing=FindOwner(key);if(existing!=none)return existing;
    const auto mask=owners_.size()-1;auto at=std::size_t(key_hash(key))&mask;
    for(std::size_t n=0;n<owners_.size();++n,at=(at+1)&mask)if(owners_[at].state==0){
        owners_[at]=OwnerIndex{};owners_[at].key=key;owners_[at].state=1;return at;
    }
    return none;
}
void ReadyQueue::EraseOwner(std::size_t gap) noexcept {
    // Allocation-free linear-probe deletion. Move the COMPLETE owner row, so
    // spawn/event FIFO links and reference counts keep their original slots.
    const auto mask=owners_.size()-1;
    owners_[gap]=OwnerIndex{};
    for(auto at=(gap+1)&mask;owners_[at].state!=0;at=(at+1)&mask){
        const auto home=std::size_t(key_hash(owners_[at].key))&mask;
        if(((at-home)&mask)>=((at-gap)&mask)){
            owners_[gap]=owners_[at];owners_[at]=OwnerIndex{};gap=at;
        }
    }
}
bool ReadyQueue::ValidPacket(const Packet& p) const noexcept {
    if(!p.token||!p.owner.valid()||!valid_position(p.position)||p.dependency_count>p.dependencies.size())return false;
    for(unsigned i=0;i<p.dependency_count;++i)
        if(!p.dependencies[i].valid()||p.dependencies[i].world!=p.owner.world||p.dependencies[i]==p.owner)return false;
    return p.kind==Kind::Spawn||p.kind==Kind::Event;
}
Admission ReadyQueue::Enqueue(const Packet& p) noexcept {
    if(!ValidPacket(p))return Admission::Invalid;
    if(!p.ordinary_ready)return Admission::SynchronousRequired;
    for(const auto& s:slots_)if(s.occupied&&s.packet.token==p.token)return Admission::DuplicateToken;
    // A second spawn for the same exact native incarnation is ambiguous. Drain
    // and let the original bridge decide, rather than dropping/merging packets.
    if(p.kind==Kind::Spawn&&HasPendingSpawn(p.owner))return Admission::NeedDrain;
    if(occupied_==slots_.size()||next_sequence_==std::numeric_limits<std::uint64_t>::max())return Admission::NeedDrain;
    const auto owner_index=Owner(p.owner);if(owner_index==none)return Admission::NeedDrain;
    for(std::size_t at=0;at<slots_.size();++at){auto& s=slots_[at];if(!s.occupied){
        s.packet=p;s.sequence=next_sequence_++;s.attempted_frame=0;s.attempted_drain=0;s.required_generation=0;
        s.next_event=none;s.occupied=true;s.in_flight=false;
        auto& owner=owners_[owner_index];++owner.references;
        if(p.kind==Kind::Spawn)owner.spawn=at;
        else{if(owner.last_event!=none)slots_[owner.last_event].next_event=at;
             else owner.first_event=at;owner.last_event=at;}
        ++occupied_;return Admission::Queued;
    }}
    return Admission::NeedDrain;
}
bool ReadyQueue::HasPendingSpawn(SourceKey key) const noexcept {
    const auto owner=FindOwner(key);return owner!=none&&owners_[owner].spawn!=none;
}
bool ReadyQueue::ReadClock(const Callbacks& c,std::uint64_t& now) noexcept {
    if(!c.clock_us(c.context,now)||now<last_clock_)return false;
    last_clock_=now;return true;
}
bool ReadyQueue::EarlierEvent(std::size_t index) const noexcept {
    const auto& candidate=slots_[index];
    if(candidate.packet.kind!=Kind::Event)return false;
    if(HasPendingSpawn(candidate.packet.owner))return true;
    const auto owner=FindOwner(candidate.packet.owner);
    return owner!=none&&owners_[owner].first_event!=index;
}
std::size_t ReadyQueue::Select(const Frame& f,std::uint64_t now,std::uint64_t required,bool all) const noexcept {
    std::size_t best=none;bool best_aged=false;double best_distance=0;
    for(std::size_t i=0;i<slots_.size();++i){
        const auto& s=slots_[i];
        if(!s.occupied||s.in_flight||s.attempted_drain==drain_generation_||s.packet.owner.world!=f.world||
           (!all&&s.attempted_frame==f.serial)||
           (required&&s.required_generation!=required)||EarlierEvent(i))continue;
        bool pending=false;
        for(unsigned d=0;d<s.packet.dependency_count;++d)
            if(HasPendingSpawn(s.packet.dependencies[d])){pending=true;break;}
        if(pending)continue;
        const bool aged=now>=s.packet.arrived_us&&now-s.packet.arrived_us>=limits_.age_priority_us;
        const double distance=distance_squared(s.packet.position,f.actual_actor_position);
        if(best==none|| (aged&&!best_aged)||
           (aged==best_aged&&(aged?s.sequence<slots_[best].sequence:
             (distance<best_distance||(distance==best_distance&&s.sequence<slots_[best].sequence))))){
            best=i;best_aged=aged;best_distance=distance;
        }
    }
    return best;
}
State ReadyQueue::Dependencies(const Packet& p,const Callbacks& c) const noexcept {
    if(p.kind==Kind::Event){
        const auto state=c.dependency(c.context,p.owner);if(state!=State::Ready)return state;
    }
    for(unsigned i=0;i<p.dependency_count;++i){
        const auto state=c.dependency(c.context,p.dependencies[i]);if(state!=State::Ready)return state;
    }
    return State::Ready;
}
void ReadyQueue::Release(std::size_t i,const Callbacks& c,ReleaseReason reason) noexcept {
    auto& s=slots_[i];c.release(c.context,s.packet,reason);
    const auto index=FindOwner(s.packet.owner);
    if(index!=none){
        auto& owner=owners_[index];
        if(s.packet.kind==Kind::Spawn)owner.spawn=none;
        else{
            std::size_t previous=none;
            for(auto at=owner.first_event;at!=none;at=slots_[at].next_event){
                if(at==i){
                    if(previous==none)owner.first_event=s.next_event;else slots_[previous].next_event=s.next_event;
                    if(owner.last_event==i)owner.last_event=previous;break;
                }
                previous=at;
            }
        }
        if(--owner.references==0)EraseOwner(index);
    }
    s.occupied=false;s.in_flight=false;--occupied_;
}
DrainResult ReadyQueue::Drain(const Frame& f,const Callbacks& c,std::uint64_t required,bool all) noexcept {
    DrainResult result;result.incomplete=required?RequiredPending(required):occupied_!=0;
    if(draining_){result.reentrant=true;return result;}
    if(!f.world||!f.serial||!valid_position(f.actual_actor_position)||!callbacks_valid(c)||
       drain_generation_==std::numeric_limits<std::uint64_t>::max()){
        result.invalid_frame=true;return result;
    }
    if(f.paused&&!all)return result;
    std::uint64_t now{};if(!ReadClock(c,now)){result.clock_failed=true;return result;}
    if(frame_world_!=f.world||frame_serial_!=f.serial){
        if(frame_world_==f.world&&f.serial<frame_serial_){result.invalid_frame=true;return result;}
        frame_world_=f.world;frame_serial_=f.serial;frame_start_=now;
        frame_spawns_=frame_packets_=frame_probes_=0;
    }
    draining_=true;
    ++drain_generation_;
    struct Leave {bool& value;~Leave(){value=false;}} leave{draining_};
    std::size_t operations=0;
    while(operations++<slots_.size()){
        if(!all&&(frame_packets_>=limits_.packets_per_frame||frame_probes_>=limits_.probes_per_frame||
          now-frame_start_>=limits_.slice_us))break;
        const auto index=Select(f,now,required,all);if(index==none)break;
        auto& s=slots_[index];s.attempted_frame=f.serial;s.attempted_drain=drain_generation_;s.in_flight=true;
        ++result.examined;++frame_probes_;
        State state=c.inspect(c.context,s.packet);
        if(state==State::Ready)state=Dependencies(s.packet,c);
        if(state==State::Retired){Release(index,c,ReleaseReason::NativeRetired);++result.retired;}
        else if(state!=State::Ready){s.in_flight=false;++result.waiting;}
        else if(!all&&s.packet.kind==Kind::Spawn&&s.packet.spawn_budget&&frame_spawns_>=limits_.spawns_per_frame){
            s.in_flight=false;break;
        }else{
            if(!ReadClock(c,now)){s.in_flight=false;result.clock_failed=true;break;}
            if(!all&&now-frame_start_>=limits_.slice_us){s.in_flight=false;break;}
            const auto kind=s.packet.kind;const auto spawn_budget=s.packet.spawn_budget;
            const auto dispatched=c.dispatch(c.context,s.packet);
            if(dispatched==Dispatch::Applied){
                Release(index,c,ReleaseReason::Applied);++frame_packets_;
                if(kind==Kind::Spawn){if(spawn_budget)++frame_spawns_;++result.spawned;}else ++result.events;
            }else if(dispatched==Dispatch::Retired){Release(index,c,ReleaseReason::NativeRetired);++result.retired;}
            else{s.in_flight=false;++result.waiting;}
        }
        // Even an irreversible Applied result is recorded/released before a
        // subsequent clock failure. Never replay the completed native spawn.
        if(!ReadClock(c,now)){result.clock_failed=true;break;}
    }
    result.incomplete=required?RequiredPending(required):occupied_!=0;
    return result;
}
DrainResult ReadyQueue::DrainFrame(const Frame& f,const Callbacks& c) noexcept { return Drain(f,c,0,false); }
DrainResult ReadyQueue::DrainRecovery(const Frame& f,const Callbacks& c) noexcept { return Drain(f,c,0,true); }
bool ReadyQueue::RequiredPending(std::uint64_t generation) const noexcept {
    for(const auto& s:slots_)if(s.occupied&&s.required_generation==generation)return true;
    return false;
}
DrainResult ReadyQueue::DrainRequired(const Packet& immediate,const Frame& f,const Callbacks& c) noexcept {
    DrainResult failed;failed.incomplete=true;
    if(draining_){failed.reentrant=true;return failed;}
    if(!callbacks_valid(c)||!f.world||!f.serial||!valid_position(f.actual_actor_position)||
       !ValidPacket(immediate)||immediate.owner.world!=f.world||
       closure_generation_==std::numeric_limits<std::uint64_t>::max()){
        failed.invalid_frame=true;return failed;
    }
    const auto generation=++closure_generation_;
    auto mark=[&](SourceKey key)noexcept{
        bool changed=false;
        for(auto& s:slots_)if(s.occupied&&s.packet.owner==key&&s.required_generation!=generation){
            s.required_generation=generation;changed=true;
        }
        return changed;
    };
    // All already-parked traffic for the same owner preceded this critical
    // packet; preserve it even when the critical packet is a second spawn.
    mark(immediate.owner);
    for(unsigned i=0;i<immediate.dependency_count;++i)mark(immediate.dependencies[i]);
    for(std::size_t pass=0;pass<slots_.size();++pass){
        bool changed=false;
        for(const auto& s:slots_)if(s.occupied&&s.required_generation==generation)
            for(unsigned i=0;i<s.packet.dependency_count;++i)changed=mark(s.packet.dependencies[i])||changed;
        if(!changed)break;
    }
    auto result=Drain(f,c,generation,true);
    result.incomplete=RequiredPending(generation);
    if(result.clock_failed||result.invalid_frame||result.reentrant)result.incomplete=true;
    if(!result.incomplete){
        // Explicitly require the external native parents/targets too. Missing
        // dependency metadata is a bridge error, not a fabricated success.
        const auto external=Dependencies(immediate,c);
        result.incomplete=external!=State::Ready;
    }
    return result;
}
bool ReadyQueue::RetireWorld(std::uint64_t world,const Callbacks& c) noexcept {
    if(draining_||!world||!c.release)return false;
    for(std::size_t i=0;i<slots_.size();++i)
        if(slots_[i].occupied&&slots_[i].packet.owner.world==world)Release(i,c,ReleaseReason::NativeRetired);
    return true;
}
} // namespace sss24_stream
