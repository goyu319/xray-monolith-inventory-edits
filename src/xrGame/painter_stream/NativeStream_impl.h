#pragma once
// Compiled once by Level.cpp. Uses native typed objects; no binary ABI views.
#include "NativeStream.h"
#include "ReadyQueue.h"
#include "ReadyQueue_impl.h"
#include "../NET_Queue.h"
#include "../Level.h"
#include "../actor.h"
#include "../xrServer.h"
#include "../ai_space.h"
#include "xrserver_objects_alife.h"
#include "xrserver_objects_alife_monsters.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <mutex>
#include <vector>

namespace painter_stream {
namespace {
thread_local const CSE_ALifeDynamicObject* ordinary_root=nullptr;
thread_local const CSE_Abstract* bundle_root=nullptr;
thread_local xrServer* local_delivery_server=nullptr;
thread_local NativeStream* receive_stream=nullptr;
thread_local const NET_Packet* receive_packet=nullptr;
thread_local std::size_t receive_record=0;
NativeStream* stream_for(xrServer*)noexcept;
using Key=sss24_stream::SourceKey;
Key key(const Birth& b)noexcept{return {b.world,b.activation,b.id};}
bool valid(const Birth& b)noexcept{return key(b).valid()&&b.server_birth;}
bool same(const Birth& a,const Birth& b)noexcept{return key(a)==key(b)&&a.server_birth==b.server_birth;}
bool finite(const Fvector& p)noexcept{return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
bool clock_us(void*,std::uint64_t& value)noexcept{
    const auto n=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    if(n<0)return false;value=static_cast<std::uint64_t>(n);return true;
}
bool ordinary_member(const CSE_Abstract& object)noexcept{
    if(!ordinary_root||!ordinary_root->PainterBirthCookie())return false;
    const auto* current=&object;
    for(unsigned depth=0;current&&depth<64;++depth){
        if(current==ordinary_root)return current->PainterBirthCookie()==ordinary_root->PainterBirthCookie();
        const auto* monster=smart_cast<const CSE_ALifeMonsterAbstract*>(current);
        if(monster&&monster->m_group_id==ordinary_root->ID)return true;
        if(current->ID_Parent==0xffff||!g_pGameLevel||!Level().Server)break;
        current=Level().Server->ID_to_entity(current->ID_Parent);
    }
    return false;
}
// Actual typed message schema. Required object references are creation
// dependencies; nullable combat attribution urgently promotes only an already
// parked exact incarnation and never blocks damage on an absent shooter.
struct References {std::array<std::uint16_t,4> ids{};unsigned count{};bool critical{},optional{};};
References references(std::uint16_t type,const std::uint8_t* bytes,std::size_t size)noexcept{
    References result;
    auto take=[&](unsigned offset){if(offset+2<=size){std::uint16_t id;std::memcpy(&id,bytes+offset,2);if(id!=0xffff&&result.count<result.ids.size())result.ids[result.count++]=id;}};
    switch(type){
    case GE_OWNERSHIP_TAKE:case GE_OWNERSHIP_TAKE_MP_FORCED:case GE_OWNERSHIP_REJECT:
    case GE_DESTROY_REJECT:case GE_TRADE_SELL:case GE_TRADE_BUY:case GE_ADDON_ATTACH:
    case GEG_PLAYER_ITEM2SLOT:case GEG_PLAYER_ITEM2BELT:case GEG_PLAYER_ITEM2RUCK:
    case GEG_PLAYER_ITEM_EAT:case GEG_PLAYER_ACTIVATEARTEFACT:case GEG_PLAYER_ATTACH_HOLDER:
    case GE_LAUNCH_ROCKET:take(0);break;
    case GE_HIT:case GE_HIT_STATISTIC:take(0);take(2);result.critical=true;result.optional=true;break;
    case GE_DIE:case GE_ASSIGN_KILLER:case GE_KILL_SOMEONE:take(0);result.critical=true;result.optional=true;break;
    case GE_DESTROY:result.critical=true;break;
    default:break;
    }
    return result;
}
}
OrdinaryOnlineScope::OrdinaryOnlineScope(CSE_ALifeDynamicObject& object)noexcept:previous_(ordinary_root){ordinary_root=&object;}
OrdinaryOnlineScope::~OrdinaryOnlineScope(){ordinary_root=previous_;}
LocalDeliveryScope::LocalDeliveryScope(xrServer& server)noexcept:previous_(local_delivery_server){local_delivery_server=&server;}
LocalDeliveryScope::~LocalDeliveryScope(){local_delivery_server=previous_;}
OnlineBundleScope::OnlineBundleScope(CSE_ALifeDynamicObject& object)noexcept:previous_(bundle_root){
    if(ordinary_member(object)){root_=&object;bundle_root=root_;}
}
OnlineBundleScope::~OnlineBundleScope(){
    if(root_&&g_pGameLevel&&Level().Server)if(auto* stream=stream_for(Level().Server))stream->CloseBundle(*root_);
    bundle_root=previous_;
}

struct NativeStream::Impl {
    static constexpr std::size_t capacity=2048,retained_capacity=65536+capacity;
    struct ServerLife {Birth birth,bundle;sss24_stream::Position position;bool active{},ordinary{},rejected{};};
    struct Bundle {Key root;std::vector<Key> members;bool closed{};};
    struct ClientLife {Key source;std::uint64_t cookie{};};
    struct Stored {NET_Event event;bool used{},critical{},admitted{};std::uint64_t token{},arrived_us{};std::size_t active_position{};};
    CLevel& level;const std::uint64_t world;
    std::uint64_t activation{},token{},arrival{},frame_serial{};std::uint32_t last_frame=std::numeric_limits<std::uint32_t>::max();
    mutable std::mutex metadata;
    mutable std::recursive_mutex owner;
    std::array<ServerLife,65536> server;
    std::array<ClientLife,65536> client;
    struct ReceivedRecord {std::size_t offset{},size{};Stamp stamp;};
    // One entry for each actual native INetQueue packet, erased BEFORE Release
    // permits pointer reuse. Storage follows the already-owned native queue;
    // there is no 4096-emission cutoff or missing authentication placeholder.
    std::map<const NET_Packet*,std::vector<ReceivedRecord>> received;
    // Core scheduling slots are bounded separately from retained packets. Native
    // object IDs cap a closed bundle; gear outside core slots remains owned.
    std::vector<Stored> stored;
    std::vector<std::size_t> free_slots,active_slots;
    std::array<std::size_t,65536> spawn_slot{};
    std::array<std::uint32_t,65536> owner_traffic{};
    std::map<std::uint16_t,Bundle> bundles;
    std::array<Key,capacity> just_born{};std::size_t born_count{};
    sss24_stream::ReadyQueue queue;
    std::atomic<bool> stopped{false};bool dispatching{};
    explicit Impl(CLevel& l):level(l),world(l.PainterWorldCookie()),stored(retained_capacity),queue(sss24_stream::Limits{}){
        free_slots.reserve(retained_capacity);active_slots.reserve(retained_capacity);
        for(std::size_t n=retained_capacity;n;--n)free_slots.push_back(n-1);
        spawn_slot.fill(SIZE_MAX);
    }
    ServerLife life(std::uint16_t id)const noexcept{
        std::lock_guard<std::mutex> lock(metadata);return server[id];
    }
    bool frame(sss24_stream::Frame& out)noexcept{
        auto* actor=smart_cast<CActor*>(level.CurrentEntity());
        if(stopped||!actor||actor->getDestroy()||!finite(actor->Position())||!world)return false;
        if(last_frame!=Device.dwFrame){if(frame_serial==UINT64_MAX)return false;last_frame=Device.dwFrame;++frame_serial;}
        out={world,frame_serial,{actor->Position().x,actor->Position().y,actor->Position().z},Device.Paused()};return true;
    }
    bool current_client(Key expected)noexcept{
        if(!expected.valid()||expected.world!=world)return false;
        const auto& known=client[expected.id];
        auto* object=level.Objects.net_Find(expected.id);
        return object&&known.source==expected&&known.cookie&&object->PainterBirthCookie()==known.cookie&&!object->getDestroy();
    }
    sss24_stream::State dependency(Key expected)noexcept{
        if(current_client(expected))return sss24_stream::State::Ready;
        const auto at=life(expected.id);
        return key(at.birth)==expected&&at.active&&!at.rejected?sss24_stream::State::Wait:sss24_stream::State::Retired;
    }
    Stored* find(std::uint64_t value)noexcept{
        const auto index=static_cast<std::size_t>(value&((1ULL<<17)-1));
        return index<stored.size()&&stored[index].used&&stored[index].token==value?&stored[index]:nullptr;
    }
    void release(Stored& row)noexcept{
        const auto index=static_cast<std::size_t>(&row-stored.data());const auto id=row.event.painter_stamp.owner.id;
        if(spawn_slot[id]==index)spawn_slot[id]=SIZE_MAX;
        R_ASSERT(owner_traffic[id]);--owner_traffic[id];
        // Preserve admission order for backlog owner FIFO. All capacity was
        // reserved at world construction; erasing indices cannot allocate.
        active_slots.erase(active_slots.begin()+row.active_position);
        for(std::size_t n=row.active_position;n<active_slots.size();++n)stored[active_slots[n]].active_position=n;
        row.event.data.clear();row.event.painter_stamp={};row.used=false;row.critical=false;row.admitted=false;
        free_slots.push_back(index);
    }
    bool pending_traffic(Key expected)const noexcept{
        if(!owner_traffic[expected.id])return false;
        for(const auto index:active_slots)if(key(stored[index].event.painter_stamp.owner)==expected)return true;
        return false;
    }
    bool unadmitted_traffic(Key expected)const noexcept{
        if(!owner_traffic[expected.id])return false;
        for(const auto index:active_slots){const auto& row=stored[index];
            if(!row.admitted&&key(row.event.painter_stamp.owner)==expected)return true;}
        return false;
    }
    Stored* spawn(Key expected)noexcept{
        const auto index=spawn_slot[expected.id];
        if(index==SIZE_MAX)return nullptr;auto& row=stored[index];
        return row.used&&row.event.ID==M_SPAWN&&key(row.event.painter_stamp.owner)==expected?&row:nullptr;
    }
    sss24_stream::State bundle_ready(Key expected)noexcept{
        std::lock_guard<std::mutex> lock(metadata);
        const auto found=bundles.find(expected.id);
        if(found==bundles.end()||found->second.root!=expected||!found->second.closed)return sss24_stream::State::Wait;
        for(const auto member:found->second.members){
            if(member==expected)continue;
            const auto& at=server[member.id];
            if(key(at.birth)!=member||!at.active||at.rejected||current_client(member))continue;
            auto* child=spawn(member);if(!child)return sss24_stream::State::Wait;
            // The root must not become visible while a child is blocked by an
            // older client instance, even when its bytes are already prefetched.
            if(level.Objects.net_Find(member.id))return sss24_stream::State::Wait;
            const auto& stamp=child->event.painter_stamp;
            for(unsigned d=0;d<stamp.dependency_count;++d){
                const auto required=key(stamp.dependencies[d]);bool inside=false;
                for(const auto sibling:found->second.members)if(sibling==required){inside=true;break;}
                if(!inside&&!current_client(required))return sss24_stream::State::Wait;
            }
        }
        return sss24_stream::State::Ready;
    }
    sss24_stream::Callbacks callbacks()noexcept{
        return {this,&clock_us,
            [](void* context,const sss24_stream::Packet& packet)noexcept{
                auto& self=*static_cast<Impl*>(context);auto* stored=self.find(packet.token);
                if(!stored||!same(stored->event.painter_stamp.owner,self.life(packet.owner.id).birth)){
                    // An old destroy/hit still belongs to an old *live client*.
                    if(packet.kind==sss24_stream::Kind::Event&&self.current_client(packet.owner))return sss24_stream::State::Ready;
                    return sss24_stream::State::Retired;
                }
                if(packet.kind==sss24_stream::Kind::Event)return self.dependency(packet.owner);
                const auto at=self.life(packet.owner.id);
                if(!at.active||at.rejected)return sss24_stream::State::Retired;
                if(key(at.bundle)==packet.owner){
                    const auto bundle=self.bundle_ready(packet.owner);if(bundle!=sss24_stream::State::Ready)return bundle;
                }
                // Never create a reused ID over its preceding client instance.
                auto* old=self.level.Objects.net_Find(packet.owner.id);
                return old&&!self.current_client(packet.owner)?sss24_stream::State::Wait:sss24_stream::State::Ready;
            },
            [](void* context,Key expected)noexcept{return static_cast<Impl*>(context)->dependency(expected);},
            [](void* context,const sss24_stream::Packet& packet)noexcept{
                auto& self=*static_cast<Impl*>(context);auto* row=self.find(packet.token);
                if(!row)return sss24_stream::Dispatch::Unavailable;
                if(self.dispatching)return sss24_stream::Dispatch::Wait;
                self.dispatching=true;
                struct Leave {bool& value;~Leave(){value=false;}}leave{self.dispatching};
                if(row->event.ID==M_SPAWN){
                    if(!self.level.PainterDispatchSpawn(row->event))return sss24_stream::Dispatch::Retired;
                    self.spawn_applied(row->event);return sss24_stream::Dispatch::Applied;
                }
                NET_Packet packet_body;row->event.implication(packet_body);
                self.level.cl_Process_Event(row->event.destination,row->event.type,packet_body);
                return sss24_stream::Dispatch::Applied;
            },
            [](void* context,const sss24_stream::Packet& packet,sss24_stream::ReleaseReason)noexcept{
                auto& self=*static_cast<Impl*>(context);if(auto* row=self.find(packet.token))self.release(*row);
            }};
    }
    void spawn_applied(const NET_Event& event)noexcept{
        const auto& born=event.painter_stamp.owner;
        if(!event.painter_stamp.authenticated||!valid(born)||born.world!=world)return;
        auto* object=level.Objects.net_Find(born.id);
        const bool accepted=object&&!object->getDestroy();
        if(accepted)client[born.id]={key(born),object->PainterBirthCookie()};
        std::lock_guard<std::mutex> lock(metadata);auto& at=server[born.id];
        if(!same(at.birth,born))return;
        at.rejected=!accepted;
        if(accepted&&key(at.bundle)==key(born)){
            R_ASSERT2(born_count<just_born.size(),"Native atomic bundle completion overflow");
            just_born[born_count++]=key(born);
        }
    }
    bool waiting(const Birth& born)noexcept{
        if(!valid(born)||born.world!=world||current_client(key(born)))return false;
        const auto at=life(born.id);
        // An already-created native object that took the untouched fallback
        // path is outside this sidecar's ownership, not a permanently missing
        // deferred spawn. Native synchronous event dispatch remains its owner.
        if(level.Objects.net_Find(born.id)&&!spawn(key(born)))return false;
        return same(at.birth,born)&&at.active&&!at.rejected;
    }
    void promote_optional(const Stamp& stamp,const sss24_stream::Frame& frame,const sss24_stream::Callbacks& calls)noexcept{
        for(unsigned n=0;n<stamp.optional_count;++n){
            const auto expected=key(stamp.optional_dependencies[n]);
            if(!queue.HasPendingSpawn(expected))continue;
            sss24_stream::Packet urgent;urgent.token=1;urgent.owner=expected;
            urgent.kind=sss24_stream::Kind::Spawn;urgent.position=life(expected.id).position;
            urgent.ordinary_ready=true;
            // Attribution is NEVER a retained dependency of target damage.
            // Failure/retirement here leaves the original nullable ID untouched.
            (void)queue.DrainRequired(urgent,frame,calls);finish_bundles(frame,calls);
        }
    }
    sss24_stream::Packet packet_for(const Stored& row)noexcept{
        const auto& stamp=row.event.painter_stamp;const auto at=life(stamp.owner.id);
        sss24_stream::Packet packet;packet.token=row.token;packet.owner=key(stamp.owner);
        packet.kind=row.event.ID==M_SPAWN?sss24_stream::Kind::Spawn:sss24_stream::Kind::Event;
        packet.position=at.position;packet.ordinary_ready=true;packet.arrived_us=row.arrived_us;
        packet.spawn_budget=!valid(at.bundle)||key(at.bundle)==packet.owner;
        for(unsigned d=0;d<stamp.dependency_count;++d)packet.dependencies[packet.dependency_count++]=key(stamp.dependencies[d]);
        return packet;
    }
    void admit_backlog(const sss24_stream::Callbacks& calls)noexcept{
        for(std::size_t n=0;n<active_slots.size();){
            auto& row=stored[active_slots[n]];
            if(row.admitted){++n;continue;}
            auto packet=packet_for(row);const auto state=calls.inspect(calls.context,packet);
            if(state==sss24_stream::State::Retired){release(row);continue;}
            if(queue.size()==queue.capacity())break;
            const auto result=queue.Enqueue(packet);
            if(result==sss24_stream::Admission::Queued)row.admitted=true;
            else if(result==sss24_stream::Admission::NeedDrain)break;
            ++n;
        }
    }
    void finish_bundles(const sss24_stream::Frame& frame,const sss24_stream::Callbacks& calls)noexcept{
        // No AI update occurs between the root dispatch and this completion.
        // Only this exact closed NPC bundle bypasses the remaining frame budget.
        while(born_count){
            const auto root=just_born[--born_count];std::size_t count{};
            {std::lock_guard<std::mutex> lock(metadata);const auto at=bundles.find(root.id);
                if(at==bundles.end()||at->second.root!=root)continue;count=at->second.members.size();}
            for(std::size_t pass=0;pass<count;++pass){bool pending=false,progress=false;
                for(std::size_t n=0;n<count;++n){Key child;
                    {std::lock_guard<std::mutex> lock(metadata);const auto at=bundles.find(root.id);
                        if(at==bundles.end()||at->second.root!=root||n>=at->second.members.size())break;child=at->second.members[n];}
                    if(child==root)continue;
                    auto* row=spawn(child);if(!row)continue;pending=true;
                    if(row->admitted){
                        const auto before=queue.size();sss24_stream::Packet urgent;urgent.token=1;
                        urgent.owner=child;urgent.kind=sss24_stream::Kind::Event;urgent.position=frame.actual_actor_position;urgent.ordinary_ready=true;
                        queue.DrainRequired(urgent,frame,calls);progress=progress||queue.size()!=before;
                    }else{
                        const auto packet=packet_for(*row);auto state=calls.inspect(calls.context,packet);
                        if(state==sss24_stream::State::Ready)for(unsigned d=0;d<packet.dependency_count;++d){
                            state=calls.dependency(calls.context,packet.dependencies[d]);if(state!=sss24_stream::State::Ready)break;
                        }
                        const auto result=state==sss24_stream::State::Ready?calls.dispatch(calls.context,packet):
                            state==sss24_stream::State::Retired?sss24_stream::Dispatch::Retired:sss24_stream::Dispatch::Wait;
                        if(result==sss24_stream::Dispatch::Applied||result==sss24_stream::Dispatch::Retired){
                            calls.release(calls.context,packet,result==sss24_stream::Dispatch::Applied?
                                sss24_stream::ReleaseReason::Applied:sss24_stream::ReleaseReason::NativeRetired);progress=true;
                        }
                    }
                }
                if(!pending||!progress)break;
            }
        }
    }
    bool promote_prerequisite(Key expected,const sss24_stream::Frame& frame,const sss24_stream::Callbacks& calls,unsigned depth=0)noexcept{
        if(current_client(expected))return true;
        if(depth>=64)return false;
        auto* row=spawn(expected);if(!row)return false;
        const auto token=row->token;const auto packet=packet_for(*row);
        for(unsigned d=0;d<packet.dependency_count;++d)
            if(!promote_prerequisite(packet.dependencies[d],frame,calls,depth+1))return false;
        // A parent atomic-bundle completion can have consumed this child.
        row=find(token);if(!row)return current_client(expected);
        if(row->admitted){
            sss24_stream::Packet urgent;urgent.token=1;urgent.owner=expected;urgent.kind=sss24_stream::Kind::Event;
            urgent.position=frame.actual_actor_position;urgent.ordinary_ready=true;
            queue.DrainRequired(urgent,frame,calls);finish_bundles(frame,calls);
            return current_client(expected);
        }
        auto state=calls.inspect(calls.context,packet);
        if(state==sss24_stream::State::Ready)for(unsigned d=0;d<packet.dependency_count;++d){
            state=calls.dependency(calls.context,packet.dependencies[d]);if(state!=sss24_stream::State::Ready)break;
        }
        const auto result=state==sss24_stream::State::Ready?calls.dispatch(calls.context,packet):
            state==sss24_stream::State::Retired?sss24_stream::Dispatch::Retired:sss24_stream::Dispatch::Wait;
        if(result==sss24_stream::Dispatch::Applied||result==sss24_stream::Dispatch::Retired)
            calls.release(calls.context,packet,result==sss24_stream::Dispatch::Applied?
                sss24_stream::ReleaseReason::Applied:sss24_stream::ReleaseReason::NativeRetired);
        finish_bundles(frame,calls);return current_client(expected);
    }
    Stamp authenticate(const std::uint8_t* data,std::size_t size)noexcept{
        if(stopped||size<2||size>16384)return {};
        std::uint16_t type;std::memcpy(&type,data,2);
        Stamp stamp;std::uint16_t owner=0xffff;
        std::uint16_t event_type=0;References refs;
        if(type==M_SPAWN){
            NET_Packet packet;packet.B.count=static_cast<u32>(size);std::memcpy(packet.B.data,data,size);
            u16 parent;shared_str section;owner=GetSpawnInfo(packet,parent,section);
            if(parent!=0xffff){refs.ids[0]=parent;refs.count=1;}
        }else if(type==M_EVENT&&size>=10){
            std::memcpy(&event_type,data+6,2);std::memcpy(&owner,data+8,2);
            refs=references(event_type,data+10,size-10);
        }else return {};
        if(owner==0xffff)return {};
        const auto& at=server[owner];if(!valid(at.birth)||at.birth.world!=world)return {};
        stamp.owner=at.birth;stamp.authenticated=true;stamp.ordinary=type==M_SPAWN&&at.ordinary;
        stamp.critical=refs.critical;
        for(unsigned i=0;i<refs.count;++i){const auto& dependency=server[refs.ids[i]].birth;
            if(valid(dependency)&&dependency.world==world&&!same(dependency,stamp.owner)){
                if(refs.optional)stamp.optional_dependencies[stamp.optional_count++]=dependency;
                else stamp.dependencies[stamp.dependency_count++]=dependency;
            }
        }
        return stamp;
    }
};

NativeStream::NativeStream(CLevel& level):impl_(new Impl(level)){}
NativeStream::~NativeStream(){WorldStopped();delete impl_;}
void NativeStream::ServerActivation(CSE_Abstract& object,bool existing)noexcept{
    auto& i=*impl_;if(i.stopped||object.ID==0xffff||!finite(object.o_Position))return;
    std::lock_guard<std::mutex> lock(i.metadata);
    if(i.activation==UINT64_MAX)return;
    auto& at=i.server[object.ID];at={};at.birth={i.world,++i.activation,object.PainterBirthCookie(),object.ID};
    at.position={object.o_Position.x,object.o_Position.y,object.o_Position.z};at.active=true;
    at.ordinary=existing&&g_bootComplete&&ordinary_member(object);
    if(at.ordinary&&bundle_root&&bundle_root->ID!=0xffff){
        const auto& root=i.server[bundle_root->ID].birth;
        if(valid(root)&&root.server_birth==bundle_root->PainterBirthCookie()){
            at.bundle=root;auto& bundle=i.bundles[root.id];
            if(bundle.root!=key(root)){bundle.root=key(root);bundle.members.clear();bundle.closed=false;}
            bundle.members.push_back(key(at.birth));
        }
    }
}
void NativeStream::CloseBundle(const CSE_Abstract& object)noexcept{
    auto& i=*impl_;if(object.ID==0xffff)return;std::lock_guard<std::mutex> lock(i.metadata);
    const auto& root=i.server[object.ID].birth;const auto at=i.bundles.find(object.ID);
    if(root.server_birth==object.PainterBirthCookie()&&at!=i.bundles.end()&&at->second.root==key(root))at->second.closed=true;
}
void NativeStream::ServerRetired(const CSE_Abstract& object)noexcept{
    auto& i=*impl_;if(object.ID==0xffff)return;std::lock_guard<std::mutex> lock(i.metadata);
    auto& at=i.server[object.ID];if(at.birth.server_birth==object.PainterBirthCookie()){
        at.active=false;const auto bundle=i.bundles.find(object.ID);
        if(bundle!=i.bundles.end()&&bundle->second.root==key(at.birth))i.bundles.erase(bundle);
    }
}
void NativeStream::PacketQueued(const NET_Packet& packet){
    auto& i=*impl_;if(i.stopped||stream_for(local_delivery_server)!=this)return;
    std::lock_guard<std::mutex> lock(i.metadata);
    R_ASSERT2(i.received.find(&packet)==i.received.end(),"Native receive metadata pointer reused before release");
    std::vector<Impl::ReceivedRecord> records;
    const auto* data=packet.B.data;const auto size=packet.B.count;
    if(size<2||size>NET_PacketSizeLimit)return;
    u16 type;std::memcpy(&type,data,2);
    auto append=[&](std::size_t offset,std::size_t bytes){
        const auto stamp=i.authenticate(data+offset,bytes);
        // Preserve the exact native packed-message ordinal, including messages
        // outside the owned schema. Never search later identical bytes.
        records.push_back({offset,bytes,stamp});
    };
    if(type==M_EVENT_PACK){
        for(std::size_t offset=2;offset<size;){const auto bytes=data[offset++];
            if(!bytes||bytes>size-offset)return;append(offset,bytes);offset+=bytes;}
    }else append(0,size);
    i.received.emplace(&packet,std::move(records));
}
void NativeStream::BeginReceive(const NET_Packet& packet)noexcept{
    receive_stream=this;receive_packet=&packet;receive_record=0;
}
void NativeStream::EndReceive(const NET_Packet& packet)noexcept{
    auto& i=*impl_;std::lock_guard<std::mutex> lock(i.metadata);i.received.erase(&packet);
    if(receive_stream==this&&receive_packet==&packet){receive_stream=nullptr;receive_packet=nullptr;receive_record=0;}
}
void NativeStream::Capture(void* context,const NET_Packet& packet,NET_Event& event)noexcept{
    auto* stream=static_cast<NativeStream*>(context);auto& i=*stream->impl_;event.painter_stamp={};
    if(receive_stream!=stream||!receive_packet)return;
    std::lock_guard<std::mutex> lock(i.metadata);const auto source=i.received.find(receive_packet);
    if(source==i.received.end()||receive_record>=source->second.size()||i.arrival==UINT64_MAX)return;
    const auto& record=source->second[receive_record++];
    if(record.size!=packet.B.count||record.offset+record.size>receive_packet->B.count||
        std::memcmp(receive_packet->B.data+record.offset,packet.B.data,record.size)!=0)return;
    if(record.stamp.authenticated){event.painter_stamp=record.stamp;event.painter_stamp.arrival=++i.arrival;}
}
bool NativeStream::Ordinary(const NET_Event& event)const noexcept{return !impl_->stopped&&event.painter_stamp.authenticated&&event.painter_stamp.ordinary&&event.painter_stamp.owner.world==impl_->world;}
bool NativeStream::Pending(std::uint16_t id)const noexcept{
    std::lock_guard<std::recursive_mutex> lock(impl_->owner);
    const auto at=impl_->life(id);return impl_->spawn(key(at.birth))!=nullptr;
}
Admission NativeStream::Admit(NET_Event& event,bool prefetched)noexcept{
    auto& i=*impl_;std::lock_guard<std::recursive_mutex> lock(i.owner);if(i.stopped||i.dispatching)return Admission::Forward;
    const auto& stamp=event.painter_stamp;
    if(!stamp.authenticated||!valid(stamp.owner)||stamp.owner.world!=i.world)return Admission::Forward;
    if(i.level.PainterNativeBacklogBefore(event))return Admission::Retry;
    sss24_stream::Frame frame;if(!i.frame(frame))return Admission::Forward;
    const auto callbacks=i.callbacks();i.admit_backlog(callbacks);i.promote_optional(stamp,frame,callbacks);
    bool needs=false;
    if(event.ID==M_SPAWN)needs=prefetched&&stamp.ordinary;
    else if(event.ID==M_EVENT)needs=i.waiting(stamp.owner)||i.pending_traffic(key(stamp.owner));
    else return Admission::Forward;
    for(unsigned d=0;d<stamp.dependency_count;++d)needs=needs||i.waiting(stamp.dependencies[d])||i.queue.HasPendingSpawn(key(stamp.dependencies[d]));
    if(!needs)return Admission::Forward;
    sss24_stream::Packet packet;packet.owner=key(stamp.owner);packet.kind=event.ID==M_SPAWN?sss24_stream::Kind::Spawn:sss24_stream::Kind::Event;
    const auto at=i.life(stamp.owner.id);packet.position=at.position;packet.ordinary_ready=true;
    for(unsigned d=0;d<stamp.dependency_count;++d)packet.dependencies[packet.dependency_count++]=key(stamp.dependencies[d]);
    if(!clock_us(nullptr,packet.arrived_us))return Admission::Retry;
    packet.token=1; // Temporary only for dependency-closure validation below.
    packet.spawn_budget=!valid(at.bundle)||key(at.bundle)==packet.owner;
    if(stamp.critical||!stamp.ordinary){
        i.promote_prerequisite(packet.owner,frame,callbacks);
        for(unsigned d=0;d<packet.dependency_count;++d)i.promote_prerequisite(packet.dependencies[d],frame,callbacks);
        const auto drained=i.queue.DrainRequired(packet,frame,callbacks);i.finish_bundles(frame,callbacks);
        if(!drained.incomplete&&!i.pending_traffic(key(stamp.owner)))return Admission::Forward;
    }
    if(i.token>=(UINT64_MAX>>17))return Admission::Retry;
    if(i.free_slots.empty()){i.queue.DrainRecovery(frame,callbacks);i.finish_bundles(frame,callbacks);}
    if(i.free_slots.empty())return Admission::Retry;
    const bool prior_backlog=i.unadmitted_traffic(packet.owner);
    const auto index=i.free_slots.back();i.free_slots.pop_back();auto* stored=&i.stored[index];
    packet.token=(++i.token<<17)|index;stored->used=true;stored->token=packet.token;
    stored->critical=stamp.critical||!stamp.ordinary;stored->admitted=false;stored->arrived_us=packet.arrived_us;
    stored->event=std::move(event);stored->active_position=i.active_slots.size();i.active_slots.push_back(index);
    ++i.owner_traffic[packet.owner.id];if(packet.kind==sss24_stream::Kind::Spawn)i.spawn_slot[packet.owner.id]=index;
    const auto accepted=prior_backlog?sss24_stream::Admission::NeedDrain:i.queue.Enqueue(packet);
    if(accepted==sss24_stream::Admission::Queued){stored->admitted=true;return Admission::Held;}
    if(accepted==sss24_stream::Admission::NeedDrain)return Admission::Held;
    const auto retained_stamp=stored->event.painter_stamp;event=std::move(stored->event);stored->event.painter_stamp=retained_stamp;i.release(*stored);
    // Caller retains this exact event in its native game/ready queue. No loss,
    // arbitrary flush, body-ID substitution or synchronous dependency overtake.
    return Admission::Retry;
}
void NativeStream::SpawnApplied(const NET_Event& event)noexcept{std::lock_guard<std::recursive_mutex> lock(impl_->owner);impl_->spawn_applied(event);}
void NativeStream::Advance()noexcept{
    auto& i=*impl_;std::lock_guard<std::recursive_mutex> lock(i.owner);sss24_stream::Frame frame;if(!i.frame(frame)||i.dispatching)return;
    const auto callbacks=i.callbacks();i.admit_backlog(callbacks);
    // Synchronous/critical traffic may be retained only while its genuine
    // native prerequisites are still being prefetched. Promote that closure.
    for(std::size_t n=0,visited=0;n<i.active_slots.size()&&visited++<Impl::retained_capacity;){
        const auto index=i.active_slots[n];auto& row=i.stored[index];
        if(!row.critical||!row.admitted){++n;continue;}
        const auto frozen_stamp=row.event.painter_stamp;
        i.promote_optional(frozen_stamp,frame,callbacks);
        if(!row.used){continue;}
        const auto frozen_token=row.token;
        i.promote_prerequisite(key(frozen_stamp.owner),frame,callbacks);
        for(unsigned d=0;d<frozen_stamp.dependency_count;++d)i.promote_prerequisite(key(frozen_stamp.dependencies[d]),frame,callbacks);
        if(!i.find(frozen_token))continue;
        sss24_stream::Packet critical;critical.token=row.token;critical.owner=key(row.event.painter_stamp.owner);
        critical.kind=row.event.ID==M_SPAWN?sss24_stream::Kind::Spawn:sss24_stream::Kind::Event;critical.ordinary_ready=true;
        critical.position=i.life(critical.owner.id).position;
        for(unsigned d=0;d<row.event.painter_stamp.dependency_count;++d)critical.dependencies[critical.dependency_count++]=key(row.event.painter_stamp.dependencies[d]);
        i.queue.DrainRequired(critical,frame,callbacks);i.finish_bundles(frame,callbacks);
        if(n<i.active_slots.size()&&i.active_slots[n]==index)++n;
    }
    i.queue.DrainFrame(frame,callbacks);i.finish_bundles(frame,callbacks);i.admit_backlog(callbacks);
}
void NativeStream::WorldStopped()noexcept{
    auto& i=*impl_;std::lock_guard<std::recursive_mutex> owner(i.owner);if(i.stopped)return;i.stopped=true;
    i.queue.RetireWorld(i.world,i.callbacks());
    std::lock_guard<std::mutex> lock(i.metadata);
    i.received.clear();i.bundles.clear();i.born_count=0;
    while(!i.active_slots.empty())i.release(i.stored[i.active_slots.back()]);
}
namespace {
NativeStream* stream_for(xrServer* server)noexcept{
    // The server exists before the client Game() during bootstrap.
    if(!g_pGameLevel||!server||!server->game||server->game->Type()!=eGameIDSingle||Level().Server!=server)return nullptr;
    return Level().PainterSpawnStream();
}
}
void ServerActivation(xrServer& server,CSE_Abstract& object,bool existing)noexcept{if(auto* stream=stream_for(&server))stream->ServerActivation(object,existing);}
void ServerRetired(xrServer& server,const CSE_Abstract& object)noexcept{if(auto* stream=stream_for(&server))stream->ServerRetired(object);}
void Offline(const CSE_ALifeDynamicObject& object)noexcept{if(g_pGameLevel&&Level().Server)ServerRetired(*Level().Server,object);}
}
