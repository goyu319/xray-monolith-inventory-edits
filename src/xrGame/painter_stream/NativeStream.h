#pragma once
#include "Stamp.h"
#include <cstdint>
class CLevel;
class CSE_Abstract;
class CSE_ALifeDynamicObject;
class NET_Packet;
class NET_Event;
class xrServer;

namespace painter_stream {
// Marks only the ordinary, distance-approved switch branch. Forced online
// requests never construct this scope. Native existing group members/inventory
// children are authenticated by their native root/group/parent relationship.
class OrdinaryOnlineScope final {
public:
    explicit OrdinaryOnlineScope(CSE_ALifeDynamicObject&)noexcept;
    ~OrdinaryOnlineScope();
private:
    const CSE_ALifeDynamicObject* previous_{};
};
// add_online is already inside the native ordinary switch. Group members each
// receive this scope separately, keeping NPC + inventory atomic, not whole squads.
class OnlineBundleScope final {
public:
    explicit OnlineBundleScope(CSE_ALifeDynamicObject&)noexcept;
    ~OnlineBundleScope();
private:
    const CSE_Abstract* previous_{};const CSE_Abstract* root_{};
};

// Authentication exists only during the actual local server delivery call.
// Metadata is bound to the INetQueue packet's own lifetime, not a finite FIFO
// searched later by possibly repeated packet bytes.
class LocalDeliveryScope final {
public:
    explicit LocalDeliveryScope(xrServer&)noexcept;
    ~LocalDeliveryScope();
private:xrServer* previous_{};
};
enum class Admission {Forward,Held,Retry};
class NativeStream final {
public:
    explicit NativeStream(CLevel&);
    ~NativeStream();
    NativeStream(const NativeStream&)=delete;
    NativeStream& operator=(const NativeStream&)=delete;
    void ServerActivation(CSE_Abstract&,bool existing)noexcept;
    void ServerRetired(const CSE_Abstract&)noexcept;
    void CloseBundle(const CSE_Abstract&)noexcept;
    void PacketQueued(const NET_Packet&);
    void BeginReceive(const NET_Packet&)noexcept;
    void EndReceive(const NET_Packet&)noexcept;
    static void Capture(void*,const NET_Packet&,NET_Event&)noexcept;
    bool Ordinary(const NET_Event&)const noexcept;
    bool Pending(std::uint16_t)const noexcept;
    Admission Admit(NET_Event&,bool prefetched)noexcept;
    void SpawnApplied(const NET_Event&)noexcept;
    void Advance()noexcept;
    void WorldStopped()noexcept;
private:
    struct Impl;Impl* impl_;
};
void ServerActivation(xrServer&,CSE_Abstract&,bool existing)noexcept;
void ServerRetired(xrServer&,const CSE_Abstract&)noexcept;

void Offline(const CSE_ALifeDynamicObject&)noexcept;
}
