#pragma once
// Read-only posed-mesh query, authorized only by the CURRENT synchronous native hit callback.
#include "Hit.h"
#include "GameObject.h"
#include "Level.h"
#include "../Include/xrRender/Kinematics.h"
#include "../xrEngine/bone.h"
#include "../xrEngine/device.h"
#include "../Layers/xrRender/PainterHitMain.h" // triggerhitch
#include <array>
#include <cstdint>
#include <cmath>
#include <cstring>

namespace painter_exit_surface {
inline bool finite(const Fvector& p) { return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z); }
inline bool unit(Fvector& p) { const float n=p.square_magnitude();if(!finite(p)||!std::isfinite(n)||n<1e-12f)return false;p.mul(1.f/std::sqrt(n));return true; }
struct Context {
    u32 owner,bone,bullet,initiator,weapon;u64 birth,world,generation;std::uintptr_t visual;
    int type;Fvector contact,direction;char ammo[128]{};bool known_ammo;
    Context()=default;
    Context(CObject& o,const SHit& h):owner(o.ID()),bone(h.boneID),bullet(h.BulletID),initiator(h.whoID),
        weapon(h.weaponID),birth(o.PainterBirthCookie()),world(Level().PainterWorldCookie()),generation(0),
        visual(reinterpret_cast<std::uintptr_t>(o.Visual())),type(h.hit_type),contact(h.p_in_bone_space),
        direction(h.dir),known_ammo(h.ammo.valid()) {
        if(auto* k=PKinematics(o.Visual()))generation=k->LL_SurfaceGeometryGeneration();
        if(known_ammo)std::memcpy(ammo,h.ammo.section,sizeof(ammo));
    }
};
inline thread_local const Context* current=nullptr;
class Scope {
    const Context value_;const Context* previous_;
public:
    Scope(CObject& owner,const SHit& hit):value_(owner,hit),previous_(current){current=&value_;}
    ~Scope(){current=previous_;}
    Scope(const Scope&)=delete;Scope& operator=(const Scope&)=delete;
};
struct Request {u32 owner,bone,bullet,initiator,weapon;Fvector incoming;const char* ammo=nullptr;bool capture_only=false;u64 capture_token=0;};
struct Reply {Fvector point,normal;u64 triangle=0,capture_token=0;};
inline bool matches(const Context& c,const Request& r) {
    if(r.owner!=c.owner||r.bone!=c.bone||r.bullet!=c.bullet||r.initiator!=c.initiator||r.weapon!=c.weapon||
        c.type!=ALife::eHitTypeFireWound||!finite(c.contact))return false;
    Fvector a=r.incoming,b=c.direction;if(!unit(a)||!unit(b)||a.dotproduct(b)<.9995f)return false;
    return !r.ammo||(c.known_ammo&&std::strcmp(r.ammo,c.ammo)==0);
}
inline bool authenticate(const Context& c,const Request& r,CObject* live,u64 world) {
    return live&&!live->getDestroy()&&live->ID()==c.owner&&live->PainterBirthCookie()==c.birth&&world==c.world&&
        reinterpret_cast<std::uintptr_t>(live->Visual())==c.visual&&matches(c,r);
}
// Scalar-only deferred receipts: no CObject, SHit or borrowed string is retained. Tokens never repeat.
struct Receipt {Context hit{};u64 token=0;u32 issued=0;};
inline std::array<Receipt,128> receipts;inline xrCriticalSection receipt_mutex;inline u64 next_token=0;
inline bool alive(u32 now,u32 issued){return u32(now-issued)<1000u;}
inline bool capture(const Context& c,u32 now,u64& token) {
    xrCriticalSectionGuard lock(&receipt_mutex);
    for(auto& row:receipts){if(row.token&&!alive(now,row.issued))row.token=0;
        if(!row.token){if(next_token==((1ULL<<53)-1))return false;row.token=++next_token;row.issued=now;
            row.hit=c;token=row.token;return true;}}
    return false; // Never evict an admitted, unexpired receipt.
}
inline bool take(const Request& r,u32 now,Context& c) {
    xrCriticalSectionGuard lock(&receipt_mutex);
    for(auto& row:receipts)if(row.token==r.capture_token){
        if(!alive(now,row.issued)){row.token=0;return false;}
        c=row.hit;if(!matches(c,r))return false;
        row.token=0;return true; // One budgeted replay attempt, including a later mesh rejection.
    }
    return false;
}
// Native PickBone is first-face/first-child, but every accepted distance is STRICTLY below its bound.
// Exhaust decreasing bounds to prove the actual closest hit. Never publish a partial/budget-limited answer.
template<class Pick> inline bool nearest(u16 count,float range,Pick pick,IKinematics::pick_result& best) {
    bool found=false;unsigned calls=0;
    for(u16 bone=0;bone<count;++bone)for(;;){
        if(++calls>1024)return false;
        IKinematics::pick_result candidate{};
        if(!pick(bone,range,candidate))break;
        if(!std::isfinite(candidate.dist)||candidate.dist<0.f||candidate.dist>=range)return false;
        range=candidate.dist;best=candidate;found=true;
    }
    return found;
}
inline bool finish(const Fvector& start,const Fvector& back,const Fvector& contact,const Fvector& incoming,
    float range,const IKinematics::pick_result& pick,Reply& out) {
    if(!std::isfinite(pick.dist)||pick.dist<0.f||pick.dist>=range)return false;
    for(const auto& p:pick.tri)if(!finite(p))return false;
    Fvector e1,e2,n,p,v;e1.sub(pick.tri[1],pick.tri[0]);e2.sub(pick.tri[2],pick.tri[0]);n.crossproduct(e1,e2);
    if(!unit(n)||n.dotproduct(incoming)<=1e-5f)return false;
    p.mad(start,back,pick.dist);v.sub(p,pick.tri[0]);
    if(!finite(p)||std::abs(v.dotproduct(n))>1e-4f)return false;
    const double a=e1.dotproduct(e1),b=e1.dotproduct(e2),c=e2.dotproduct(e2),d=v.dotproduct(e1),e=v.dotproduct(e2);
    const double den=a*c-b*b;if(!std::isfinite(den)||den<1e-20)return false;
    const double u=(c*d-b*e)/den,w=(a*e-b*d)/den;
    if(u< -1e-5||w< -1e-5||u+w>1.00001)return false;
    v.sub(p,contact);const float forward=v.dotproduct(incoming);
    if(forward<=.001f||forward>4.f)return false;
    u64 witness=1469598103934665603ULL;
    for(const auto& vertex:pick.tri){const auto* bytes=reinterpret_cast<const unsigned char*>(&vertex);
        for(unsigned i=0;i<sizeof(vertex);++i){witness^=bytes[i];witness*=1099511628211ULL;}}
    out.point=p;out.normal=n;out.triangle=(witness&((1ULL<<53)-1))|1;return true;
}
inline bool query(const Request& r,Reply& out) {
    painter_hit_main::PartScope hit_main_surface(painter_hit_main::Surface);
    out=Reply{};if(!g_pGameLevel||(!current&&!r.capture_token)||r.owner>=u16(-1)||(r.capture_only&&r.capture_token))return false;
    CObject* owner=Level().Objects.net_Find(u16(r.owner));
    if(!owner)return false;
    Context saved{};const Context* c=current;
    if(r.capture_token){if(!take(r,Device.dwTimeContinual,saved))return false;c=&saved;}
    if(!c||!authenticate(*c,r,owner,Level().PainterWorldCookie()))return false;
    IKinematics* k=PKinematics(owner->Visual());if(!k)return false;
    if(r.capture_only)return capture(*c,Device.dwTimeContinual,out.capture_token);
    xrCriticalSectionGuard pose(&k->UCalc_Mutex);
    const u16 count=k->LL_BoneCount();
    if(!count||count>256||r.bone>=count||!k->LL_GetBoneVisible(u16(r.bone))||
        k->LL_SurfaceGeometryGeneration()!=c->generation)return false;
    k->CalculateBones(TRUE);
    const Fmatrix world=owner->XFORM();Fvector contact,model,direction=c->direction;
    if(!unit(direction)||!finite(c->contact))return false;
    k->LL_GetBoneInstance(u16(r.bone)).mTransform.transform_tiny(model,c->contact);
    world.transform_tiny(contact,model);if(!finite(contact))return false;
    // A positive projection beyond ALL posed bone OBB corners is outside the target along this shot line.
    float reach=0.f;
    for(u16 bone=0;bone<count;++bone)if(k->LL_GetBoneVisible(bone)){
        Fmatrix box,posed,absolute;k->LL_GetData(bone).obb.xform_full(box);
        posed.mul_43(k->LL_GetBoneInstance(bone).mTransform,box);absolute.mul_43(world,posed);
        Fvector delta;delta.sub(absolute.c,contact);
        const float end=delta.dotproduct(direction)+std::abs(absolute.i.dotproduct(direction))+
            std::abs(absolute.j.dotproduct(direction))+std::abs(absolute.k.dotproduct(direction));
        if(!std::isfinite(end))return false;if(end>reach)reach=end;
    }
    reach+=.05f;if(reach<=.05f||reach>4.f)return false;
    Fvector start,back;start.mad(contact,direction,reach);back.invert(direction);
    IKinematics::pick_result pick{};
    const int one_pass=k->LL_PickNearest(world,pick,reach,start,back,count); // triggerhitch: the same face in one pass
    if(!one_pass||(one_pass<0&&!nearest(count,reach,[&](u16 bone,float bound,IKinematics::pick_result& value){
        return k->LL_GetBoneVisible(bone)&&k->PickBone(world,value,bound,start,back,bone);
    },pick)))return false;
    return finish(start,back,contact,direction,reach,pick,out);
}
// Entry face (R18): the posed-mesh triangle the shot went IN through, for the impact particles. The same
// authenticated current hit, cast from outside the body forward along the shot. Lua: level.painter_native_surface_hit.
inline bool finish_entry(const Fvector& start,const Fvector& forward,const Fvector& contact,const Fvector& incoming,
    float range,const IKinematics::pick_result& pick,Reply& out) {
    if(!std::isfinite(pick.dist)||pick.dist<0.f||pick.dist>=range)return false;
    for(const auto& p:pick.tri)if(!finite(p))return false;
    Fvector e1,e2,n,p,v;e1.sub(pick.tri[1],pick.tri[0]);e2.sub(pick.tri[2],pick.tri[0]);n.crossproduct(e1,e2);
    if(!unit(n))return false;
    if(n.dotproduct(incoming)>0.f)n.invert(); // the entry face looks back at the shooter
    p.mad(start,forward,pick.dist);v.sub(p,pick.tri[0]);
    if(!finite(p)||std::abs(v.dotproduct(n))>1e-4f)return false;
    v.sub(p,contact);const float ahead=v.dotproduct(incoming); // negative: before the engine's contact point
    if(ahead>.05f||ahead< -.5f)return false;
    u64 witness=1469598103934665603ULL;
    for(const auto& vertex:pick.tri){const auto* bytes=reinterpret_cast<const unsigned char*>(&vertex);
        for(unsigned i=0;i<sizeof(vertex);++i){witness^=bytes[i];witness*=1099511628211ULL;}}
    out.point=p;out.normal=n;out.triangle=(witness&((1ULL<<53)-1))|1;return true;
}
inline bool query_entry(const Request& r,Reply& out) {
    painter_hit_main::PartScope hit_main_surface(painter_hit_main::Surface);
    out=Reply{};if(!g_pGameLevel||!current||r.owner>=u16(-1)||r.capture_only||r.capture_token)return false;
    CObject* owner=Level().Objects.net_Find(u16(r.owner));
    if(!owner)return false;
    const Context* c=current;
    if(!authenticate(*c,r,owner,Level().PainterWorldCookie()))return false;
    IKinematics* k=PKinematics(owner->Visual());if(!k)return false;
    xrCriticalSectionGuard pose(&k->UCalc_Mutex);
    const u16 count=k->LL_BoneCount();
    if(!count||count>256||r.bone>=count||!k->LL_GetBoneVisible(u16(r.bone))||
        k->LL_SurfaceGeometryGeneration()!=c->generation)return false;
    k->CalculateBones(TRUE);
    const Fmatrix world=owner->XFORM();Fvector contact,model,direction=c->direction;
    if(!unit(direction)||!finite(c->contact))return false;
    k->LL_GetBoneInstance(u16(r.bone)).mTransform.transform_tiny(model,c->contact);
    world.transform_tiny(contact,model);if(!finite(contact))return false;
    // How far the posed bone boxes reach back from the contact against the shot: start outside them all.
    float reach=0.f;
    for(u16 bone=0;bone<count;++bone)if(k->LL_GetBoneVisible(bone)){
        Fmatrix box,posed,absolute;k->LL_GetData(bone).obb.xform_full(box);
        posed.mul_43(k->LL_GetBoneInstance(bone).mTransform,box);absolute.mul_43(world,posed);
        Fvector delta;delta.sub(contact,absolute.c);
        const float end=delta.dotproduct(direction)+std::abs(absolute.i.dotproduct(direction))+
            std::abs(absolute.j.dotproduct(direction))+std::abs(absolute.k.dotproduct(direction));
        if(!std::isfinite(end))return false;if(end>reach)reach=end;
    }
    reach+=.05f;if(reach<=.05f||reach>4.f)return false;
    Fvector start;start.mad(contact,direction,-reach);const float range=reach+.05f;
    IKinematics::pick_result pick{};
    const int one_pass=k->LL_PickNearest(world,pick,range,start,direction,count); // triggerhitch: the same face in one pass
    if(!one_pass||(one_pass<0&&!nearest(count,range,[&](u16 bone,float bound,IKinematics::pick_result& value){
        return k->LL_GetBoneVisible(bone)&&k->PickBone(world,value,bound,start,direction,bone);
    },pick)))return false;
    return finish_entry(start,direction,contact,direction,range,pick,out);
}
}
