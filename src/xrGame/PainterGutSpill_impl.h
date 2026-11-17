#pragma once
#include <deque>
#include <cstring>
#include <cmath>
#include "painter_voxel/GoreControls.h"
#include "../xrphysics/ExtendedGeom.h"
namespace painter_gut_spill {
struct Part {CPhysicsShellHolder* obj=nullptr;xr_vector<u16> bones; // the bones the part holds (its elements and posed bones)
    bool holds(u16 b)const{return b!=BI_NONE&&std::find(bones.begin(),bones.end(),b)!=bones.end();}};
namespace detail {
struct Settings {float chance=.65f,chance_blast=.85f,break_force=900.f,wound_chance=1.f;u32 max_ropes=3,max_live=10;};
inline const Settings& settings(){static const Settings s=[]{Settings s;const char* sec="painter_gut_spill";
    if(pSettings&&pSettings->section_exist(sec)){
        if(pSettings->line_exist(sec,"chance"))s.chance=pSettings->r_float(sec,"chance");
        if(pSettings->line_exist(sec,"chance_blast"))s.chance_blast=pSettings->r_float(sec,"chance_blast");
        if(pSettings->line_exist(sec,"break_force"))s.break_force=pSettings->r_float(sec,"break_force");
        if(pSettings->line_exist(sec,"wound_chance"))s.wound_chance=pSettings->r_float(sec,"wound_chance");
        if(pSettings->line_exist(sec,"max_ropes"))s.max_ropes=pSettings->r_u32(sec,"max_ropes");
        if(pSettings->line_exist(sec,"max_live"))s.max_live=pSettings->r_u32(sec,"max_live");}
    s.chance=_max(0.f,_min(1.f,s.chance));s.chance_blast=_max(0.f,_min(1.f,s.chance_blast));s.break_force=_max(20.f,s.break_force);
    s.wound_chance=_max(0.f,_min(4.f,s.wound_chance));s.max_ropes=_min(s.max_ropes,3u);s.max_live=_max(1u,_min(s.max_live,64u));return s;}();return s;}
struct Variant {const char* visual;bool loop;float mass,radius;};
inline const Variant& variant(u32 i){
    static const Variant v[]={{"painter\\anatomy\\gut_rope_01",true,.18f,.017f},{"painter\\anatomy\\gut_rope_02",true,.18f,.017f},
                              {"painter\\anatomy\\gut_rope_03",false,.18f,.017f},{"painter\\anatomy\\gut_rope_04",false,.25f,.024f}};
    return v[i<4?i:0];}
struct Arm {u16 bone=BI_NONE;Fvector point{},axis{};u32 time=0;gfy_live::GoreControls controls;CGameObject* source_identity=nullptr;};
struct Pending {
    u16 host=u16(-1),host_bone=BI_NONE;Fmatrix local;CGID group=0;u32 at=0,variant=0;shared_str visual;bool wound=false;
    CPhysicsShellHolder* host_identity=nullptr;CPhysicsShell* host_shell=nullptr;
    gfy_live::GoreControls controls;float break_force=900.f,mouth_radius=.03f;
};
struct Live {
    u16 id=u16(-1),host=u16(-1),host_bone=BI_NONE;IPHTether* tether[2]={nullptr,nullptr};u16 endpoint[2]={BI_NONE,BI_NONE};
    Fvector local_anchor[2];CPhysicsShell* host_shell=nullptr;float mouth_radius=.03f;u32 born=0;
};
struct Wound {
    u16 bone=BI_NONE;Fvector point{},normal{};u32 time=0,count=1;bool used=false;
    gfy_live::GoreControls controls;float mouth_radius=.03f,break_force=900.f;CGameObject* source_identity=nullptr;
};
inline std::map<u16,Arm> arms;              // torso tear (sever point, axis) per body, consumed by its next split
inline std::map<std::string,Pending> pending; // ropes sent to the server, by unique name
inline std::map<u16,Pending> spawning;      // ropes between begin_spawn and end_spawn
inline std::deque<Live> live;                // ropes in the world, oldest first
inline std::map<u16,Wound> wounds;           // abdominal wound mouths waiting for the ragdoll (bone frame)
inline u32 serial=0;
constexpr u32 max_event_ropes=9;
inline bool finite(const Fvector& v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
inline bool valid_controls(const gfy_live::GoreControls& c){
    return gfy_live::valid_gore_controls(c);
}
inline bool enabled(const gfy_live::GoreControls& c){return valid_controls(c)&&c.intestines_enabled&&c.intestine_amount_mult>0&&c.intestine_frequency_mult>0;}
inline float break_force(const gfy_live::GoreControls& c){return settings().break_force*float(c.intestine_tether_strength_mult);}
// At multiplier1 there is no extra draw and the existing base count is exact.
inline u32 amount(u32 base,const gfy_live::GoreControls& c){
    if(!enabled(c)||!base)return 0;const double scaled=double(base)*c.intestine_amount_mult;
    u32 count=u32(scaled);const double fraction=scaled-count;
    if(fraction>0&&::Random.randF(0.f,1.f)<fraction)++count;
    return _min(count,max_event_ropes);
}
// A rope element's spin limit. The element default (9.8 rad/s, PHElement CutVelocity) is far below what a 9 cm link of a
// swinging chain needs; the cut fights the joints and the links part (the stretching seen in the 2026-09-18 test).
constexpr float rope_w_limit=40.f;
// Angular air resistance of a link, applied explicitly after the step (PHElement.cpp:550): k*dt/I <= .3 about the link
// axis at a 0.02 s step (the element default .05 overshoots a link's inertia hundreds of times).
inline float air_w(const Variant& v){return .3f*v.mass*.5f*v.radius*v.radius/.02f;}
inline void release(Live& l){for(auto*& t:l.tether)if(t)phtether_destroy(t);}
inline void retire_oldest(){
    auto& l=live.front();release(l);
    if(auto* o=smart_cast<CGameObject*>(Level().Objects.net_Find(l.id)))if(!o->getDestroy())o->DestroyObject();
    live.pop_front();}
inline void purge(){const u32 now=Device.dwTimeGlobal;
    for(auto& l:live)for(auto*& t:l.tether)if(t&&!t->Alive())phtether_destroy(t);
    for(auto it=pending.begin();it!=pending.end();)if(now-it->second.at>10000)it=pending.erase(it);else ++it;
    for(auto it=spawning.begin();it!=spawning.end();)if(now-it->second.at>10000)it=spawning.erase(it);else ++it;
    for(auto it=arms.begin();it!=arms.end();)if(now-it->second.time>60000)it=arms.erase(it);else ++it;
    for(auto it=wounds.begin();it!=wounds.end();)if(now-it->second.time>(it->second.used?600000u:60000u))it=wounds.erase(it);else ++it;}
// Pending server spawns reserve their slot too; a burst cannot bypass max_live
// while its ropes have not reached net_Spawn yet.
inline u32 capacity(u32 wanted){
    const std::size_t waiting=pending.size()+spawning.size();
    while(!live.empty()&&live.size()+waiting+wanted>settings().max_live)retire_oldest();
    const std::size_t occupied=live.size()+waiting;
    return occupied>=settings().max_live?0:_min(wanted,u32(settings().max_live-occupied));
}
// A wound loop's two links in the wound mouth start inside their body and stay tethered there: they never collide with
// it. The rest of the loop does, so it lies on the body instead of falling through it.
inline void mouth_contact(bool& do_colide,bool bo1,dContact& c,SGameMtl*,SGameMtl*){
    dxGeomUserData *mine=nullptr,*other=nullptr;get_user_data(mine,other,bo1,c.geom);
    if(!mine||!other||!mine->ph_ref_object||!other->ph_ref_object)return;
    const u16 rope=mine->ph_ref_object->ObjectID(),body=other->ph_ref_object->ObjectID();
    for(const auto& l:live)if(l.id==rope){
        if(l.host!=body||!l.host_shell)return;
        for(unsigned i=0;i<2;++i)if(l.endpoint[i]==mine->bone_id&&l.tether[i]&&l.tether[i]->Alive()){
            auto* host=smart_cast<CPhysicsShellHolder*>(Level().Objects.net_Find(body));
            if(!host||host->getDestroy()||host->PPhysicsShell()!=l.host_shell)return;
            auto* he=l.host_shell->get_Element(l.host_bone);if(!he)return;
            Fmatrix hx;he->GetGlobalTransformDynamic(&hx);Fvector anchor;hx.transform_tiny(anchor,l.local_anchor[i]);
            Fvector contact;contact.set(c.geom.pos[0],c.geom.pos[1],c.geom.pos[2]);
            // Only the tethered mouth-sized patch skips its own body. A freed
            // endpoint, another link, or a contact elsewhere still collides.
            if(anchor.distance_to_sqr(contact)<=l.mouth_radius*l.mouth_radius)do_colide=false;
            return;}
        return;}}
// The rope's frame: root at `root`, its coil (-Y) along `out`, turned `twist` about it.
inline Fmatrix rope_frame(const Fvector& root,const Fvector& out,float twist){
    Fvector a;a.set(0.f,1.f,0.f);if(_abs(out.y)>.9f)a.set(1.f,0.f,0.f);Fvector e1,e2;e1.crossproduct(a,out).normalize();e2.crossproduct(out,e1).normalize();
    Fmatrix rope;rope.identity();rope.j.set(out).invert();
    rope.i.set(e1).mul(_cos(twist)).mad(e2,_sin(twist));rope.k.crossproduct(rope.i,rope.j).normalize();rope.i.crossproduct(rope.j,rope.k).normalize();
    rope.c.set(root);return rope;}
// Sends one rope to the server; it is placed on `he` (host element) as it will be when it spawns.
inline bool send(CGameObject& source,CPhysicsShellHolder& host,CPhysicsElement* he,u16 host_bone,const Fmatrix& rope,u32 v,CGID group,
                 const gfy_live::GoreControls& controls,float force,bool wound=false,float mouth_radius=.03f){
    CPhysicsShell* hs=host.PPhysicsShell();
    if(!enabled(controls)||!he||!hs||host_bone==BI_NONE||hs->get_Element(host_bone)!=he||
        !finite(rope.c)||!finite(rope.i)||!finite(rope.j)||!finite(rope.k)||!std::isfinite(force)||force<=0)return false;
    Fmatrix hx,inv;he->GetGlobalTransformDynamic(&hx);inv.invert(hx);
    Pending pnd;pnd.host=host.ID();pnd.host_bone=host_bone;pnd.local.mul_43(inv,rope);pnd.group=group;pnd.at=Device.dwTimeGlobal;
    pnd.variant=v;pnd.visual=variant(v).visual;pnd.wound=wound;pnd.controls=controls;pnd.break_force=force;
    pnd.host_identity=&host;pnd.host_shell=hs;pnd.mouth_radius=mouth_radius;
    const std::string name="painter_gut_"+std::to_string(++serial);
    CSE_Abstract* se=Level().spawn_item("ph_skeleton_object",rope.c,source.ai_location().level_vertex_id(),u16(-1),true);
    auto* po=smart_cast<CSE_ALifePHSkeletonObject*>(se);
    if(!po){if(se)F_entity_Destroy(se);Msg("[painter_gut_spill] id=%u phase=spawn_failed reason=server_class",source.ID());return false;}
    po->set_visual(variant(v).visual);po->set_name_replace(name.c_str());po->source_id=BI_NONE;
    po->saved_bones.root_bone=0;po->saved_bones.bones_mask=u64(-1);
    po->_flags.set(CSE_PHSkeleton::flActive,TRUE);po->_flags.set(CSE_PHSkeleton::flNotSave,TRUE);
    rope.getHPB(po->o_Angle);po->o_Position=rope.c;
    NET_Packet packet;po->Spawn_Write(packet,TRUE);
    // Publish the authenticated pending record before a possible synchronous
    // spawn callback consumes the packet.
    try{pending.emplace(name,pnd);}catch(...){F_entity_Destroy(se);return false;}
    Level().Send(packet,net_flags(TRUE));F_entity_Destroy(se);return true;}
}
void arm(u16 source,u16 bone,const Fvector& rest_point,const Fvector& rest_axis,const gfy_live::GoreControls& controls){
    if(!detail::valid_controls(controls)||!detail::finite(rest_point)||!detail::finite(rest_axis))return;
    auto* object=smart_cast<CGameObject*>(Level().Objects.net_Find(source));if(!object)return;
    auto& a=detail::arms[source];a.bone=bone;a.point=rest_point;a.axis=rest_axis;a.time=Device.dwTimeGlobal;a.controls=controls;a.source_identity=object;}
void arm(u16 source,u16 bone,const Fvector& rest_point,const Fvector& rest_axis){arm(source,bone,rest_point,rest_axis,gfy_live::GoreControls{});}
// At a split commit: `parts` are the stump and the new pieces (all at the source's pose), `group` their collision group.
inline void split_committed(CAI_Stalker& source,IKinematics& k,const xr_vector<Part>& parts,CGID group,const gfy_live::GoreControls& controls){
    using namespace detail;purge();
    Arm armed;{auto a=arms.find(source.ID());if(a!=arms.end()){armed=a->second;arms.erase(a);}}
    if(armed.source_identity!=&source)armed=Arm{};
    const auto frozen=armed.bone!=BI_NONE?armed.controls:controls;if(!enabled(frozen))return;
    const u16 pelvis=k.LL_BoneID("bip01_pelvis"),spine=k.LL_BoneID("bip01_spine"),spine1=k.LL_BoneID("bip01_spine1"),spine2=k.LL_BoneID("bip01_spine2");
    if(pelvis==BI_NONE||spine1==BI_NONE||spine2==BI_NONE)return;
    const Part* L=nullptr;const Part* U=nullptr; // L: the pelvis without the chest; U: the chest (both chest elements)
    for(const auto& p:parts){if(!p.obj||!p.obj->PPhysicsShell())continue;
        if(!L&&p.holds(pelvis)&&!p.holds(spine2))L=&p;
        if(!U&&p.holds(spine2)&&p.holds(spine1))U=&p;}
    if(!L||!U||L==U)return;
    const bool blast=painter_explosions::blast_on(source.ID())!=nullptr;
    const float chance=_min(1.f,(blast?settings().chance_blast:settings().chance)*float(frozen.intestine_frequency_mult)),roll=::Random.randF(0.f,1.f);
    if(roll>=chance){Msg("[painter_gut_spill] id=%u phase=no_spill chance=%.2f roll=%.2f",source.ID(),chance,roll);return;}
    auto host=[&](const Part& p,std::initializer_list<u16> order)->std::pair<CPhysicsElement*,u16>{
        for(u16 b:order)if(p.holds(b))if(auto* e=p.obj->PPhysicsShell()->get_Element(b))return {e,b};
        // Never attach to an unrelated element when the expected torso anchor
        // is absent from the physical part.
        return {nullptr,BI_NONE};};
    const auto hl=host(*L,{spine,pelvis}),hu=host(*U,{spine,spine1,spine2});
    if(!hl.first||!hu.first)return;
    Fmatrix lx,ux;hl.first->GetGlobalTransformDynamic(&lx);hu.first->GetGlobalTransformDynamic(&ux);
    Fvector axis;axis.sub(ux.c,lx.c);Fvector mid;mid.add(lx.c,ux.c).mul(.5f);Fvector P=mid;bool at_tear=false;
    // The cut: the torso tear's station centre and axis when one armed this split, else between the two parts.
    if(armed.bone!=BI_NONE&&armed.bone<k.LL_BoneCount()){Fvector model,dir;
        {xrCriticalSectionGuard guard(&k.UCalc_Mutex);k.CalculateBones(TRUE);const Fmatrix& b=k.LL_GetBoneInstance(armed.bone).mRenderTransform;
         b.transform_tiny(model,armed.point);b.transform_dir(dir,armed.axis);}
        Fvector p,d;source.XFORM().transform_tiny(p,model);source.XFORM().transform_dir(d,dir);
        if(p.distance_to(mid)<.35f){P=p;at_tear=true;if(d.square_magnitude()>1e-8f){if(d.dotproduct(axis)<0.f)d.invert();axis=d;}}}
    if(axis.square_magnitude()<1e-8f)return;axis.normalize();
    Fvector e1,e2;{Fvector a;a.set(0.f,1.f,0.f);if(_abs(axis.y)>.9f)a.set(1.f,0.f,0.f);e1.crossproduct(a,axis).normalize();e2.crossproduct(axis,e1).normalize();}
    u32 ropes=1+u32(::Random.randF(0.f,1.f)<.6f)+u32(blast&&::Random.randF(0.f,1.f)<.5f);ropes=_min(ropes,settings().max_ropes);
    ropes=capacity(amount(ropes,frozen));
    u32 sent=0;char kinds[max_event_ropes+1]={};
    for(u32 i=0;i<ropes;++i){
        const bool lower=i%3!=1; // preserve the old first3, then distribute added ropes across both cut faces
        const Part& side=lower?*L:*U;const auto h=lower?hl:hu;
        const u32 v=i==0?u32(::Random.randI(2)):(i==1?(::Random.randF(0.f,1.f)<.6f?2u:1u):(::Random.randF(0.f,1.f)<.5f?3u:u32(::Random.randI(2))));
        Fvector out=axis;if(!lower)out.invert();
        const float a=::Random.randF(0.f,PI_MUL_2),r=(variant(v).loop?.0f:.015f)+.03f*::Random.randF(0.f,1.f);
        Fvector root;root.set(P).mad(out,-.01f).mad(e1,_cos(a)*r).mad(e2,_sin(a)*r);
        if(send(source,*side.obj,h.first,h.second,rope_frame(root,out,::Random.randF(0.f,PI_MUL_2)),v,group,frozen,break_force(frozen))){kinds[sent]=char('1'+v);++sent;}}
    Msg("[painter_gut_spill] id=%u phase=spill ropes=%u kinds=%s blast=%u at_tear=%u lower=%u upper=%u live=%u",
        source.ID(),sent,kinds,unsigned(blast),unsigned(at_tear),L->obj->ID(),U->obj->ID(),unsigned(live.size()));
}
inline void split_committed(CAI_Stalker& source,IKinematics& k,const xr_vector<Part>& parts,CGID group){split_committed(source,k,parts,group,gfy_live::GoreControls{});}
// A big abdominal exit or burst entry that destroyed intestine (PainterCommittedEffects_impl.h): its mouth in the hit
// bone's frame; `chance` from its size. One wound spill per body.
void wound(u16 source,u16 bone,const Fvector& point,const Fvector& normal,float chance,const gfy_live::GoreControls& controls,float opening_radius){
    using namespace detail;auto* object=smart_cast<CGameObject*>(Level().Objects.net_Find(source));if(!object)return;
    auto it=wounds.find(source);if(it!=wounds.end()){if(it->second.source_identity==object)return;wounds.erase(it);}
    if(!enabled(controls)||bone==BI_NONE||!finite(point)||!finite(normal)||normal.square_magnitude()<1e-8f||
        !std::isfinite(chance)||chance<0||!std::isfinite(opening_radius)||opening_radius<=0)return;
    const float c=_min(1.f,chance*settings().wound_chance*float(controls.intestine_frequency_mult)),roll=::Random.randF(0.f,1.f);
    if(roll>=c){Msg("[painter_gut_spill] id=%u phase=wound_no_spill chance=%.2f roll=%.2f",source,c,roll);return;}
    const u32 count=amount(1,controls);if(!count)return;
    Wound w;w.bone=bone;w.point=point;w.normal=normal;w.normal.normalize();w.time=Device.dwTimeGlobal;
    w.controls=controls;w.count=count;w.mouth_radius=_max(.005f,_min(.16f,opening_radius));w.break_force=break_force(controls);w.source_identity=object;wounds[source]=w;
    Msg("[painter_gut_spill] id=%u phase=wound_marked bone=%u chance=%.2f",source,unsigned(bone),c);}
void wound(u16 source,u16 bone,const Fvector& point,const Fvector& normal,float chance){wound(source,bone,point,normal,chance,gfy_live::GoreControls{},.03f);}
bool wound_pending(u16 source){auto it=detail::wounds.find(source);return it!=detail::wounds.end()&&!it->second.used;}
// Called for the body every frame while a wound spill waits: once it is a ragdoll, the loop comes out of the mouth.
void poll_wound(CAI_Stalker& source,CPhysicsShell* shell){
    using namespace detail;auto it=wounds.find(source.ID());if(it==wounds.end()||it->second.used)return;
    auto& w=it->second;
    if(w.source_identity!=&source){wounds.erase(it);return;}
    if(Device.dwTimeGlobal-w.time>60000){Msg("[painter_gut_spill] id=%u phase=wound_expired",source.ID());wounds.erase(it);return;}
    if(source.g_Alive()||!shell||!shell->isActive()||source.getDestroy())return;
    auto* k=smart_cast<IKinematics*>(source.Visual());if(!k||w.bone>=k->LL_BoneCount()){wounds.erase(it);return;}
    u16 b=w.bone;for(unsigned steps=0;b!=BI_NONE&&b<k->LL_BoneCount()&&!shell->get_Element(b)&&steps<64;++steps)b=k->LL_GetData(b).GetParentID();
    CPhysicsElement* he=b!=BI_NONE&&b<k->LL_BoneCount()?shell->get_Element(b):nullptr;
    auto* holder=smart_cast<CPhysicsShellHolder*>(&source);
    if(!he||!holder){wounds.erase(it);return;}
    purge();
    Fmatrix bone_world;{xrCriticalSectionGuard guard(&k->UCalc_Mutex);k->CalculateBones(TRUE);bone_world.mul_43(source.XFORM(),k->LL_GetTransform(w.bone));}
    Fvector p,n;bone_world.transform_tiny(p,w.point);bone_world.transform_dir(n,w.normal);if(n.square_magnitude()<1e-8f){wounds.erase(it);return;}
    n.normalize();
    const u32 count=capacity(w.count);if(!count)return;
    // The loop lies on this body (no shared collision group); only its links in the wound mouth pass through it.
    // The supplied mouth normal points out of the actual surviving opening.
    // Start just outside it; never push the coil into the torso/back. Extra
    // loops stay inside that same opening, not arbitrary neighbouring bones.
    Fvector tangent,bitangent;Fvector a;a.set(0,1,0);if(_abs(n.y)>.9f)a.set(1,0,0);
    tangent.crossproduct(a,n).normalize();bitangent.crossproduct(n,tangent).normalize();
    u32 sent=0;
    for(u32 i=0;i<count;++i){Fvector root;root.set(p).mad(n,.003f);
        if(i){const float angle=PI_MUL_2*float(i-1)/float(_max(1u,count-1));const float radius=w.mouth_radius*.35f;
            root.mad(tangent,_cos(angle)*radius).mad(bitangent,_sin(angle)*radius);}
        const u32 v=u32(::Random.randI(2));
        if(send(source,*holder,he,b,rope_frame(root,n,::Random.randF(0.f,PI_MUL_2)),v,0,w.controls,w.break_force,true,w.mouth_radius+.025f))++sent;}
    w.used=sent!=0;
    Msg("[painter_gut_spill] id=%u phase=wound_spill sent=%u bone=%u host_bone=%u live=%u",source.ID(),sent,unsigned(w.bone),unsigned(b),unsigned(live.size()));
}
// CPhysicsSkeletonObject::net_Spawn, before the shell is built: 1 a rope of ours (placed on its part as it is now),
// -1 refused (unknown or invalid rope: the spawn fails), 0 not a rope.
inline int begin_spawn(CPhysicsSkeletonObject& rope,CSE_Abstract* se){
    using namespace detail;const char* name=se?se->name_replace():nullptr;
    if(!name||std::strncmp(name,"painter_gut_",12))return 0;
    rope.setVisible(FALSE);rope.setEnabled(FALSE);
    auto it=pending.find(name);
    if(it==pending.end()){Msg("[painter_gut_spill] rope=%s phase=spawn_refused reason=unknown",name);return -1;}
    const Pending p=it->second;pending.erase(it);
    if(Device.dwTimeGlobal-p.at>10000)return -1;
    auto* k=smart_cast<IKinematics*>(rope.Visual());
    auto* host=smart_cast<CPhysicsShellHolder*>(Level().Objects.net_Find(p.host));
    CPhysicsElement* he=host&&!host->getDestroy()&&host->PPhysicsShell()?host->PPhysicsShell()->get_Element(p.host_bone):nullptr;
    const bool exact_host=host==p.host_identity&&host&&host->PPhysicsShell()==p.host_shell&&p.host_bone!=BI_NONE;
    if(!enabled(p.controls)||!exact_host||!k||k->LL_BoneCount()<4||k->LL_BoneCount()>32||k->LL_BoneID("gut_00")==BI_NONE||rope.cNameVisual()!=p.visual||!he){
        Msg("[painter_gut_spill] rope=%s phase=spawn_refused reason=%s",name,!he?"host_gone":"visual");return -1;}
    Fmatrix hx;he->GetGlobalTransformDynamic(&hx);rope.XFORM().mul_43(hx,p.local);
    try{spawning[rope.ID()]=p;}catch(...){return -1;}return 1;
}
// After the shell is built (from the rope's own bone shapes and joints): attach it. false: the spawn fails.
inline bool end_spawn(CPhysicsSkeletonObject& rope){
    using namespace detail;auto it=spawning.find(rope.ID());if(it==spawning.end())return false;
    const Pending p=it->second;spawning.erase(it);
    if(Device.dwTimeGlobal-p.at>10000)return false;
    CPhysicsShell* shell=rope.PPhysicsShell();
    auto* k=smart_cast<IKinematics*>(rope.Visual());
    auto* host=smart_cast<CPhysicsShellHolder*>(Level().Objects.net_Find(p.host));
    CPhysicsShell* hs=host&&!host->getDestroy()?host->PPhysicsShell():nullptr;
    CPhysicsElement* he=hs?hs->get_Element(p.host_bone):nullptr;
    const bool exact_host=host==p.host_identity&&hs==p.host_shell&&p.host_bone!=BI_NONE;
    if(!enabled(p.controls)||!exact_host||!shell||!shell->isActive()||!k||shell->get_ElementsNumber()!=k->LL_BoneCount()||!he){
        Msg("[painter_gut_spill] rope=%u phase=attach_refused shell=%u elements=%u host=%u",rope.ID(),unsigned(shell!=nullptr),
            shell?unsigned(shell->get_ElementsNumber()):0u,unsigned(he!=nullptr));return false;}
    const Variant& v=variant(p.variant);
    const u16 root=k->LL_BoneID("gut_00"),last=u16(k->LL_BoneCount()-1);
    if(root==BI_NONE||!shell->get_Element(root)||(v.loop&&(last==root||!shell->get_Element(last))))return false;
    shell->SetAirResistance(default_k_l,air_w(v));shell->set_DynamicLimits(default_l_limit,rope_w_limit);
    shell->SetIgnoreSmall();
    if(!p.wound){shell->SetSmall();shell->SetIgnoreRagDoll();shell->RegisterToCLGroup(p.group);}
    Fvector w;hs->get_AngularVel(w);
    for(u16 i=0;i<shell->get_ElementsNumber();++i){CPhysicsElement* e=shell->get_ElementByStoreOrder(i);Fmatrix m;e->GetGlobalTransformDynamic(&m);
        Fvector vel;he->GetPointVel(vel,m.c);e->set_LinearVel(vel);e->set_AngularVel(w);}
    shell->Enable();
    Live l;l.id=rope.ID();l.host=p.host;l.host_bone=p.host_bone;l.host_shell=hs;l.mouth_radius=p.mouth_radius;l.born=Device.dwTimeGlobal;
    l.endpoint[0]=root;l.endpoint[1]=v.loop?last:BI_NONE;
    Fmatrix host_world,host_inverse;he->GetGlobalTransformDynamic(&host_world);host_inverse.invert(host_world);
    const Fvector start=rope.XFORM().c;host_inverse.transform_tiny(l.local_anchor[0],start);
    l.tether[0]=phtether_create_exact(hs,p.host_bone,shell,root,start,p.break_force,u32(24u*3600u*1000u));
    if(!l.tether[0]||!l.tether[0]->Alive()){
        release(l);Msg("[painter_gut_spill] rope=%u phase=attach_refused reason=root_tether",rope.ID());return false;}
    if(v.loop){ // a loop: its last link's head lies at the cut next to the root (make_ropes2.py) - tethered there too
        Fvector end;{xrCriticalSectionGuard guard(&k->UCalc_Mutex);k->CalculateBones_Invalidate();k->CalculateBones(TRUE);rope.XFORM().transform_tiny(end,k->LL_GetTransform(last).c);}
        const float mouth_span=2.f*p.mouth_radius+2.f*v.radius;
        if(!finite(end)||end.distance_to_sqr(start)>mouth_span*mouth_span){
            release(l);Msg("[painter_gut_spill] rope=%u phase=attach_refused reason=loop_endpoint_outside_mouth",rope.ID());return false;}
        host_inverse.transform_tiny(l.local_anchor[1],end);
        l.tether[1]=phtether_create_exact(hs,p.host_bone,shell,last,end,p.break_force,u32(24u*3600u*1000u));}
    // A loop requires BOTH anchors. A failed endpoint never turns an admitted
    // spill into untethered debris; release the first joint on partial failure.
    if(!l.tether[0]||!l.tether[0]->Alive()||(v.loop&&(!l.tether[1]||!l.tether[1]->Alive()))){
        release(l);rope.setVisible(FALSE);rope.setEnabled(FALSE);
        Msg("[painter_gut_spill] rope=%u phase=attach_refused reason=required_tether",rope.ID());return false;}
    try{live.push_back(l);}catch(...){release(l);return false;}
    if(p.wound){
        // Install collision exemptions only after every required anchor exists;
        // failed spawns leave no callbacks or live reservation behind.
        shell->get_Element(root)->add_ObjectContactCallback(mouth_contact);
        if(v.loop)shell->get_Element(last)->add_ObjectContactCallback(mouth_contact);}
    shell->EnableCollision();
    rope.PainterRopeThermal(*host); // thermal3: the guts show interior heat on thermal and cool from the spill (cold before)
    rope.setVisible(TRUE);rope.setEnabled(TRUE);
    Msg("[painter_gut_spill] rope=%u phase=attached kind=%u host=%u bone=%u tethers=%u/%u on_body=%u delay_ms=%u",rope.ID(),p.variant+1,p.host,unsigned(p.host_bone),
        unsigned(l.tether[0]!=nullptr)+unsigned(l.tether[1]!=nullptr),v.loop?2u:1u,unsigned(p.wound),Device.dwTimeGlobal-p.at);
    return true;
}
inline void gone(CPhysicsSkeletonObject& rope){
    using namespace detail;spawning.erase(rope.ID());
    for(auto it=live.begin();it!=live.end();++it)if(it->id==rope.ID()){release(*it);live.erase(it);break;}
}
}
