#pragma once
// rpgdirect: the engine side of RocketStrike.h (included by Explosive.cpp only): the bone-shape ray, the explosion's place, the log
#include "RocketStrike.h"

namespace painter_rocket_strike {
static V3 v3(const Fvector& v){return V3{double(v.x),double(v.y),double(v.z)};}
static Fvector fv(V3 v){Fvector r;r.set(float(v.x),float(v.y),float(v.z));return r;}
struct RayFilter {u16 rocket,owner;};
// the bullet's test against bone shapes, stalkers only (living or dead), never the rocket, its owner or anything carried
static BOOL stalkerBody(const collide::ray_defs&,CObject* o,LPVOID data){const RayFilter& f=*static_cast<const RayFilter*>(data);
    if(!o||o->ID()==f.rocket||o->ID()==f.owner||o->H_Parent()||o->getDestroy())return FALSE;
    return smart_cast<CAI_Stalker*>(o)?TRUE:FALSE;}
static Ray cast(const Stretch& s,u16 rocket,u16 owner,CObject* self){Ray r;if(!s.valid||!g_pGameLevel)return r;
    RayFilter filter{rocket,owner};collide::rq_results hits;
    collide::ray_defs rd(fv(s.a),fv(s.dir),float(s.length),CDB::OPT_ONLYNEAREST,collide::rqtObject);
    g_pGameLevel->ObjectSpace.RayQuery(hits,rd,NULL,&filter,stalkerBody,self);
    if(hits.r_count()){const collide::rq_result& q=*hits.r_begin();
        if(q.O){r.hit=true;r.id=q.O->ID();r.bone=q.element>=0&&q.element<0xFFFF?u16(q.element):none;r.range=double(q.range);}}
    return r;}
static u16 nearestBone(CObject& o,const Fvector& at){IKinematics* k=o.Visual()?smart_cast<IKinematics*>(o.Visual()):nullptr;if(!k)return none;
    Fmatrix inv;inv.invert(o.XFORM());Fvector local;inv.transform_tiny(local,at);u16 best=none;float bestD=flt_max;
    for(u16 b=0;b<k->LL_BoneCount();++b){const float d=k->LL_GetTransform(b).c.distance_to_sqr(local);if(d<bestD){bestD=d;best=b;}}
    return best;}

bool frame(CGameObject& rocket,CGameObject* owner,const Fvector& launch,Fvector& at,Fvector& normal){
    std::lock_guard<std::mutex> lock(guard());Flight& f=flights()[rocket.ID()];begin(f,v3(launch),owner?owner->ID():none);
    if(!f.struck){const Stretch s=stretch(f,v3(at),0.);if(!fly(f,s,cast(s,rocket.ID(),f.owner,&rocket)))return false;}
    at=fv(f.at);normal=fv(mul(f.dir,-1.));if(normal.square_magnitude()<1e-8f)normal.set(0.f,1.f,0.f);return true;}

void ended(CGameObject& rocket,CGameObject* owner,const Fvector& launch,const Fvector& pos){
    std::lock_guard<std::mutex> lock(guard());end(flights()[rocket.ID()],v3(launch),owner?owner->ID():none,v3(pos));}

void place(CGameObject& explosive,Fvector& pos,bool first){
    const u16 id=explosive.ID();
    std::lock_guard<std::mutex> lock(guard());const auto it=flights().find(id);if(it==flights().end())return;Flight& f=it->second;
    if(!f.started)return; // a launcher grenade (only its touch is kept): as before
    if(first&&!f.struck&&!f.resolved&&(f.ended||f.touched)){
        CObject* touched=f.touched&&f.touchedId!=none&&g_pGameLevel?Level().Objects.net_Find(f.touchedId):nullptr;
        const bool touchedStalker=touched&&!touched->getDestroy()&&smart_cast<CAI_Stalker*>(touched)&&f.touchedId!=f.owner;
        const Stretch s=stretch(f,lastPoint(f),touchedStalker?touchReach:0.);
        if(settle(f,s,cast(s,id,f.owner,&explosive),touchedStalker)&&f.bone==none)
            if(CObject* body=Level().Objects.net_Find(f.target))f.bone=nearestBone(*body,fv(f.at));}
    if(f.struck)pos=fv(f.at);
    if(!first||f.logged)return;
    f.logged=true;
    if(!f.struck){Msg("[painter_voxel_live] phase=rocket_no_strike explosive=%u section=%s flown_m=%.2f touched=%u",
        unsigned(id),explosive.cNameSect().c_str(),flownTo(f),unsigned(f.touched?f.touchedId:none));return;}
    CObject* body=g_pGameLevel?Level().Objects.net_Find(f.target):nullptr;CAI_Stalker* stalker=body?smart_cast<CAI_Stalker*>(body):nullptr;
    IKinematics* k=body&&body->Visual()?smart_cast<IKinematics*>(body->Visual()):nullptr;
    LPCSTR bone=k&&f.bone!=none&&f.bone<k->LL_BoneCount()?k->LL_BoneName_dbg(f.bone):nullptr;
    Msg("[painter_voxel_live] id=%u phase=rocket_direct_strike explosive=%u section=%s bone=%s how=%s flown_m=%.2f alive=%u at=%.3f,%.3f,%.3f",
        unsigned(f.target),unsigned(id),explosive.cNameSect().c_str(),bone?bone:"none",f.how,f.flownAt,unsigned(stalker&&stalker->g_Alive()),
        f.at.x,f.at.y,f.at.z);}

bool struck(CGameObject& explosive,CObject& body){
    std::lock_guard<std::mutex> lock(guard());const auto it=flights().find(explosive.ID());
    return it!=flights().end()&&it->second.struck&&it->second.target==body.ID();}
}
