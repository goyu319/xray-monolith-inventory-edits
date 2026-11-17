#pragma once
#include <cmath>
#include <cstdint>
#include <mutex>
#include <unordered_map>

namespace painter_rocket_strike {
constexpr std::uint16_t none=0xFFFF;
struct V3 {double x,y,z;};
inline V3 add(V3 a,V3 b){return V3{a.x+b.x,a.y+b.y,a.z+b.z};}
inline V3 sub(V3 a,V3 b){return V3{a.x-b.x,a.y-b.y,a.z-b.z};}
inline V3 mul(V3 a,double s){return V3{a.x*s,a.y*s,a.z*s};}
inline double len(V3 a){return std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z);}
// past a touch on a stalker, how far the line looks for his bone shapes (the column's radius 0.2 m and a torso's depth)
constexpr double touchReach=.35;
struct Flight {
    bool started=false;V3 from{0,0,0};double flown=0;                      // the next stretch starts at `from`; metres tested
    std::uint16_t owner=none;                                               // the launcher's owner: never struck
    bool touched=false;std::uint16_t touchedId=none;V3 touchedAt{0,0,0};   // the physics contact: what (none = the level), where
    bool ended=false;V3 end{0,0,0};                                         // where the contact (or the fuse) put the rocket
    bool resolved=false;                                                    // the last stretch was tested
    bool struck=false;std::uint16_t target=none,bone=none;V3 at{0,0,0},dir{0,0,0};double flownAt=0;const char* how="";
    bool logged=false;};
inline std::mutex& guard(){static std::mutex m;return m;}
inline std::unordered_map<std::uint16_t,Flight>& flights(){static std::unordered_map<std::uint16_t,Flight> t;return t;}
inline void begin(Flight&f,V3 launch,std::uint16_t owner){if(!f.started){f.started=true;f.from=launch;f.flown=0;}f.owner=owner;}
// the physics contact (CCustomRocket::ObjectContactCallback, the physics thread): the first touch is kept
inline void touch(std::uint16_t rocket,std::uint16_t other,V3 at){std::lock_guard<std::mutex> lock(guard());Flight&f=flights()[rocket];
    if(f.touched)return;f.touched=true;f.touchedId=other;f.touchedAt=at;}
inline void forget(std::uint16_t rocket){std::lock_guard<std::mutex> lock(guard());flights().erase(rocket);}
// the rocket's contact or fuse (CExplosiveRocket::Contact): where its flight ended
inline void end(Flight&f,V3 launch,std::uint16_t owner,V3 pos){begin(f,launch,owner);if(f.struck||f.ended)return;f.ended=true;f.end=pos;}
// the last stretch's end: the touch point when there is one (the contact's own position is a physics step back), else the end
inline V3 lastPoint(const Flight&f){return f.touched?f.touchedAt:f.end;}
// one stretch of the flight: from the last tested point to `to`, `reach` metres further
struct Stretch {bool valid=false;V3 a{0,0,0},dir{0,0,0};double length=0,flown=0;};
inline Stretch stretch(const Flight&f,V3 to,double reach){Stretch s;const V3 d=sub(to,f.from);const double l=len(d);
    if(!f.started||!std::isfinite(l)||!(l>1e-4))return s;
    s.valid=true;s.a=f.from;s.dir=mul(d,1/l);s.length=l+(reach>0?reach:0);s.flown=l;return s;}
// what the line met first (the engine's bone-shape ray): a stalker's bone shape `range` metres along the stretch
struct Ray {bool hit=false;std::uint16_t id=none,bone=none;double range=0;};
inline bool counts(const Flight&f,const Stretch&s,const Ray&r){return s.valid&&r.hit&&r.id!=none&&r.id!=f.owner&&r.range>=0&&r.range<=s.length;}
inline void strike(Flight&f,const Stretch&s,std::uint16_t id,std::uint16_t bone,V3 at,double along,const char* how){
    f.struck=true;f.target=id;f.bone=bone;f.at=at;f.dir=s.dir;f.flownAt=f.flown+along;f.how=how;}
// a frame of flight: true = the rocket struck a stalker (it detonates at f.at); else the stretch is flown
inline bool fly(Flight&f,const Stretch&s,const Ray&r){
    if(f.struck)return true;if(!s.valid)return false;
    if(counts(f,s,r)){strike(f,s,r.id,r.bone,add(s.a,mul(s.dir,r.range)),r.range,"path");return true;}
    f.from=add(s.a,mul(s.dir,s.flown));f.flown+=s.flown;return false;}
// the flight's last stretch, tested once when the explosion is made: the first stalker on the line, else a touched stalker at the
// touch; else no strike (the explosion as before)
inline bool settle(Flight&f,const Stretch&s,const Ray&r,bool touchedStalker){
    if(f.struck)return true;if(f.resolved)return false;f.resolved=true;
    if(counts(f,s,r)){strike(f,s,r.id,r.bone,add(s.a,mul(s.dir,r.range)),r.range,"path");return true;}
    if(f.touched&&touchedStalker&&f.touchedId!=none&&f.touchedId!=f.owner){Stretch t=s;if(!t.valid){t.a=f.from;t.dir=V3{0,0,0};}
        strike(f,t,f.touchedId,none,f.touchedAt,len(sub(f.touchedAt,f.from)),"contact");return true;}
    return false;}
// metres flown when the flight ended without a strike (the log)
inline double flownTo(const Flight&f){return f.flown+((f.ended||f.touched)?len(sub(lastPoint(f),f.from)):0.);}
}
#ifndef PAINTER_ROCKET_STRIKE_PURE
class CGameObject;class CObject;
namespace painter_rocket_strike {
// the rocket's frame (CExplosiveRocket::UpdateCL; `at` = the rocket now): true = it struck a stalker, `at` / `normal` = where to go off
bool frame(CGameObject& rocket,CGameObject* owner,const Fvector& launch,Fvector& at,Fvector& normal);
// the rocket's contact or fuse (CExplosiveRocket::Contact; also on the physics thread): where its flight ended
void ended(CGameObject& rocket,CGameObject* owner,const Fvector& launch,const Fvector& pos);
// the explosion (CExplosive::Explode): a rocket that struck a stalker goes off at the strike (first: before anything uses the
// position - the last stretch is tested and the line logged; not first: after the explosion box moved it)
void place(CGameObject& explosive,Fvector& pos,bool first);
// the wave (CExplosive::ExplodeWaveProcessObject): the body this explosive's rocket struck
bool struck(CGameObject& explosive,CObject& body);
}
#endif
