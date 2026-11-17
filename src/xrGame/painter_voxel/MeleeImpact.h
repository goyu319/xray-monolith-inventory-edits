#pragma once
#include "LiveDamageWorker.h"
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
namespace gfy_live { namespace melee {
enum Kind:std::uint8_t {none=0,slash=1,stab=2,chop=3,blunt=4};
inline bool blade(std::uint8_t k){return k==slash||k==stab||k==chop;}
inline const char* name(std::uint8_t k){return k==slash?"slash":k==stab?"stab":k==chop?"chop":k==blunt?"blunt":"none";}
struct Spec {
    Kind kind=none;
    Point point,direction; // pose (model) space: the struck surface point and the strike's travel into the body
    Point swing;           // pose space: the blade's motion across the body (slash / chop); zero = across the travel, level
    Point edge;            // stab: the blade's width axis (pose space); zero = upright across the travel
    double length=.08;     // slash / chop: the cut's length along the swing (m); stab: the blade's width (m)
    double depth=.03;      // how deep the edge goes in (m)
    double thickness=.004; // the kerf (m)
    double reach=0;        // blade sever: the thickest part (diameter, m) one blow cuts through
    bool sever=false;      // blade: this blow may cut the part through (Lua's decision)
    double dent_radius=0,dent_depth=0,chunk_radius=0,chunk_depth=0; // blunt (m)
    // blunt, a KILLING blow only (Lua: the sledgehammer): crush = the limb / neck it struck is crushed off - the legacy kill sever's
    // torn tear (killSeverRound: kill_sever 1), never a clean cut; head = the skull bursts (1 cap, 2 pieces) through the head event
    // (headfix's burst, HeadBurst.h) sized by head_strength (0.2-1: a rifle round's burst ~0.5, a slug's ~0.82)
    bool crush=false;std::uint8_t head=0;double head_strength=0;
    std::uint64_t seed=1;
};
inline double len3(const Point&p){return std::sqrt(double(p.x)*p.x+double(p.y)*p.y+double(p.z)*p.z);}
inline Point unit3(const Point&p,Point fallback){const double l=len3(p);if(!(l>1e-9)||!std::isfinite(l))return fallback;return {p.x/l,p.y/l,p.z/l};}
inline double dot3(const Point&a,const Point&b){return double(a.x)*b.x+double(a.y)*b.y+double(a.z)*b.z;}
inline Point cross3(const Point&a,const Point&b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline Point sub3(const Point&a,const Point&b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
inline Point mad3(const Point&a,const Point&b,double s){return {a.x+b.x*s,a.y+b.y*s,a.z+b.z*s};}
// a direction squared up against d (unit); fallback: across d, level (model y up)
inline Point across(const Point&want,const Point&d){Point a=mad3(want,d,-dot3(want,d));
    if(len3(a)<.2*std::max(1e-9,len3(want))||len3(want)<1e-9){a=cross3(d,Point{0,1,0});if(len3(a)<.2)a=cross3(d,Point{1,0,0});}
    return unit3(a,Point{1,0,0});}
// The events of one melee hit. `base` carries what every event shares (sequence, gore, palette, bone, power, living / killing,
// initiator, ammo_section = the weapon's section, ammo_seed). Returns false (no events) for an invalid spec.
inline bool fill(const Spec&s,const Impact&base,std::vector<Impact>&out){
    if(s.kind==none)return false;
    const Point d=unit3(s.direction,Point{0,0,0});if(len3(d)<.5)return false;
    Impact e=base;e.melee=std::uint8_t(s.kind);e.ammo_seed=s.seed?s.seed:1;e.melee_blow=e.ammo_seed; // every event of the blow carries it
    e.round_class=0;e.kill_sever=0; // never the bullet kill sever (killSeverRound) or a round's head burst class
    e.cavitation=0;e.burst=0;e.vent=0;e.entry_burst=0;e.exit_diameter_mult=1;e.blast=false;e.blade=0;e.tear_request=false;
    e.source_search_distance=std::max(e.source_search_distance,.2);
    // the event's ray starts 6 cm out along the travel, so the source surface (skin or gear) is found ahead of it
    const Point start=mad3(s.point,d,-.06);e.accepted_pose_point=start;e.accepted_pose_direction=d;
    if(blade(s.kind)){
        const double len=std::clamp(s.length,.01,.6),dep=std::clamp(s.depth,.004,.45),thick=std::clamp(s.thickness,.002,.02);
        Point side=s.kind==stab?across(len3(s.edge)>1e-9?s.edge:Point{0,1,0},d):across(s.swing,d);
        Point n=unit3(cross3(side,d),Point{0,0,1});
        e.slab=true;e.ragged_slab=false;e.slab_normal=n;e.slab_side=side;
        e.slab_half_width=std::clamp(.5*len,.005,.3);e.radius=std::clamp(.5*thick,.001,.01);e.penetration_m=dep; // from the first tissue
        e.melee_sever=s.sever;e.melee_reach=std::max(0.,s.reach); // round 3: any blade blow may finish a cut that went through
        // the wound gapes: skin and the tissue under it pull back from a cut, so its mouth is wider than the edge's kerf. A slash's
        // mouth is a row of short shallow slabs along the cut (one per ~2.2 cm, 3-7), each found on the skin under its own stretch of
        // the cut (it follows the limb's curve), turned up to 7 degrees out of the blade's plane, set off up to a third of its width,
        // 0.75-1.25 times as wide and 0.7-1.15 times as deep, the two end ones tapered: the mouth's outline is a ragged polyline, not
        // one straight 2 mm staircase. A stab's mouth is one short slab 1.8 kerfs wide. Gape slabs never sever, carry no dent and
        // carry no blood of their own (melee_part: the blow's blood rides on its main event, report()).
        auto u=[&](unsigned k){std::uint64_t x=(e.ammo_seed^0x6A9E5ull)*0x9E3779B97F4A7C15ull+k*0xC2B2AE3D27D4EB4Full;x^=x>>29;x*=0xBF58476D1CE4E5B9ull;x^=x>>32;return double(x>>11)*(1./9007199254740992.);};
        const double gT=std::clamp(.5*thick*(s.kind==stab?1.8:2.4),.0025,.016),gD=std::clamp((s.kind==stab?.18:.33)*dep,.005,.035);
        const unsigned nseg=s.kind==stab?1u:std::clamp(unsigned(std::lround(len/.022)),3u,7u);const double segLen=len/double(nseg);
        const Point start0=e.accepted_pose_point;
        out.push_back(e);
        for(unsigned k=0;k<nseg;++k){Impact g=e;g.sequence=e.sequence+1+k;g.ammo_seed=e.ammo_seed^(0x6A9Eull+0x9E37ull*k);
            g.melee_sever=false;g.melee_reach=0;g.melee_part=1;
            if(s.kind==stab){g.radius=gT;g.penetration_m=gD;g.slab_half_width=std::clamp(.5*len,.005,.3);out.push_back(std::move(g));continue;}
            const double t=-.5*len+(double(k)+.5)*segLen,ang=(2*u(10+k)-1)*.122,taper=(k==0||k+1==nseg)?.55:1.;
            const Point side2=unit3(mad3(Point{side.x*std::cos(ang),side.y*std::cos(ang),side.z*std::cos(ang)},n,std::sin(ang)),side);
            const Point n2=unit3(cross3(side2,d),n);const double off=(2*u(20+k)-1)*.33*gT;
            g.accepted_pose_point=mad3(mad3(start0,side,t),n,off);g.slab_normal=n2;g.slab_side=side2;
            g.slab_half_width=std::clamp(.5*segLen*1.3,.005,.3);g.radius=std::clamp(gT*taper*(.75+.5*u(30+k)),.0015,.016);
            g.penetration_m=std::clamp(gD*taper*(.7+.45*u(40+k)),.004,.035);out.push_back(std::move(g));}
        return true;}
    if(s.kind==blunt){
        e.dent_radius=std::clamp(s.dent_radius,0.,.15);e.dent_depth=std::clamp(s.dent_depth,0.,.05);
        if(!(s.chunk_radius>.004&&s.chunk_depth>.003)){ // a dent only: nothing is carved (LiveDamageWorker.cpp records the dent and stops)
            if(!(e.dent_radius>.005&&e.dent_depth>.001))return false;
            e.radius=.01;e.penetration_m=0;out.push_back(std::move(e));return true;}
        // the chunk: a shallow crater below the dent's floor, knocked out as a ragged cluster - a main bore and 3-4 smaller ones around
        // it at seeded offsets, radii and depths (each with the rough edge every seeded bore has and a little wall swell); no entry
        // burst (that bore runs 2.5 radii deep). The first event carries the dent.
        const double R=std::clamp(s.chunk_radius,.006,.06),D=std::clamp(s.chunk_depth+e.dent_depth,.004,.12);
        auto u=[&](unsigned k){std::uint64_t x=(e.ammo_seed^0xC4A2Bull)*0x9E3779B97F4A7C15ull+k*0xC2B2AE3D27D4EB4Full;x^=x>>29;x*=0xBF58476D1CE4E5B9ull;x^=x>>32;return double(x>>11)*(1./9007199254740992.);};
        const Point a1=across(Point{0,1,0},d),a2=unit3(cross3(d,a1),Point{0,0,1});
        const unsigned n=3u+(u(0)<.5?1u:0u);const double phase=6.283185307179586*u(1);
        e.radius=R*(.68+.12*u(2));e.penetration_m=D;e.cavitation=.3;e.entry_burst=0;
        // a killing sledge blow: the struck limb / neck crushed off (the worker's legacy kill sever: torn at the station nearest the
        // blow, the approved tear look); on the head the skull bursts (the head event, kind and size from Lua). Both also hold the
        // death shell for the tear (severHoldRound: kill_sever, or round_class >= 1 on the head).
        if(e.killing_hit&&s.head&&e.cranial_bone!=65535&&e.hit_bone==e.cranial_bone){e.melee_head=std::uint8_t(std::min<unsigned>(s.head,2u));
            e.melee_head_strength=std::clamp(s.head_strength,.2,1.);e.round_class=e.melee_head_strength>=.66?2:1;}
        else if(e.killing_hit&&s.crush)e.kill_sever=1;
        out.push_back(e);
        for(unsigned k=0;k<n;++k){Impact c=e;c.sequence=e.sequence+1+k;c.ammo_seed=e.ammo_seed+0x51u*(k+1);c.dent_radius=0;c.dent_depth=0;c.melee_part=1;
            c.kill_sever=0;c.melee_head=0;c.melee_head_strength=0;c.round_class=0; // the side bores never sever or burst
            const double ang=phase+6.283185307179586*(double(k)+.35*u(10+k))/double(n),off=R*(.38+.32*u(20+k));
            c.accepted_pose_point=mad3(mad3(e.accepted_pose_point,a1,off*std::cos(ang)),a2,off*std::sin(ang));
            c.radius=R*(.34+.26*u(30+k));c.penetration_m=D*(.45+.5*u(40+k));c.cavitation=.25;out.push_back(std::move(c));}
        return true;}
    return false;}
// The size Lua's blood reads off the receipt (radius, exit radius): a slash's length, not its kerf - the blood of a bullet wound of the
// same size (a 10 cm slash bleeds like a 4.5 cm wound); a stab like a hole of its blade's width; a blunt crater as carved.
// A part of a blow (a gape slab, a crater's side bore: melee_part) reports a 0.6 mm wound: painter_melee.script drops such a
// receipt (one blow, one blood); without the script it paints nothing visible.
constexpr double part_radius=.0006;
inline void report(ImpactResult&r,const Impact&e){
    if(e.melee_part){r.radius=part_radius;r.exit_radius=part_radius;r.penetrated=false;return;}
    if(e.melee==blunt){r.penetrated=false;return;} // a club's crater never comes out the far side (an exit test of a shallow bore
                                                   // that ran into a cavity or the clothing gap would paint a far-side wound)
    if(!blade(e.melee))return;
    const double eq=e.melee==stab?std::max(.006,.6*e.slab_half_width):std::clamp(.45*e.slab_half_width,.008,.05);
    r.radius=std::max(r.radius,eq);r.exit_radius=std::max(r.exit_radius,r.radius);}
// ---- the body's dents (rest space), merged as blows land: a blow within the reach of an earlier dent deepens and widens it, else
// a new dent; at most max_dents per body (the oldest dent leaves first; the renderer reads 2 texels per dent)
constexpr std::size_t max_dents=48;
inline void addDent(std::vector<gfy_packed::Dent>&list,const gfy_packed::Dent&d,double maxDepth=.05){ // modes: maxDepth < .05 (Bullet holes only): never past max(maxDepth, the dent's depth)
    for(auto&o:list){const double dx=double(o.centre[0])-d.centre[0],dy=double(o.centre[1])-d.centre[1],dz=double(o.centre[2])-d.centre[2];
        const double dist=std::sqrt(dx*dx+dy*dy+dz*dz);if(dist>.45*(double(o.radius)+d.radius))continue;
        const double wo=double(o.depth),wn=double(d.depth),w=std::max(1e-9,wo+wn);
        for(int a=0;a<3;++a){o.centre[a]=float((o.centre[a]*wo+d.centre[a]*wn)/w);o.push[a]=float((o.push[a]*wo+d.push[a]*wn)/w);}
        const double pl=std::sqrt(double(o.push[0])*o.push[0]+double(o.push[1])*o.push[1]+double(o.push[2])*o.push[2]);
        if(pl>1e-9)for(int a=0;a<3;++a)o.push[a]=float(o.push[a]/pl);
        o.radius=float(std::min(.15,std::max(double(o.radius),double(d.radius))*1.08));
        o.depth=float(std::min(maxDepth<.05?std::max(maxDepth,wo):.05,std::max(wo,wn)+.4*std::min(wo,wn)));return;}
    list.push_back(d);if(list.size()>max_dents)list.erase(list.begin());}
// ---- the body's blade slits (rest space boxes, gfy_packed::Slit): at most max_slits (the oldest leaves first; 3 texels each)
constexpr std::size_t max_slits=96;
inline void addSlit(std::vector<gfy_packed::Slit>&list,const gfy_packed::Slit&s){
    const float v[12]={s.centre[0],s.centre[1],s.centre[2],s.half_side,s.side[0],s.side[1],s.side[2],s.half_normal,s.normal[0],s.normal[1],s.normal[2],s.half_depth};
    for(float x:v)if(!std::isfinite(x))return;
    if(!(s.half_side>0)||!(s.half_normal>0)||!(s.half_depth>0))return;
    list.push_back(s);if(list.size()>max_slits)list.erase(list.begin());}
}} // namespace gfy_live::melee
