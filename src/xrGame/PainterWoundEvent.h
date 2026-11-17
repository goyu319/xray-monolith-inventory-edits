// body_hit_outcomes_v1
#pragma once
// One accepted fire-wound hit -> the worker's events. Lua sizes it (CPainterVoxelWound, or CPainterVoxelFragment for a
// grenade fragment), the engine draws its forensic shape. Shared by live bodies (PainterVoxelLive_impl.h) and
// severed pieces and stumps (PainterPieceContinuation_impl.h), so a wound is the same whatever it hits.
#include "Hit.h"
#include "painter_voxel/LiveDamageWorker.h"
#include "painter_voxel/GoreSettings.h"
#include "painter_voxel/WoundShapePolicy.h"
#include "painter_voxel/PainterExplosions.h"
#include "ai_space.h"
#include "../xrServerEntities/script_engine.h"
#include "../Include/xrRender/Kinematics.h"
#include "../Include/xrRender/PainterLogVerbose.h"
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring>
#include "PainterWoundSkipReason.h"

namespace painter_wound_event {
enum class Status {silent,rejected,queued};
struct Result {
    Status status=Status::silent;const char* reason="";
    std::string ammo_section;unsigned pellets=0;double radius=0,penetration=0,exit_mult=1;
    const char* policy="lua";unsigned blade=0; // sub-cuts of a fragment blade
};
// Fmatrix keeps its translation in the bottom row; gfy_live::Matrix is column-vector (translation in row_major[3],
// [7], [11]), so this is the transpose - the same conversion as matrix() in PainterVoxelLive_impl.h.
inline gfy_live::Matrix matrix(const Fmatrix& m){gfy_live::Matrix out;
    out.row_major={m._11,m._21,m._31,m._41,m._12,m._22,m._32,m._42,m._13,m._23,m._33,m._43,m._14,m._24,m._34,m._44};return out;}
// Events numbered from sequence+1 are appended to `events`. `tag` names the owner kind in the forensics log line.
inline Result build(CObject& object,IKinematics* k,bool alive,const SHit& hit,u64 sequence,
    std::vector<gfy_live::Impact>& events,const char* tag)
{
    Result r;
    painter_explosions::Record* fragmentOf=hit.ammo.valid()?nullptr:painter_explosions::fragment(hit,object);
    if(!hit.ammo.valid()&&!fragmentOf){r.reason="no_ammo_or_recent_fragment";return r;}
    if(fragmentOf&&!painter_explosions::can_fragment(*fragmentOf,object)){painter_explosions::note_fragment_cap(*fragmentOf,object);r.reason="explosion_fragment_cap";return r;} // fragwound: over the cap, silent
    PainterHitAmmo fragmentAmmo;
    if(fragmentOf&&!fragmentAmmo.assign(fragmentOf->section.c_str(),1,std::isfinite(hit.armor_piercing)?_max(0.f,hit.armor_piercing):0.f,_max(0.f,hit.damage()))){r.reason="invalid_fragment_ammo";return r;}
    const PainterHitAmmo& ammo=fragmentOf?fragmentAmmo:hit.ammo;
    auto reject=[&](const char* why){r.status=Status::rejected;r.reason=why;return r;};
    ::luabind::functor<::luabind::object> policy;
    if(!ai().script_engine().functor(fragmentOf?"_G.CPainterVoxelFragment":"_G.CPainterVoxelWound",policy))
        return reject(fragmentOf?"missing_script_fragment_policy":"missing_script_wound_policy");
    ::luabind::object source=::luabind::newtable(ai().script_engine().lua());
    source["ammo_section"]=ammo.section;
    source["pellet_count"]=unsigned(ammo.pellet_count);
    if(fragmentOf)source["fragment"]=true;
    source["bullet_id"]=hit.BulletID;
    source["owner_id"]=object.ID();
    source["sequence"]=double(sequence+1);
    source["impact_power"]=ammo.impact_power;
    source["armor_piercing"]=ammo.armor_piercing;
    source["accepted_power"]=hit.painter_effective_power_valid?hit.painter_effective_power:hit.damage();
    if(!fragmentOf && (hit.painter_armor_penetration==0 || hit.painter_armor_penetration==1))
        source["armor_penetrated"]=hit.painter_armor_penetration==1; // exact receiver-local decision, if supplied
    source["hit_bone"]=unsigned(hit.bone());
    {LPCSTR bone_name=k->LL_BoneName_dbg(hit.bone());source["bone_name"]=bone_name?bone_name:"";}
    source["alive"]=alive;
    if(const CObject* who=hit.initiator())source["initiator"]=unsigned(who->ID()); // NPC-on-NPC switch (Lua)
    double distance=-1;
    {if(fragmentOf){Fvector model,world;k->LL_GetBoneInstance(hit.bone()).mTransform.transform_tiny(model,hit.bone_space_position());
            object.XFORM().transform_tiny(world,model);distance=world.distance_to(fragmentOf->pos);} // from the explosion
        else if(const CObject* who=hit.initiator()){Fvector model,world;k->LL_GetBoneInstance(hit.bone()).mTransform.transform_tiny(model,hit.bone_space_position());
            object.XFORM().transform_tiny(world,model);distance=world.distance_to(who->Position());}
        source["distance_m"]=distance;}
    const ::luabind::object output=policy(source);
    if(!output||output.type()!=LUA_TTABLE){
        // hitadmission: retain the policy's exact refusal instead of guessing from the body's model or health.
        // hitadmission_safe_v2: fixed buffer and raw Lua access; a diagnostic cannot use the userdata string converter.
        char why[161];read_skip_reason(source,why);
        Msg("[painter_wound_admission] id=%u sequence=%llu bullet=%u policy=%s alive=%u bone=%u ammo=%s pellets=%u accepted=%.9g impact=%.9g ap=%.9g receiver_armor=%d distance_m=%.6f",
            object.ID(),sequence+1,hit.BulletID,why,unsigned(alive),unsigned(hit.bone()),ammo.section,unsigned(ammo.pellet_count),
            hit.painter_effective_power_valid?hit.painter_effective_power:hit.damage(),ammo.impact_power,ammo.armor_piercing,hit.painter_armor_penetration,distance);
        return reject("script_wound_skipped");
    }
    auto number=[&](LPCSTR key,double& value){
        const ::luabind::object field=output[key];
        if(!field||field.type()!=LUA_TNUMBER)return false;
        value=::luabind::object_cast<double>(field);return std::isfinite(value);
    };
    double radius=0,penetration=0,exit_mult=1;
    // Geometry limits, not caliber rules. Reject bad Lua output before any
    // pose capture or worker enqueue; no health/damage values are changed.
    if(!number("radius_m",radius)||!number("penetration_m",penetration)||
        radius<0.0005||radius>0.08||penetration<=0||penetration>0.5)
        return reject("invalid_script_wound_dimensions");
    const ::luabind::object exit_field=output["exit_diameter_mult"];
    if(exit_field&&exit_field.type()!=LUA_TNIL&&
        (!number("exit_diameter_mult",exit_mult)||exit_mult<1||exit_mult>2))
        return reject("invalid_script_exit_dimension");
    // klass (2026-11-11): what the round is comes from Lua with the wound's size, never from the size of the hole (real bullet
    // diameters since 2026-11-11). Both fields are optional: round_class 0 small arms (pistol, buckshot pellet), 1 rifle, 2 heavy
    // round or slug; kill_sever true/false (or 1/0) = this shot, if it is the killing hit, tears the limb or neck it went through.
    // Absent (nil) keeps the old hole-size tests (-1 in the event); present but invalid rejects the hit like the fields above.
    int round_class=-1,kill_sever=-1;
    {const ::luabind::object class_field=output["round_class"],sever_field=output["kill_sever"];
        if(class_field&&class_field.type()!=LUA_TNIL){double value=-1;
            if(!number("round_class",value)||(value!=0&&value!=1&&value!=2))return reject("invalid_script_round_class");
            round_class=int(value);}
        if(sever_field&&sever_field.type()!=LUA_TNIL){
            if(sever_field.type()==LUA_TBOOLEAN)kill_sever=::luabind::object_cast<bool>(sever_field)?1:0;
            else{double value=-1;
                if(!number("kill_sever",value)||(value!=0&&value!=1))return reject("invalid_script_kill_sever");
                kill_sever=int(value);}}}
    const Fmatrix object_world=object.XFORM();Fmatrix world_to_object;world_to_object.invert(object_world);
    gfy_live::Impact event;event.sequence=sequence+1;
    event.gore=painter_deformation::snapshot();event.cranial_bone=k->LL_BoneID("bip01_head");
    Fvector point_model,direction_model;
    {xrCriticalSectionGuard pose(&k->UCalc_Mutex);k->CalculateBones_Invalidate();k->CalculateBones(TRUE);
        k->LL_GetBoneInstance(hit.bone()).mTransform.transform_tiny(point_model,hit.bone_space_position());
        world_to_object.transform_dir(direction_model,hit.direction());
        event.bone_model_render.reserve(k->LL_BoneCount());for(u16 b=0;b<k->LL_BoneCount();++b)event.bone_model_render.push_back(matrix(k->LL_GetBoneInstance(b).mRenderTransform));}
    event.accepted_pose_point={point_model.x,point_model.y,point_model.z};event.accepted_pose_direction={direction_model.x,direction_model.y,direction_model.z};
    // Freeze script output with this hit. No Lua or ammo policy runs on workers.
    event.radius=radius;event.penetration_m=penetration;event.exit_diameter_mult=exit_mult;
    event.round_class=round_class;event.kill_sever=kill_sever; // klass: as given by Lua (-1: not given)
    event.ammo_section=ammo.section;event.hit_bone=hit.bone();event.initiator=fragmentOf?fragmentOf->initiator:hit.whoID;
    {double cavitation=-1,burst=-1,vent=-1,entryBurst=-1;const ::luabind::object cf=output["cavitation"],bf=output["burst"],vf=output["vent"],ef=output["entry_burst"];
        if((cf&&cf.type()!=LUA_TNIL&&(!number("cavitation",cavitation)||cavitation<0||cavitation>2))||
           (bf&&bf.type()!=LUA_TNIL&&(!number("burst",burst)||burst<0||burst>1))||
           (vf&&vf.type()!=LUA_TNIL&&(!number("vent",vent)||vent<0||vent>1))||
           (ef&&ef.type()!=LUA_TNIL&&(!number("entry_burst",entryBurst)||entryBurst<0||entryBurst>1)))
            return reject("invalid_script_forensic_shape");
        const std::uint64_t fs=(std::uint64_t(hit.BulletID)<<32)^(std::uint64_t(event.sequence)*0x9E3779B97F4A7C15ull)^(std::uint64_t(hit.bone())<<48)^std::uint64_t(object.ID());
        event.ammo_seed=fs?fs:1; // the kernel seeds the edge shape, cracks, cavity placement, vent and sever draws from this
        double nominal=penetration;{const ::luabind::object nf=output["nominal_penetration_m"];if(nf&&nf.type()!=LUA_TNIL&&!number("nominal_penetration_m",nominal))nominal=penetration;}
        LPCSTR boneName=k->LL_BoneName_dbg(hit.bone());const bool headBone=boneName&&strstr(boneName,"head");
        gfy_live::WoundShapeInput shape;shape.seed=fs;shape.radius=radius;shape.penetration=nominal;shape.exit_mult=exit_mult;
        shape.head=headBone;shape.fragment=fragmentOf!=nullptr;
        shape.cavitation=cavitation;shape.burst=burst;shape.vent=vent;shape.entry_burst=entryBurst;
        shape.round_class=round_class; // klass: Lua's class decides the shape draws when given
        const auto selected=gfy_live::wound_shape(shape,event.gore);
        const double entryMult=selected.entry_mult;radius=selected.radius;exit_mult=selected.exit_mult;cavitation=selected.cavitation;
        event.radius=radius;event.exit_diameter_mult=exit_mult;event.cavitation=cavitation;
        event.burst=selected.burst;event.vent=selected.vent;event.entry_burst=selected.entry_burst;
        if(painter_log_verbose())Msg("[%s] id=%u phase=forensics sequence=%llu entry_mult=%.3f radius_m=%.4f exit_mult=%.3f cavitation=%.3f burst=%.3f vent=%.3f entry_burst=%.3f head=%u round_class=%d kill_sever=%d",tag,object.ID(),event.sequence,entryMult,radius,exit_mult,cavitation,event.burst,event.vent,event.entry_burst,unsigned(headBone),round_class,kill_sever);}
    event.hit_power=hit.painter_effective_power_valid?hit.painter_effective_power:hit.damage();
    // R62: the body after this hit's damage (the condition has applied it); a killing hit is this very hit's death transition -
    // alive when its receiver stamped it (CAI_Stalker::Hit, before any before-hit Lua) and dead now (a later same-frame pellet on the
    // corpse is not a killing hit)
    event.living_hit=alive;
    event.killing_hit=!alive&&hit.painter_alive_before==1;
    // klass: one line per killing hit, always written: what Lua decided for the round that killed (-1 = field not given).
    if(event.killing_hit)Msg("[%s] id=%u phase=killing_hit sequence=%llu bone=%u ammo=%s radius_m=%.4f penetration_m=%.3f round_class=%d kill_sever=%d",tag,object.ID(),event.sequence,unsigned(hit.bone()),event.ammo_section.c_str(),event.radius,event.penetration_m,event.round_class,event.kill_sever);
    r.ammo_section=ammo.section;r.pellets=unsigned(ammo.pellet_count);r.radius=radius;r.penetration=penetration;r.exit_mult=exit_mult;
    r.policy=fragmentOf?"fragment":"lua";r.status=Status::queued;
    if(fragmentOf){event.explosion_world=fragmentOf->source.world;event.explosion_cookie=fragmentOf->source.cookie;
        event.explosion_target_birth=object.PainterBirthCookie();}
    constexpr bool shrapnel_blades=false;
    if(shrapnel_blades&&fragmentOf&&painter_explosions::next_blade(*fragmentOf,object)){
        const double proximity=distance>=0?std::clamp(1.-distance/6.,0.,1.):.5;
        const double length=.07+.13*proximity,bore=std::clamp(radius*1.8,.005,.010),spacing=bore*1.35;
        const unsigned count=unsigned(std::clamp(int(std::lround(length/spacing))+1,3,14));
        Fvector d=direction_model;d.normalize_safe();Fvector a;a.set(_abs(d.x)<.8f?1.f:0.f,_abs(d.x)<.8f?0.f:1.f,0.f);
        Fvector u;u.crossproduct(d,a);u.normalize_safe();Fvector v;v.crossproduct(d,u);
        const std::uint64_t bs=(event.ammo_seed^0xB1ADEull)*0x9E3779B97F4A7C15ull;const double angle=6.283185307179586*double((bs>>11)&0xFFFFF)/1048576.;
        Fvector along;along.set(u).mul(float(std::cos(angle)));along.mad(v,float(std::sin(angle)));
        const double beta=(double((bs>>40)&0xFFFF)/65535.*2.-1.)*.6981317007977318;
        Fvector across;across.crossproduct(along,d);across.normalize_safe();
        Fvector slant;slant.set(d).mul(float(std::cos(beta)));slant.mad(across,float(std::sin(beta)));slant.normalize_safe();
        const std::uint32_t blade=std::uint32_t(bs>>32)|1u;
        const double depth=std::max(penetration,.06+.10*proximity);
        for(unsigned i=0;i<count;++i){gfy_live::Impact cut=event;cut.sequence=sequence+1+i;
            const double o=(-.5+double(i)/double(count-1))*length;
            cut.accepted_pose_point={point_model.x+along.x*o,point_model.y+along.y*o,point_model.z+along.z*o};
            cut.accepted_pose_direction={slant.x,slant.y,slant.z};cut.blade=blade;
            cut.radius=bore;cut.penetration_m=std::min(.5,depth);cut.exit_diameter_mult=1.1;cut.cavitation=0;cut.burst=0;cut.vent=0;cut.entry_burst=0;
            cut.blast=true;cut.source_search_distance=.2;cut.ammo_seed=event.ammo_seed+i;
            events.push_back(std::move(cut));}
        r.blade=count;return r;}
    events.push_back(std::move(event));
    return r;
}
}
