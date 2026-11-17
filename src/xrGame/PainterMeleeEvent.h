#pragma once
#include "PainterWoundEvent.h"
#include "painter_voxel/MeleeImpact.h"
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

namespace painter_melee_event {
// A hit the melee policy is asked about: value 3, a wound (every knife / axe in the install) or a strike (a blunt blow).
inline bool eligible(const SHit&hit){
    if(gfy_live::mask_only_mode_value()!=3)return false;
    return hit.type()==ALife::eHitTypeWound||hit.type()==ALife::eHitTypeStrike;}
// Events numbered from sequence+1 are appended to `events` (one per blow). `tag` names the owner kind in the log line.
inline painter_wound_event::Result build(CObject&object,IKinematics*k,bool alive,const SHit&hit,u64 sequence,
    std::vector<gfy_live::Impact>&events,const char*tag){
    painter_wound_event::Result r;
    auto reject=[&](const char*why){r.status=painter_wound_event::Status::rejected;r.reason=why;return r;};
    ::luabind::functor<::luabind::object> policy;
    if(!ai().script_engine().functor("_G.CPainterVoxelMelee",policy))return r; // no melee policy loaded: silent, as before
    const bool killing=!alive&&hit.painter_alive_before==1; // this very hit's death transition (R62 rule, PainterWoundEvent.h)
    Fvector pointModel,pointWorld;
    {xrCriticalSectionGuard pose(&k->UCalc_Mutex);k->LL_GetBoneInstance(hit.bone()).mTransform.transform_tiny(pointModel,hit.bone_space_position());}
    object.XFORM().transform_tiny(pointWorld,pointModel);
    ::luabind::object source=::luabind::newtable(ai().script_engine().lua());
    source["owner_id"]=object.ID();source["sequence"]=double(sequence+1);
    source["hit_type"]=hit.type()==ALife::eHitTypeStrike?"strike":"wound";
    source["power"]=double(hit.damage());
    source["accepted_power"]=double(hit.painter_effective_power_valid?hit.painter_effective_power:hit.damage());
    source["impulse"]=double(hit.impulse);
    source["hit_bone"]=unsigned(hit.bone());
    {LPCSTR bone_name=k->LL_BoneName_dbg(hit.bone());source["bone_name"]=bone_name?bone_name:"";}
    source["alive"]=alive;source["killing"]=killing;
    if(hit.weaponID!=u16(-1))source["weapon_id"]=unsigned(hit.weaponID);
    if(const CObject* who=hit.initiator())source["initiator"]=unsigned(who->ID());
    source["point_x"]=double(pointWorld.x);source["point_y"]=double(pointWorld.y);source["point_z"]=double(pointWorld.z);
    {const Fvector&d=hit.direction();source["dir_x"]=double(d.x);source["dir_y"]=double(d.y);source["dir_z"]=double(d.z);}
    const ::luabind::object output=policy(source);
    if(!output||output.type()!=LUA_TTABLE)return r; // Lua: not a melee wound (silent)
    auto number=[&](LPCSTR key,double&value){const ::luabind::object field=output[key];
        if(!field||field.type()!=LUA_TNUMBER)return false;value=::luabind::object_cast<double>(field);return std::isfinite(value);};
    auto optional=[&](LPCSTR key,double&value,double lo,double hi){const ::luabind::object field=output[key];
        if(!field||field.type()==LUA_TNIL)return true;return number(key,value)&&value>=lo&&value<=hi;};
    auto vector3=[&](LPCSTR x,LPCSTR y,LPCSTR z,Fvector&v,bool&given){const ::luabind::object fx=output[x];given=false;
        if(!fx||fx.type()==LUA_TNIL)return true;double a=0,b=0,c=0;if(!number(x,a)||!number(y,b)||!number(z,c))return false;
        v.set(float(a),float(b),float(c));given=true;return true;};
    std::string kindName;{const ::luabind::object kf=output["kind"];if(!kf||kf.type()!=LUA_TSTRING)return reject("invalid_script_melee_kind");
        kindName=::luabind::object_cast<const char*>(kf);}
    gfy_live::melee::Spec spec;
    if(kindName=="slash")spec.kind=gfy_live::melee::slash;else if(kindName=="stab")spec.kind=gfy_live::melee::stab;
    else if(kindName=="chop")spec.kind=gfy_live::melee::chop;else if(kindName=="blunt")spec.kind=gfy_live::melee::blunt;
    else return reject("invalid_script_melee_kind");
    std::string section="melee";{const ::luabind::object sf=output["section"];if(sf&&sf.type()==LUA_TSTRING)section=::luabind::object_cast<const char*>(sf);}
    double length=spec.length,depth=spec.depth,thickness=spec.thickness,reach=0,dentR=0,dentD=0,chunkR=0,chunkD=0,seed=0;
    if(!optional("length_m",length,.005,.6)||!optional("depth_m",depth,.002,.45)||!optional("thickness_m",thickness,.001,.03)||
       !optional("reach_m",reach,0.,1.)||!optional("dent_radius_m",dentR,0.,.15)||!optional("dent_depth_m",dentD,0.,.05)||
       !optional("chunk_radius_m",chunkR,0.,.06)||!optional("chunk_depth_m",chunkD,0.,.08)||!optional("seed",seed,0.,9.007199254740992e15))
        return reject("invalid_script_melee_dimensions");
    bool sever=false;{const ::luabind::object sv=output["sever"];if(sv&&sv.type()==LUA_TBOOLEAN)sever=::luabind::object_cast<bool>(sv);
        else if(sv&&sv.type()!=LUA_TNIL)return reject("invalid_script_melee_sever");}
    // a crushing weapon's killing blow (the sledgehammer): crush = the struck limb / neck is crushed off (torn); head_burst 1 / 2 =
    // the skull bursts (cap / pieces), head_strength 0.2-1 its size (MeleeImpact.h)
    bool crush=false;{const ::luabind::object cf=output["crush"];if(cf&&cf.type()==LUA_TBOOLEAN)crush=::luabind::object_cast<bool>(cf);
        else if(cf&&cf.type()!=LUA_TNIL)return reject("invalid_script_melee_crush");}
    double headBurst=0,headStrength=0;
    if(!optional("head_burst",headBurst,0.,2.)||!optional("head_strength",headStrength,0.,1.))return reject("invalid_script_melee_head");
    // the blow's point and travel: the hit's own, or Lua's (a script strike carries no point, e.g. the exo ram)
    Fvector worldPoint=pointWorld,worldDir=hit.direction(),worldSwing{0,0,0},worldEdge{0,0,0};bool givenPoint=false,givenDir=false,givenSwing=false,givenEdge=false;
    if(!vector3("point_x","point_y","point_z",worldPoint,givenPoint)||!vector3("dir_x","dir_y","dir_z",worldDir,givenDir)||
       !vector3("swing_x","swing_y","swing_z",worldSwing,givenSwing)||!vector3("edge_x","edge_y","edge_z",worldEdge,givenEdge))
        return reject("invalid_script_melee_vector");
    if(worldDir.square_magnitude()<1e-8f)return reject("invalid_script_melee_direction");
    worldDir.normalize_safe();
    const Fmatrix object_world=object.XFORM();Fmatrix world_to_object;world_to_object.invert(object_world);
    Fvector pm,dm,sm{0,0,0},em{0,0,0};
    if(givenPoint)world_to_object.transform_tiny(pm,worldPoint);else pm=pointModel;
    world_to_object.transform_dir(dm,worldDir);
    if(givenSwing)world_to_object.transform_dir(sm,worldSwing);
    if(givenEdge)world_to_object.transform_dir(em,worldEdge);
    spec.point={pm.x,pm.y,pm.z};spec.direction={dm.x,dm.y,dm.z};spec.swing={sm.x,sm.y,sm.z};spec.edge={em.x,em.y,em.z};
    spec.length=length;spec.depth=depth;spec.thickness=thickness;spec.reach=reach;spec.sever=sever;
    spec.dent_radius=dentR;spec.dent_depth=dentD;spec.chunk_radius=chunkR;spec.chunk_depth=chunkD;
    spec.crush=crush;spec.head=std::uint8_t(std::lround(headBurst));spec.head_strength=headStrength;
    const std::uint64_t fs=(std::uint64_t(hit.BulletID)<<32)^(std::uint64_t(sequence+1)*0x9E3779B97F4A7C15ull)^(std::uint64_t(hit.bone())<<48)^std::uint64_t(object.ID())^0x3E1EEull;
    spec.seed=seed>0?std::uint64_t(seed):(fs?fs:1);
    // what every event shares (as PainterWoundEvent.h freezes a bullet's): the pose, the gore menu, the bone, the power, the outcome
    gfy_live::Impact base;base.sequence=sequence+1;base.gore=painter_deformation::snapshot();base.cranial_bone=k->LL_BoneID("bip01_head");
    {xrCriticalSectionGuard pose(&k->UCalc_Mutex);k->CalculateBones_Invalidate();k->CalculateBones(TRUE);
        base.bone_model_render.reserve(k->LL_BoneCount());for(u16 b=0;b<k->LL_BoneCount();++b)base.bone_model_render.push_back(painter_wound_event::matrix(k->LL_GetBoneInstance(b).mRenderTransform));}
    base.ammo_section=section;base.hit_bone=hit.bone();base.initiator=hit.whoID;
    base.hit_power=hit.painter_effective_power_valid?hit.painter_effective_power:hit.damage();
    base.living_hit=alive;base.killing_hit=killing;base.round_class=0;base.kill_sever=0; // a bullet's kill sever / head burst only as fill() sets them
    std::vector<gfy_live::Impact> made;
    if(!gfy_live::melee::fill(spec,base,made)||made.empty())return reject("melee_event_shape");
    r.ammo_section=section;r.radius=made.front().radius;r.penetration=made.front().penetration_m;r.policy="melee";r.status=painter_wound_event::Status::queued;
    Msg("[%s] id=%u phase=melee sequence=%llu kind=%s section=%s bone=%u alive=%u killing=%u sever=%u length_m=%.3f depth_m=%.3f reach_m=%.3f dent=%.3f/%.3f chunk=%.3f/%.3f crush=%u head=%u/%.2f",
        tag,object.ID(),sequence+1,gfy_live::melee::name(spec.kind),section.c_str(),unsigned(hit.bone()),unsigned(alive),unsigned(killing),unsigned(sever&&spec.kind!=gfy_live::melee::stab),
        length,depth,reach,dentR,dentD,chunkR,chunkD,unsigned(made.front().kill_sever==1),unsigned(made.front().melee_head),made.front().melee_head_strength);
    for(auto&e:made)events.push_back(std::move(e));
    return r;
}
}
