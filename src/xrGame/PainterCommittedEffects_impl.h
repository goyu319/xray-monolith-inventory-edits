#pragma once
// Owner-thread delivery of committed wound effects. The Lua queue has bounded
// admission: retain immutable rest-space data while it is full, and reconstruct
// world-space mouths only on the actual delivery attempt. No worker calls Lua.
#include "painter_voxel/LiveDamageWorker.h"
#include "../Include/xrRender/PainterLogVerbose.h"
#include "painter_voxel/CommittedReceiptQueue.h"

namespace painter_committed_effects {
using Queue=gfy_live::CommittedReceiptQueue;
inline bool capacity_available(){
    ::luabind::functor<int> capacity;
    // Old script versions did not expose admission. Preserve their one-call
    // behavior; updated scripts return zero before any effect side effects.
    return !ai().script_engine().functor("_G.CPainterVoxelReceiptCapacity",capacity)||capacity()>0;
}
inline bool drain(CPhysicsShellHolder& object,Queue& pending,bool allow_sever){
    auto* k=smart_cast<IKinematics*>(object.Visual());
    if(!k||object.getDestroy()){pending.clear();return true;}
    ::luabind::functor<bool> paint;
    if(!ai().script_engine().functor("_G.CPainterVoxelCommitted",paint)){
        Msg("[painter_voxel_live] id=%u phase=wound_blood_invalidated reason=callback_unavailable receipts=%u",object.ID(),unsigned(pending.size()));
        pending.clear();return true;
    }
    auto* stalker=smart_cast<CAI_Stalker*>(&object);
    return pending.drain([&](const gfy_live::ImpactResult& impact){
        if(impact.hit_bone>=k->LL_BoneCount()||k->LL_BoneCount()>impact.entry_surface.weights.size()){
            Msg("[painter_voxel_live] id=%u phase=wound_blood_invalidated sequence=%llu reason=rig_changed",object.ID(),impact.sequence);
            return true;
        }
        if(!capacity_available())return false;
        ::luabind::object receipt=::luabind::newtable(ai().script_engine().lua());
            receipt["sequence"]=double(impact.sequence);receipt["ammo_section"]=impact.ammo_section.c_str();
            receipt["bone"]=k->LL_BoneName_dbg(impact.hit_bone);receipt["radius"]=impact.radius;
            receipt["exit_radius"]=impact.exit_radius;receipt["power"]=impact.hit_power;
            receipt["penetrated"]=impact.penetrated;
            if(impact.initiator!=65535)receipt["initiator"]=double(impact.initiator);
            // melee (round 3, value 3): a blade blow's cut - the chop splash fields (LiveDamageWorker.cpp melee_cut_*; W/melee/INSTALL.md
            // "chop splash fields"): receipt.melee_cut = {id, blow, length_m, depth_m, fraction, severed, rest_centre, rest_normal,
            // rest_dir, rest_edge, centre, normal, dir, edge}; world = rest through the struck bone's current transform, at delivery
            if(impact.melee_cut_id){::luabind::object mc=::luabind::newtable(ai().script_engine().lua());
                mc["id"]=double(impact.melee_cut_id);mc["blow"]=double(impact.melee_cut_blow);mc["length_m"]=impact.melee_cut_length;
                mc["depth_m"]=impact.melee_cut_depth;mc["fraction"]=impact.melee_cut_fraction;mc["severed"]=impact.sever;
                const Fvector rc={float(impact.melee_cut_centre.x),float(impact.melee_cut_centre.y),float(impact.melee_cut_centre.z)};
                const Fvector rn={float(impact.melee_cut_normal.x),float(impact.melee_cut_normal.y),float(impact.melee_cut_normal.z)};
                const Fvector rd={float(impact.melee_cut_dir.x),float(impact.melee_cut_dir.y),float(impact.melee_cut_dir.z)};
                const Fvector re={float(impact.melee_cut_edge.x),float(impact.melee_cut_edge.y),float(impact.melee_cut_edge.z)};
                Fvector mcp={0,0,0},mn={0,0,0},md={0,0,0},me={0,0,0};
                {xrCriticalSectionGuard guard(&k->UCalc_Mutex);k->CalculateBones(TRUE);const Fmatrix&bt=k->LL_GetBoneInstance(impact.hit_bone).mRenderTransform;
                 bt.transform_tiny(mcp,rc);bt.transform_dir(mn,rn);bt.transform_dir(md,rd);bt.transform_dir(me,re);}
                Fvector wc={0,0,0},wn={0,0,0},wd={0,0,0},we={0,0,0};object.XFORM().transform_tiny(wc,mcp);object.XFORM().transform_dir(wn,mn);
                object.XFORM().transform_dir(wd,md);object.XFORM().transform_dir(we,me);wn.normalize_safe();wd.normalize_safe();we.normalize_safe();
                mc["rest_centre"]=rc;mc["rest_normal"]=rn;mc["rest_dir"]=rd;mc["rest_edge"]=re;mc["centre"]=wc;mc["normal"]=wn;mc["dir"]=wd;mc["edge"]=we;
                receipt["melee_cut"]=mc;}
            {::luabind::object f=::luabind::newtable(ai().script_engine().lua());
                f["cavitation"]=impact.cavitation;f["burst"]=impact.burst;f["vent"]=impact.vent;f["vented"]=impact.vented;
                f["vent_radius"]=impact.vent_radius;f["entry_burst"]=impact.entry_burst;f["entry_radius"]=impact.entry_radius;f["exit_mult"]=impact.radius>0?impact.exit_radius/impact.radius:1.;receipt["forensics"]=f;}
            // Removed anatomy per material (m3): fragments (brain, bone chips) scale with what the cut destroyed.
            {double total=0,organ=0;for(unsigned s=1;s<64;++s){total+=impact.removed_m3_by_slot[s];if(s>=7&&s<=14)organ+=impact.removed_m3_by_slot[s];}
            receipt["removed_m3"]=total;receipt["removed_organ_m3"]=organ;}
            receipt["removed_bone_m3"]=impact.removed_m3_by_slot[2];receipt["removed_brain_m3"]=impact.removed_m3_by_slot[3];
            receipt["removed_muscle_m3"]=impact.removed_m3_by_slot[5];
            // 2026-10-17 read-out: the weakest tear station this cut touched when nothing tore, so a body that looks cut yet
            // stays whole can be read from the log (tissue/bone left in that 12 mm slab).
            if(!impact.sever&&impact.sever_probe_bone<k->LL_BoneCount())
                Msg("[painter_voxel_live] id=%u phase=no_tear sequence=%llu bone=%u tissue_left=%.2f bone_left=%.2f",object.ID(),impact.sequence,
                    unsigned(impact.sever_probe_bone),impact.sever_probe_tissue,impact.sever_probe_bone_left);
            // T1 (2026-10-29): a piece's sever gets its line too (no Lua sever: a piece is already a corpse's part), with its torn end
            if(!allow_sever&&impact.sever)
                Msg("[painter_voxel_live] id=%u phase=sever piece=1 sequence=%llu bone=%u tissue_left=%.2f bone_left=%.2f cells=%u %s",object.ID(),impact.sequence,unsigned(impact.sever_bone),
                    impact.sever_remaining_tissue,impact.sever_remaining_bone,unsigned(impact.sever_cells),gfy_live::tornEndText(impact).c_str());
            if(allow_sever&&impact.sever&&impact.sever_bone<k->LL_BoneCount()){::luabind::object sever=::luabind::newtable(ai().script_engine().lua());
                sever["bone"]=k->LL_BoneName_dbg(impact.sever_bone);sever["dangling"]=impact.sever_dangling;
                sever["tissue_left"]=impact.sever_remaining_tissue;sever["bone_left"]=impact.sever_remaining_bone;
                Fvector rest={float(impact.sever_point.x),float(impact.sever_point.y),float(impact.sever_point.z)},model={0,0,0},world={0,0,0};
                {xrCriticalSectionGuard guard(&k->UCalc_Mutex);k->CalculateBones(TRUE);
                    if(impact.sever_bone<k->LL_BoneCount())k->LL_GetBoneInstance(impact.sever_bone).mRenderTransform.transform_tiny(model,rest);}
                object.XFORM().transform_tiny(world,model);sever["point"]=world;receipt["sever"]=sever;
                painter_damaged_fragment::arm_cut(object.ID(),impact.sever_bone,rest,{float(impact.sever_axis.x),float(impact.sever_axis.y),float(impact.sever_axis.z)});
                const bool cranial=impact.sever_bone==k->LL_BoneID("bip01_head")||impact.sever_bone==k->LL_BoneID("bip01_neck");
                if(impact.sever_dangling&&impact.sever_bone<k->LL_BoneCount()&&(!cranial||impact.gore.head_remnants_enabled))
                    painter_damaged_fragment::arm_tether(object.ID(),impact.sever_bone,rest,
                        1500.f*float(cranial?impact.gore.head_tether_strength_mult:1.),u32(24u*3600u*1000u));
                // R62 head burst events (HeadFracture.h): only a shot's skull cap / head-in-pieces reports a sever on the head bone
                // itself (no sever row ends there); the pieces the corpse's split makes are thrown out from the head like blast pieces
                // (PainterDamagedFragment_impl.h launch rule, 4 s window).
                if(impact.sever_bone==k->LL_BoneID("bip01_head")&&!painter_explosions::blast_on(object.ID()))
                    painter_explosions::note_blast(object.ID(),world,.5f);
                // A torso tear marks the cut for the intestine spill of the split it causes (PainterGutSpill_impl.h).
                if(impact.sever_bone==k->LL_BoneID("bip01_pelvis")||impact.sever_bone==k->LL_BoneID("bip01_spine")||impact.sever_bone==k->LL_BoneID("bip01_spine1")){
                    const Fvector axis={float(impact.sever_axis.x),float(impact.sever_axis.y),float(impact.sever_axis.z)};
                    painter_gut_spill::arm(object.ID(),impact.sever_bone,rest,axis,impact.gore);}
                Msg("[painter_voxel_live] id=%u phase=sever sequence=%llu bone=%u dangling=%u tissue_left=%.2f bone_left=%.2f cells=%u alive=%u %s",object.ID(),impact.sequence,unsigned(impact.sever_bone),unsigned(impact.sever_dangling),impact.sever_remaining_tissue,impact.sever_remaining_bone,unsigned(impact.sever_cells),unsigned(stalker&&stalker->g_Alive()),gfy_live::tornEndText(impact).c_str());
                ::luabind::functor<bool> severed;
                if(ai().script_engine().functor("_G.CPainterVoxelSevered",severed)){const bool handled=severed(object.lua_game_object(),receipt);
                    if(painter_log_verbose())Msg("[painter_voxel_live] id=%u phase=sever_callback handled=%u",object.ID(),unsigned(handled));}}
            // Intestine spill from a big abdominal exit or burst entry that destroyed intestine (slot 11): the mouth in
            // the hit bone's frame, a chance from its size (PainterGutSpill_impl.h: the loop comes out on the ragdoll).
            if(impact.hit_bone==k->LL_BoneID("bip01_pelvis")||impact.hit_bone==k->LL_BoneID("bip01_spine")||impact.hit_bone==k->LL_BoneID("bip01_spine1")){
                // Use the struck-side opening when it is actually wide enough.
                // A rear exit is a fallback, rather than overriding an open abdominal entry.
                const double entry_radius=std::max(impact.radius,impact.entry_radius);
                const bool entry_mouth=impact.entry_surface.valid&&entry_radius>=.02;
                const bool exit_mouth=!entry_mouth&&impact.penetrated&&impact.exit_surface.valid&&impact.exit_radius>=.02;
                if((exit_mouth||entry_mouth)&&impact.removed_m3_by_slot[11]>=1.5e-6&&(exit_mouth?impact.exit_surface:impact.entry_surface).weights.size()>=k->LL_BoneCount()){
                    const auto& s=exit_mouth?impact.exit_surface:impact.entry_surface;const double r=exit_mouth?impact.exit_radius:entry_radius;
                    const Fvector rp={float(s.rest_point.x),float(s.rest_point.y),float(s.rest_point.z)},rn={float(s.rest_normal.x),float(s.rest_normal.y),float(s.rest_normal.z)};
                    Fvector mp={0,0,0},mn={0,0,0};Fmatrix bone_world;
                    {xrCriticalSectionGuard guard(&k->UCalc_Mutex);k->CalculateBones(TRUE);
                     for(u16 b=0;b<k->LL_BoneCount();++b)if(s.weights[b]>0){Fvector p,n;
                        k->LL_GetBoneInstance(b).mRenderTransform.transform_tiny(p,rp);k->LL_GetBoneInstance(b).mRenderTransform.transform_dir(n,rn);
                        mp.mad(p,float(s.weights[b]));mn.mad(n,float(s.weights[b]));}
                     bone_world.mul_43(object.XFORM(),k->LL_GetTransform(impact.hit_bone));}
                    Fvector wp,wn;object.XFORM().transform_tiny(wp,mp);object.XFORM().transform_dir(wn,mn);wn.normalize_safe();
                    Fmatrix inv;inv.invert(bone_world);Fvector lp,ln;inv.transform_tiny(lp,wp);inv.transform_dir(ln,wn);
                    const float chance=float(std::clamp(.25+8.*(r-.02),.25,.8))+(painter_explosions::blast_on(object.ID())?.2f:0.f);
                    painter_gut_spill::wound(object.ID(),impact.hit_bone,lp,ln,chance,impact.gore,float(r));}}
            auto surface=[&](const gfy_live::WoundSurface& source,LPCSTR prefix){
                Fvector model_point={0,0,0},model_normal={0,0,0};
                Fvector rest_point={float(source.rest_point.x),float(source.rest_point.y),float(source.rest_point.z)};
                Fvector rest_normal={float(source.rest_normal.x),float(source.rest_normal.y),float(source.rest_normal.z)};
                {xrCriticalSectionGuard guard(&k->UCalc_Mutex);k->CalculateBones(TRUE);
                    for(u16 b=0;b<k->LL_BoneCount();++b)if(source.weights[b]>0){Fvector p,n;
                        k->LL_GetBoneInstance(b).mRenderTransform.transform_tiny(p,rest_point);
                        k->LL_GetBoneInstance(b).mRenderTransform.transform_dir(n,rest_normal);
                        model_point.mad(p,float(source.weights[b]));model_normal.mad(n,float(source.weights[b]));}}
                Fvector world_point,world_normal;object.XFORM().transform_tiny(world_point,model_point);
                object.XFORM().transform_dir(world_normal,model_normal);world_normal.normalize_safe();
                ::luabind::object row=::luabind::newtable(ai().script_engine().lua());
                row["point"]=world_point;row["normal"]=world_normal;
                // Lua admits this mouth now but may project its original/halo decals several frames later.
                // Preserve the full source blend, including multi-bone knee mouths; a single hit bone is not enough.
                // bone_transform() supplies current world bone matrices; multiplying these immutable bind rows
                // reconstructs the same current skinning transform used above without retaining any native owner.
                auto anchor=::luabind::newtable(ai().script_engine().lua());
                auto bones=::luabind::newtable(ai().script_engine().lua());unsigned count=0;
                for(u16 b=0;b<k->LL_BoneCount();++b)if(source.weights[b]>0){
                    auto binding=::luabind::newtable(ai().script_engine().lua());
                    binding["id"]=unsigned(b);binding["weight"]=source.weights[b];
                    binding["bind"]=k->LL_GetData(b).m2b_transform;bones[++count]=binding;
                }
                anchor["bones"]=bones;row["anchor"]=anchor;receipt[prefix]=row;
            };
            surface(impact.entry_surface,"entry");if(impact.penetrated)surface(impact.exit_surface,"exit");if(impact.vented)surface(impact.vent_surface,"vent");
            // bulletburst (BulletBurst.h): a bullet's body burst or surface blowout - no sever, no kill. receipt.body_burst: 1 blowout, 2
            // burst (a corpse: its fragments come off in the separation), 3 a burst blown out (a crater). A corpse burst's fragments
            // are thrown out from behind the burst, as a head burst's pieces (PainterDamagedFragment_impl.h launch rule, 4 s window).
            if(impact.bullet_burst){receipt["body_burst"]=double(impact.bullet_burst);
                Msg("[painter_voxel_live] id=%u phase=body_burst sequence=%llu kind=%u bone=%u alive=%u",object.ID(),impact.sequence,unsigned(impact.bullet_burst),unsigned(impact.hit_bone),unsigned(stalker&&stalker->g_Alive()));
                if(impact.bullet_burst==2&&impact.hit_bone<k->LL_BoneCount()&&!painter_explosions::blast_on(object.ID())){
                    const Fvector rest={float(impact.bullet_burst_point.x),float(impact.bullet_burst_point.y),float(impact.bullet_burst_point.z)};Fvector model={0,0,0},world={0,0,0};
                    {xrCriticalSectionGuard guard(&k->UCalc_Mutex);k->CalculateBones(TRUE);k->LL_GetBoneInstance(impact.hit_bone).mRenderTransform.transform_tiny(model,rest);}
                    object.XFORM().transform_tiny(world,model);
                    const float s=float(impact.bullet_burst_strength);painter_explosions::note_blast(object.ID(),world,s<.15f?.15f:(s>.6f?.6f:s));}}
            const bool submitted=paint(object.lua_game_object(),receipt);
            if(!submitted||painter_log_verbose())Msg("[painter_voxel_live] id=%u phase=wound_blood sequence=%llu penetration=%u vented=%u burst=%.2f submitted=%u",object.ID(),impact.sequence,unsigned(impact.penetrated),unsigned(impact.vented),impact.burst,unsigned(submitted));
        // submitted=false is an explicit script outcome (disabled/invalid or
        // failed capture), not an admission retry. Retrying side effects would
        // duplicate debris, death callbacks or tethers. Capacity is tested first.
        return true;
    });
}
}
