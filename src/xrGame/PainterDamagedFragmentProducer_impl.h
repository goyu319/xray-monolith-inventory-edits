#pragma once
// Owner-barrier adapter for a completed, immutable CPU separation certificate.
// The CPU worker owns anatomy, surface and physical-element provenance; this
// file only maps its checked original-bone-frame payload into native objects.
#include "PainterFlightTiming.h"
#include "painter_voxel/PainterDamagedFragment.h"
#include "painter_voxel/FlightGroup.h"
#include "painter_voxel/CollisionLimits.h"
#include "ai/stalker/ai_stalker.h"
#include "../Include/xrRender/Kinematics.h"
#include "../xrPhysics/PhysicsShell.h"
#include <set>
#include <string>
#include <algorithm>
#include <cmath>

namespace painter_damaged_fragment {
namespace producer {
inline bool point(Fvector& dst,const gfy_live::Point& p){
    if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return false;
    dst.set(float(p.x),float(p.y),float(p.z));
    return std::isfinite(dst.x)&&std::isfinite(dst.y)&&std::isfinite(dst.z);
}
// Nearest rotation (+ unchanged origin) of a world matrix within 10 % of rigid (see PainterDamagedFragment_impl.h).
inline bool orthonormal(Fmatrix& m,float& deviation){
    deviation=0.f;
    auto ok=[](const Fvector& v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);};
    if(!ok(m.i)||!ok(m.j)||!ok(m.k)||!ok(m.c))return false;
    deviation=_max(_max(_abs(m.i.magnitude()-1),_abs(m.j.magnitude()-1)),_abs(m.k.magnitude()-1));
    deviation=_max(deviation,_max(_max(_abs(m.i.dotproduct(m.j)),_abs(m.i.dotproduct(m.k))),_abs(m.j.dotproduct(m.k))));
    if(!(deviation<.1f))return false;
    Fvector i=m.i,j=m.j,k;i.normalize();j.mad(i,-i.dotproduct(j));j.normalize();k.crossproduct(i,j);
    if(k.dotproduct(m.k)<0.f)k.invert();
    m.i=i;m.j=j;m.k=k;m._14_=0.f;m._24_=0.f;m._34_=0.f;m._44_=1.f;
    return true;
}
// Why the last fitted() refused its part (for the refusal log lines).
inline const char*& fitted_refusal(){static const char* why="";return why;}
inline bool fitted(const gfy_live::SeparationPiece& input,
    std::vector<PHFittedBone>& output,u64& visible,u16 bones){
    fitted_refusal()="";
    if(input.elements.empty()||input.elements.size()>64||
       input.root_bone>=bones||input.pose_anchors.size()!=bones){fitted_refusal()="part_input";return false;}
    visible=0;output.reserve(input.elements.size());std::size_t total_boxes=0;
    for(const auto& source:input.elements){
        total_boxes+=source.boxes.size();
        if(source.bone>=bones||source.boxes.size()>painter_collision_limits::boxes_per_element||
           total_boxes>painter_collision_limits::boxes_per_shell||
           (source.retain_original_shape?!source.boxes.empty():source.boxes.empty())||
           !source.cells||
           (!source.retain_original_shape&&
             (!std::isfinite(source.mass_kg)||source.mass_kg<=0))||
           visible&(1ull<<source.bone)){
            fitted_refusal()=source.bone>=bones?"bone_index":source.boxes.size()>painter_collision_limits::boxes_per_element?"boxes_per_element":
                total_boxes>painter_collision_limits::boxes_per_shell?"boxes_per_shell":!source.cells?"element_empty":
                (visible&(1ull<<source.bone))?"bone_twice":(source.retain_original_shape?!source.boxes.empty():source.boxes.empty())?"box_list":"element_mass";
            if(source.boxes.size()>painter_collision_limits::boxes_per_element||total_boxes>painter_collision_limits::boxes_per_shell){
                // R34: which elements carry the boxes (bone:boxes, * = original shape), for the refusal log line.
                static std::string detail;detail=fitted_refusal();detail+=" elements=";detail+=std::to_string(input.elements.size());detail+=" boxes=";
                for(const auto& e:input.elements){detail+=std::to_string(e.bone);detail+=e.retain_original_shape?":*":":";
                    if(!e.retain_original_shape)detail+=std::to_string(e.boxes.size());detail+=",";}
                fitted_refusal()=detail.c_str();}
            return false;}
        PHFittedBone target;target.bone=source.bone;target.mass_center.set(0.f,0.f,0.f);
        target.retain_original_shape=source.retain_original_shape;
        if(target.retain_original_shape){ // bodyjitter3: a native element of the remaining body cut back at a cut
            if(source.trims.size()>4||!(source.mass_scale>0&&source.mass_scale<=1)){fitted_refusal()="trim";return false;}
            for(const auto& t:source.trims){for(unsigned q=0;q<4;++q){if(!std::isfinite(t[q])){fitted_refusal()="trim";return false;}
                target.trim[target.trims][q]=float(t[q]);}++target.trims;}
            target.mass_scale=float(source.mass_scale);
        }
        if(!target.retain_original_shape){
            const double massScale=source.mass_kg>0&&source.mass_kg<.15?.15/source.mass_kg:1.;
            target.mass=float(source.mass_kg*massScale);
            if(!std::isfinite(target.mass)||!point(target.mass_center,source.com))return false;
            for(unsigned i=0;i<9;++i){
                if(!std::isfinite(source.inertia[i]))return false;
                target.inertia[i]=float(source.inertia[i]*massScale);
                if(!std::isfinite(target.inertia[i]))return false;
            }
        }
        target.boxes.reserve(source.boxes.size());
        for(const auto& box:source.boxes){
            Fobb native;
            if(!point(native.m_translate,box.center)||
               !point(native.m_halfsize,box.half_extents)){fitted_refusal()="box_values";return false;}
            for(double v:box.orientation)if(!std::isfinite(v)){fitted_refusal()="box_values";return false;}
            // A sliver's box can be thinner than the 5 mm the physics accepts: widen it rather than lose the part.
            native.m_halfsize.x=_max(native.m_halfsize.x,.005f);native.m_halfsize.y=_max(native.m_halfsize.y,.005f);
            native.m_halfsize.z=_max(native.m_halfsize.z,.005f);
            const auto& o=box.orientation;
            native.m_rotate.i.set(float(o[0]),float(o[3]),float(o[6]));
            native.m_rotate.j.set(float(o[1]),float(o[4]),float(o[7]));
            native.m_rotate.k.set(float(o[2]),float(o[5]),float(o[8]));
            target.boxes.push_back(native);
        }
        visible|=1ull<<source.bone;
        output.push_back(std::move(target));
    }
    if(!(visible&(1ull<<input.root_bone))){fitted_refusal()="root_not_in_part";return false;}
    return true;
}
// gearbody: a part's merged gear boxes (SeparationPiece::gear_flesh, bone frame) for the bullet flesh test - never physics.
inline void flesh_gear(const gfy_live::SeparationPiece& input,std::vector<PHFittedBone>& output){
    output.clear();
    for(const auto& source:input.gear_flesh){
        PHFittedBone target;target.bone=source.bone;target.mass_center.set(0.f,0.f,0.f);
        for(const auto& box:source.boxes){
            Fobb native;bool finite=point(native.m_translate,box.center)&&point(native.m_halfsize,box.half_extents);
            for(double v:box.orientation)finite=finite&&std::isfinite(v);
            if(!finite)continue;
            native.m_halfsize.x=_max(native.m_halfsize.x,.005f);native.m_halfsize.y=_max(native.m_halfsize.y,.005f);
            native.m_halfsize.z=_max(native.m_halfsize.z,.005f);
            const auto& o=box.orientation;
            native.m_rotate.i.set(float(o[0]),float(o[3]),float(o[6]));
            native.m_rotate.j.set(float(o[1]),float(o[4]),float(o[7]));
            native.m_rotate.k.set(float(o[2]),float(o[5]),float(o[8]));
            target.boxes.push_back(native);
        }
        if(!target.boxes.empty())output.push_back(std::move(target));
    }
}
inline bool decisions(const gfy_live::SeparationOutput& input,
    u16 original_children,Prepared& output){
    if(!original_children||original_children>=gfy_packed::max_child_slots)return false;
    std::vector<bool> surfaces(original_children,false);
    for(u16 child:input.original_surface_children){
        if(child>=original_children||surfaces[child])return false;
        surfaces[child]=true;
    }
    if(input.original_surface_children.empty())return false;
    output.original_roles.reserve(original_children);
    for(bool surface:surfaces)output.original_roles.push_back(surface?
        part_child_ownership::Role::SurfaceRequiresPartition:
        part_child_ownership::Role::WholeAccessory);
    auto plan=[&](const gfy_live::SeparationPiece& piece,
                  std::vector<part_child_ownership::Decision>& out){
        std::vector<bool> packed(original_children,false),retained(original_children,false);
        for(const auto& child:piece.mesh.children)if(child.child_index<original_children){
            if(packed[child.child_index])return false;
            packed[child.child_index]=!child.vertices.empty()&&!child.indices.empty();
        }
        for(u16 child:piece.retained_accessories){
            if(child>=original_children||surfaces[child]||retained[child]||packed[child])return false;
            retained[child]=true;
        }
        out.reserve(original_children);
        for(u16 child=0;child<original_children;++child){
            auto action=packed[child]?part_child_ownership::Action::ReplaceWithPacked:
                retained[child]?part_child_ownership::Action::RetainOriginalAccessory:
                part_child_ownership::Action::RemoveOriginal;
            out.push_back({child,action});
        }
        return true;
    };
    for(const auto& piece:input.pieces){
        if(piece.component==input.source_component){
            if(!plan(piece,output.stump_children))return false;
        }else{
            auto it=std::find_if(output.pieces.begin(),output.pieces.end(),
                [&](const Piece& fragment){return fragment.component_id==piece.component;});
            if(it==output.pieces.end()||!plan(piece,it->children))return false;
        }
    }
    return true;
}
}

inline bool submit_cpu(CAI_Stalker& source,const gfy_live::Identity& identity,
    const gfy_live::SeparationOutput& cpu,u16 original_children,
    const gfy_live::SeparationInput& continuation_input,
    const xr_vector<RuntimeSkinBone>& rig,const xr_vector<shared_str>& textures,
    CPhysicsShell* shell,const char*& reason,std::shared_ptr<painter_flight::Group> flight={}){
    painter_flight_perf::Scope flightTime("body_spawn");
    reason="separation_cpu_refused";
    auto* k=smart_cast<IKinematics*>(source.Visual());
    if(!k||!shell||cpu.no_separation||!cpu.error.empty()||
       !(cpu.stamp.owner==identity)||cpu.stamp.expected_generation!=k->LL_RenderGeometryGeneration()||
       cpu.pieces.size()<2||cpu.pieces.size()>13||rig.size()!=k->LL_BoneCount()||
       k->LL_BoneCount()>64)return false;
    Prepared output;output.flight=std::move(flight);output.stamp=cpu.stamp;output.gore=cpu.gore;output.source_id=source.ID();
    output.current_owner=identity;
    output.expected_generation=cpu.stamp.expected_generation;
    output.publication_revision=cpu.stamp.revision;
    output.continuation_input=continuation_input;
    output.source_visual=source.cNameVisual();output.source_world=source.XFORM();
    {float deviation=0.f;
     if(!producer::orthonormal(output.source_world,deviation)){
        const Fmatrix& m=source.XFORM();
        Msg("[painter_damaged_fragment] id=%u phase=source_world_refused deviation=%.4f axes=%.4f,%.4f,%.4f",source.ID(),deviation,m.i.magnitude(),m.j.magnitude(),m.k.magnitude());
        reason="source_world";return false;}
     if(deviation>2e-3f)Msg("[painter_damaged_fragment] id=%u phase=source_world_normalized deviation=%.4f",source.ID(),deviation);}
    std::set<u32> components;bool found_source=false;
    for(const auto& piece:cpu.pieces){
        if(!piece.component||!piece.cells||!components.insert(piece.component).second){
            reason="separation_component_identity";return false;
        }
        if(piece.component==cpu.source_component){
            found_source=true;output.stump=piece.mesh;
            output.stump_continuation=piece.continuation;output.stump_seed=piece.seed;
            output.stump_root_bone=piece.root_bone;
            output.stump_pose_anchors=piece.pose_anchors;
            producer::flesh_gear(piece,output.stump_flesh_gear); // gearbody
            if(!producer::fitted(piece,output.stump_physics,output.stump_visible_bones,
                   u16(rig.size()))){reason="separation_stump_physics";
                Msg("[painter_damaged_fragment] id=%u phase=fitted_refused part=stump reason=%s",source.ID(),producer::fitted_refusal());return false;}
        }else{
            auto& fragment=output.pieces.emplace_back();
            if(output.flight)fragment.flight_visual=piece.flight_visual;
            fragment.ocular_remnant=piece.ocular_remnant;
            fragment.burst_chunk=piece.burst_chunk; // chunknosplash
            fragment.destroy_on_commit=piece.ocular_remnant&&cpu.gore.eye_remnants_mode==0;
            fragment.component_id=piece.component;fragment.mesh=piece.mesh;
            fragment.continuation=piece.continuation;fragment.seed=piece.seed;
            fragment.root_bone=piece.root_bone;fragment.anchor=piece.root_bone;
            fragment.pose_anchors=piece.pose_anchors;
            producer::flesh_gear(piece,fragment.flesh_gear); // gearbody
            if(!producer::fitted(piece,fragment.fitted,fragment.visible_bones,
                   u16(rig.size()))){reason="separation_fragment_physics";
                Msg("[painter_damaged_fragment] id=%u phase=fitted_refused part=piece reason=%s",source.ID(),producer::fitted_refusal());return false;}
        }
    }
    if(!found_source||!producer::decisions(cpu,original_children,output)){
        reason="separation_child_ownership";return false;
    }
    {
        xrCriticalSectionGuard guard(&k->UCalc_Mutex);
        k->CalculateBones_Invalidate();k->CalculateBones(TRUE);
        output.captured_render_palette.resize(rig.size());
        for(u16 b=0;b<rig.size();++b)
            output.captured_render_palette[b]=k->LL_GetBoneInstance(b).mRenderTransform;
        for(auto& fragment:output.pieces){
            const auto& bone=k->LL_GetBoneInstance(fragment.anchor);
            fragment.anchor_model=bone.mTransform;
            fragment.anchor_render=bone.mRenderTransform;
            float deviation=0.f;
            if(!producer::orthonormal(fragment.anchor_model,deviation)||!producer::orthonormal(fragment.anchor_render,deviation))
                Msg("[painter_damaged_fragment] id=%u phase=anchor_not_rigid bone=%u deviation=%.4f",source.ID(),unsigned(fragment.anchor),deviation);
        }
    }
    return submit(source,identity,output,rig,textures,shell,reason);
}
}
