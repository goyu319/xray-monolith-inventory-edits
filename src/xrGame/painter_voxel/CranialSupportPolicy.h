#pragma once
// Pure support decision. The measured volumes are the worker's cumulative,
// original-anatomy joint measures, not bullet power or a replacement model.
#include <algorithm>
#include <cmath>
namespace painter_cranial_support {
enum class State { unavailable, supported, hanging, separated };
struct Decision {
    State state=State::unavailable;
    double tissue_fraction=1.,bone_fraction=1.;
    double stiffness_scale=1.;
};
inline Decision evaluate(double original_tissue,double removed_tissue,
    double original_bone,double removed_bone,double strength) {
    Decision out;
    if(!std::isfinite(original_tissue)||!std::isfinite(removed_tissue)||
       !std::isfinite(original_bone)||!std::isfinite(removed_bone)||
       !std::isfinite(strength)||original_tissue<=0.||original_bone<=0.||
       removed_tissue<0.||removed_bone<0.||strength<=0.||
       original_bone>original_tissue*(1.+1e-6)||
       removed_tissue>original_tissue*(1.+1e-6)||
       removed_bone>original_bone*(1.+1e-6)||removed_bone>removed_tissue*(1.+1e-6))return out;
    out.tissue_fraction=std::clamp(1.-removed_tissue/original_tissue,0.,1.);
    out.bone_fraction=std::clamp(1.-removed_bone/original_bone,0.,1.);
    // No surviving bridge is a separation decision. Do not invent a tether.
    if(out.tissue_fraction<=1e-6){out.state=State::separated;return out;}
    out.state=State::supported;
    // Only measured loss of most skeletal support unlocks the original head
    // joint. These thresholds are game tuning, not forensic probabilities.
    if(out.bone_fraction<=.25&&out.tissue_fraction<=.65){
        out.state=State::hanging;
        out.stiffness_scale=std::clamp((.025+.25*out.bone_fraction)*strength,.01,1.);
    }
    return out;
}
inline Decision evaluateCranial(double neck_tissue,double removed_neck_tissue,
    double neck_bone,double removed_neck_bone,double cranial_tissue,double removed_cranial_tissue,
    double skull,double removed_skull,double brain,double removed_brain,double strength){
    auto out=evaluate(neck_tissue,removed_neck_tissue,neck_bone,removed_neck_bone,strength);
    if(out.state!=State::supported)return out;
    const auto cranial=evaluate(cranial_tissue,removed_cranial_tissue,skull,removed_skull,strength);
    if(cranial.state==State::unavailable||!std::isfinite(brain)||!std::isfinite(removed_brain)||brain<=0.||
       removed_brain<0.||removed_brain>brain*(1.+1e-6)||brain+skull>cranial_tissue*(1.+1e-6)||
       removed_brain+removed_skull>removed_cranial_tissue*(1.+1e-6))return out;
    const double brain_fraction=std::clamp(1.-removed_brain/brain,0.,1.);
    // A major upper-skull/brain defect can destabilize the original head
    // while the neck disk is intact. Require BOTH measured losses and the
    // surviving neck bridge above. These are explicit game tuning thresholds.
    if(cranial.bone_fraction<=.65&&brain_fraction<=.60){
        out.state=State::hanging;out.bone_fraction=cranial.bone_fraction;
        out.stiffness_scale=std::clamp((.05+.30*cranial.bone_fraction)*strength,.01,1.);
    }
    return out;
}
}
