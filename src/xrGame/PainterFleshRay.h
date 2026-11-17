#pragma once
// bodypass (2026-11-16): whether a bullet that struck bone shape `bone` at `range` along the ray (start, unit dir) meets flesh of
// `fitted` (the fitted boxes a part's physics shell is built from: bone frame, its owned anatomy) inside the struck shape. Reads the
// bone transforms and the object's XFORM the way the collision form does (CCF_Skeleton::BuildState), nothing else; no allocation.
// margin: the boxes and kept native shapes grow by it (3 cm: the collider fitter omits no cell farther than 2.5 cm from its boxes).
// 1: flesh there (or undecided), 2: none.
#include "../Include/xrRender/Kinematics.h"
#include "../xrPhysics/FittedShell.h"
#include <vector>
inline int painter_flesh_on_ray(IKinematics& K, const Fmatrix& L2W, const std::vector<PHFittedBone>& fitted, const Fvector& start,
    const Fvector& dir, float range, u16 bone, float margin)
{
    if(bone>=K.LL_BoneCount()||fitted.empty())return 1;
    auto sphereOf=[&](u16 b,Fvector& centre,float& radius){
        const SBoneShape& shape=K.LL_GetData(b).shape;Fvector local;
        switch(shape.type){
        case SBoneShape::stBox: local.set(shape.box.m_translate);radius=shape.box.m_halfsize.magnitude();break;
        case SBoneShape::stSphere: local.set(shape.sphere.P);radius=shape.sphere.R;break;
        case SBoneShape::stCylinder: local.set(shape.cylinder.m_center);
            radius=_sqrt(shape.cylinder.m_height*shape.cylinder.m_height*.25f+shape.cylinder.m_radius*shape.cylinder.m_radius);break;
        default: return false;}
        K.LL_GetTransform(b).transform_tiny(centre,local);L2W.transform_tiny(centre);return true;};
    auto raySphere=[&](const Fvector& centre,float radius,float& t0,float& t1){
        Fvector oc;oc.sub(centre,start);const float along=oc.dotproduct(dir),off2=oc.square_magnitude()-along*along;
        if(off2>radius*radius)return false;const float half=_sqrt(radius*radius-off2);t0=along-half;t1=along+half;return true;};
    Fvector struckCentre;float struckRadius,s0,s1;
    if(!sphereOf(bone,struckCentre,struckRadius))return 1;
    const float lo=range-margin,hi=(raySphere(struckCentre,struckRadius,s0,s1)?_max(range,s1):range)+margin;
    for(const auto& element:fitted){
        if(element.bone>=K.LL_BoneCount())return 1;
        if(element.retain_original_shape){ // an unchanged native element: its whole shape (bounding sphere) is flesh
            Fvector c;float r,t0,t1;
            if(!sphereOf(element.bone,c,r))return 1;
            if(raySphere(c,r+margin,t0,t1)&&t1>=lo&&t0<=hi)return 1;
            continue;}
        Fmatrix boneW;boneW.mul_43(L2W,K.LL_GetTransform(element.bone));
        for(const auto& box:element.boxes){
            Fmatrix ME,TW,IM;box.xform_get(ME);TW.mul_43(boneW,ME);if(!IM.invert_b(TW))return 1;
            Fvector p,d;IM.transform_tiny(p,start);IM.transform_dir(d,dir);
            float t0=-flt_max,t1=flt_max;bool meets=true;
            for(int a=0;a<3&&meets;++a){
                const float h=(&box.m_halfsize.x)[a]+margin,pa=(&p.x)[a],da=(&d.x)[a];
                if(_abs(da)<1e-9f){if(pa<-h||pa>h)meets=false;continue;}
                float ta=(-h-pa)/da,tb=(h-pa)/da;if(ta>tb){const float s=ta;ta=tb;tb=s;}
                t0=_max(t0,ta);t1=_min(t1,tb);if(t0>t1)meets=false;}
            if(meets&&t1>=lo&&t0<=hi)return 1;}
    }
    return 2;
}
