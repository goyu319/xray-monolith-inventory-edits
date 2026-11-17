#pragma once
#include "OriginalSkinProxySource.h"
#include "CompactSkinProxy.h"
#include "SourceTightBoxProxy.h"
#include "Kinematics.h"
#include "../../xrEngine/bone.h"
#include <chrono>
namespace painter_skin_proxy {
// Invoked once while loading the immutable original model, before publication
// or death-shell creation. Native ownership is checked again before append.
inline void precompute(Source& source,IKinematics& k) noexcept {
 const auto start=std::chrono::steady_clock::now();
 try{
  static_assert(sizeof(SBoneShape)==112,"original shape layout");
  if(source.vertices.empty()||source.vertices.size()>16384||!k.LL_BoneCount()||k.LL_BoneCount()>64)return;
  const u16 head=k.LL_BoneID("bip01_head");if(head>=k.LL_BoneCount())return;
  std::vector<RigBone> rig(k.LL_BoneCount());
  for(u16 b=0;b<k.LL_BoneCount();++b){const auto& bd=k.LL_GetData(b);auto& r=rig[b];std::memcpy(r.model_to_bone.data(),&bd.m2b_transform,64);std::memcpy(r.shape.data(),&bd.shape,112);r.joint_type=bd.IK_data.type;r.parent=bd.GetParentID();}
  auto physical_shape=[&](u16 b){const auto& s=k.LL_GetData(b).shape;return s.type>SBoneShape::stNone&&s.type<=SBoneShape::stCylinder&&!s.flags.test(SBoneShape::sfNoPhysics);};
  // AddElementRecursive creates a first physical root even for jtRigid;
  // later shaped rigid bones join their owning ancestor. Unshaped animation
  // controls above that root do not become source coverage owners. The native
  // append seam still checks this prediction against every actual owner.
  for(u16 b=0;b<k.LL_BoneCount();++b){u16 cursor=b;for(unsigned depth=0;depth<k.LL_BoneCount()&&cursor<k.LL_BoneCount();++depth){const auto& bd=k.LL_GetData(cursor);if(physical_shape(cursor)){bool owning_ancestor=false;u16 parent=bd.GetParentID();for(unsigned up=0;up<k.LL_BoneCount()&&parent<k.LL_BoneCount();++up){if(physical_shape(parent)){owning_ancestor=true;break;}parent=k.LL_GetData(parent).GetParentID();}if(bd.IK_data.type!=jtRigid||!owning_ancestor){rig[b].owner=cursor;break;}}cursor=bd.GetParentID();}}
  std::map<u16,std::vector<compact_skin_proxy::Vec>> groups;std::map<u16,std::uint8_t> kinds;std::map<u16,bool> mixed_kinds;std::map<u16,std::vector<std::uint32_t>> group_ids;
  for(std::uint32_t vi=0;vi<source.vertices.size();++vi){const auto& v=source.vertices[vi];u16 owner=65535;bool valid=true,positive=false;for(unsigned q=0;q<4;++q){float w=v.weights[q];if(!std::isfinite(w)||w<0||w>1){valid=false;break;}if(w==0)continue;positive=true;if(v.bones[q]>=rig.size()||rig[v.bones[q]].owner==65535){valid=false;break;}u16 e=rig[v.bones[q]].owner;if(owner!=65535&&owner!=e){valid=false;break;}owner=e;}for(float p:v.position)if(!std::isfinite(p))valid=false;if(valid&&positive&&((v.surface_kind==skin&&owner==head)||(v.surface_kind!=skin&&owner!=head))){if(kinds[owner]&&kinds[owner]!=v.surface_kind)mixed_kinds[owner]=true;else kinds[owner]=v.surface_kind;groups[owner].push_back({v.position[0],v.position[1],v.position[2]});group_ids[owner].push_back(vi);}}
  std::vector<PreparedGroup> prepared;
  for(const auto& group:groups){
   if(mixed_kinds[group.first]||(group.first==head&&rig[head].owner!=head))continue;
   // Only existing under-covered exposed source groups need extra shapes.
   double maximum=-1e9;std::vector<unsigned char> required;required.reserve(group.second.size());
   for(const auto& p:group.second){double nearest=1e9;Fvector model;model.set(float(p.x),float(p.y),float(p.z));for(u16 b=0;b<rig.size();++b){if(rig[b].owner!=group.first)continue;const auto& bd=k.LL_GetData(b);const auto& s=bd.shape;if(s.flags.test(SBoneShape::sfNoPhysics)||s.type==SBoneShape::stNone)continue;Fvector q;bd.m2b_transform.transform_tiny(q,model);
     if(s.type==SBoneShape::stSphere){double x=q.x-s.sphere.P.x,y=q.y-s.sphere.P.y,z=q.z-s.sphere.P.z;nearest=std::min(nearest,std::sqrt(x*x+y*y+z*z)-s.sphere.R);}
     else if(s.type==SBoneShape::stCylinder){Fvector d;d.sub(q,s.cylinder.m_center);double along=d.dotproduct(s.cylinder.m_direction),r=std::sqrt(std::max(0.,double(d.square_magnitude())-along*along))-s.cylinder.m_radius,h=std::abs(along)-.5*s.cylinder.m_height;nearest=std::min(nearest,std::sqrt(std::max(0.,r)*std::max(0.,r)+std::max(0.,h)*std::max(0.,h))+std::min(std::max(r,h),0.));}
     else if(s.type==SBoneShape::stBox){Fmatrix x;s.box.xform_get(x);Fmatrix inverse;inverse.invert(x);Fvector local;inverse.transform_tiny(local,q);double dx=std::abs(local.x)-s.box.m_halfsize.x,dy=std::abs(local.y)-s.box.m_halfsize.y,dz=std::abs(local.z)-s.box.m_halfsize.z;nearest=std::min(nearest,std::sqrt(std::max(0.,dx)*std::max(0.,dx)+std::max(0.,dy)*std::max(0.,dy)+std::max(0.,dz)*std::max(0.,dz))+std::min(std::max({dx,dy,dz}),0.));}
    }maximum=std::max(maximum,nearest);required.push_back(group.first==head||nearest>-.0021);}
   if(maximum<=.005)continue;PreparedGroup out;out.anchor=group.first;out.source_points=unsigned(group.second.size());out.surface_kind=kinds[group.first];for(auto q:required)out.required_points+=q?1:0;
   if(group.first==head){auto fit=compact_skin_proxy::fit(group.second);if(!fit.complete||fit.balls.size()>40)continue;out.complete=true;for(const auto& ball:fit.balls)out.spheres.push_back({{ball.center.x,ball.center.y,ball.center.z},ball.radius});}
   else {const bool partial=kinds[group.first]==solid_vest;const unsigned limit=partial?19:40;auto fit=source_tight_box_proxy::fit(group.second,required,limit,.0101,.0021,partial);if(!fit.certified||(!partial&&!fit.complete)||fit.boxes.size()>limit)continue;out.complete=fit.complete;out.uncovered_points=fit.uncovered;for(const auto& box:fit.boxes)out.boxes.push_back({{box.center.x,box.center.y,box.center.z},{box.half.x,box.half.y,box.half.z},{box.axes[0].x,box.axes[0].y,box.axes[0].z,box.axes[1].x,box.axes[1].y,box.axes[1].z,box.axes[2].x,box.axes[2].y,box.axes[2].z}});}
   out.source_vertices=group_ids[group.first];std::vector<unsigned char> member(source.vertices.size());for(auto i:out.source_vertices)member[i]=1;for(std::uint32_t i=0;i<source.triangles.size();++i){const auto& t=source.triangles[i];if(t.vertices[0]<member.size()&&t.vertices[1]<member.size()&&t.vertices[2]<member.size()&&(member[t.vertices[0]]||member[t.vertices[1]]||member[t.vertices[2]]))out.source_triangles.push_back(i);}
   prepared.push_back(std::move(out));
  }
  std::stable_sort(prepared.begin(),prepared.end(),[head](const PreparedGroup&a,const PreparedGroup&b){return (a.anchor==head)>(b.anchor==head);});
  source.rig=std::move(rig);source.prepared=std::move(prepared);source.prepare_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
 }catch(...){source.rig.clear();source.prepared.clear();}
}
}
