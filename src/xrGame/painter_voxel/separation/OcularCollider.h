#pragma once
namespace gfy_live { namespace separation_detail {
// The accepted anatomy establishes ownership; the ACTUAL published source eye
// establishes contact bounds. Some fitted atlases sit 3cm below authored eyes.
inline void fitOcularSurfaceCollider(SeparationPiece&piece,const gfy_piece_collider::original_frames::Frames&frames){
 if(!piece.ocular_remnant)return;
 need(piece.elements.size()==1,"ocular_physics_single_original_element");auto&e=piece.elements.front();
 need(e.bone<frames.bones.size(),"ocular_physics_original_frame");
 Vec lo{1e30,1e30,1e30},hi{-1e30,-1e30,-1e30};std::size_t vertices=0;
 for(const auto&child:piece.mesh.children)for(const auto&row:child.vertices){const Vec p=transform(frames.bones[e.bone].rest_to_anchor,Vec{row.surface[0],row.surface[1],row.surface[2]});
  for(unsigned k=0;k<3;++k){lo[k]=std::min(lo[k],p[k]);hi[k]=std::max(hi[k],p[k]);}++vertices;}
 need(vertices>0&&std::isfinite(e.mass_kg)&&e.mass_kg>0.,"ocular_physics_source_geometry");
 Vec half=(hi-lo)*.5;for(unsigned k=0;k<3;++k){need(std::isfinite(half[k])&&half[k]<=.075,"ocular_physics_source_extent");half[k]=std::max(half[k],.005);}
 const Vec center=(hi+lo)*.5;e.retain_original_shape=false;e.com=point(center);e.boxes.clear();
 SeparationBox box;box.center=point(center);box.half_extents=point(half);box.orientation={1,0,0,0,1,0,0,0,1};e.boxes.push_back(box);
 e.inertia={};e.inertia[0]=e.mass_kg*(half.y*half.y+half.z*half.z)/3.;
 e.inertia[4]=e.mass_kg*(half.x*half.x+half.z*half.z)/3.;e.inertia[8]=e.mass_kg*(half.x*half.x+half.y*half.y)/3.;
}
} }
