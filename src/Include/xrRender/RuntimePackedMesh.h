#pragma once
#include "SkinOwnershipField.h"
#include <array>
#include <set>
#include "RuntimeSignedSkin.h"
#include <cstdint>
#include <vector>
#include <memory>
#include "SparseTissueField.h"
#include "SparseDamageField.h"
namespace painter_detail { struct AnalyticChannelField; }
namespace gfy_packed {
// Child identities and native bone-face tables use u16. Some original GFY
// models already contain 293 render sections; reserve room for cavity sections
// while keeping per-owner allocation bounded and the 65535 face cap unchanged.
inline constexpr unsigned max_child_slots = 1024;
struct Vertex {
    std::array<float,14> surface{}; // P,N,T,B,UV in original model space.
    std::array<std::uint16_t,4> bone{};
    std::array<float,4> weight{};
};
static_assert(sizeof(Vertex)==80,"Runtime CPU vertex ABI");
// Native 4W form of a row's influences, the layout packing::vertex writes: duplicate bones merged, the largest influence
// in slot 3 (every slot starts as that bone), the others normalized into slots 0-2, and w3 rebuilt from them exactly as the
// renderer reconstructs it. Such a row always passes reconstructed_fourth_weight / gpu_quantized_fourth_weight (w0+w1+w2
// <= .75). A row with no positive weight is left alone.
inline void canonical_weights(Vertex& row){
    if(gfy_signed_hw::negative(row.weight)){const auto full=gfy_signed_hw::full(row.bone,row.weight);gfy_signed_hw::pack(full,row.bone,row.weight);return;}
    std::array<std::uint16_t,4> b{};std::array<double,4> w{};unsigned used=0;
    for(unsigned q=0;q<4;++q){if(!(row.weight[q]>0.f))continue;unsigned s=0;while(s<used&&b[s]!=row.bone[q])++s;
        if(s==used){b[used]=row.bone[q];w[used]=0;++used;}w[s]+=double(row.weight[q]);}
    if(!used)return;
    double sum=0;unsigned largest=0;for(unsigned q=0;q<used;++q){sum+=w[q];if(w[q]>w[largest])largest=q;}
    for(unsigned k=0;k<4;++k){row.bone[k]=b[largest];row.weight[k]=0.f;}
    unsigned next=0;for(unsigned q=0;q<used;++q)if(q!=largest){row.bone[next]=b[q];row.weight[next]=float(w[q]/sum);++next;}
    row.weight[3]=1.f-row.weight[0]-row.weight[1]-row.weight[2];
}
struct Child {
    std::uint16_t child_index{},material_source_child{};
    unsigned material{};
    bool skin_clip{}; // Requires an immutable removal field and masked material.
    bool skin_owned{}; // Requires its exact immutable triangle ownership table.
    std::vector<Vertex> vertices;
    std::vector<std::uint16_t> indices;
    std::vector<std::uint32_t> original_faces; // UINT32_MAX denotes new cavity.
};
// Mask-only wounds, slice 3: a decide-once tear as the GPU reads it (painter_tearfn.h): TearParams rows (51 float4, as 32-bit
// words), the section map (n x n float4: muscles' extra depth, flags 1 limb / 2 bone footprint, shell floor A, B) and the frame
// and slab the CPU side uses (pieces, coverage box).
struct TearBlock {
    std::vector<std::uint32_t> params;std::uint32_t n=0;std::vector<float> section;
    double centre[3]{},normal[3]{},u[3]{},w[3]{},footLo[2]{},footHi[2]{},reach=0,pitch=0;std::uint32_t seed=0;
    std::vector<std::uint8_t> bake;std::uint32_t bake_dim[3]{};double bake_origin[3]{},bake_pitch=0; // the baked tear: 3-D muscle field (TearDecide.h bakeBox)
    std::vector<float> map;std::uint32_t map_dim[2]{};double map_origin[2]{},map_pitch=0;std::vector<std::uint32_t> header; // its 2-D map and the programs' header
};
// Mask-only wounds, slice 4: one tear a mesh holds and its side (0 / 1; 2 a crack kept on both sides; 255 decided on this grid,
// not split yet: the separation splits it). A piece binds its tears with side <= 2 (painter_vmaskt_*, at most 8).
struct TearRef {std::shared_ptr<const TearBlock> block;std::uint8_t side=255;};
// slice 4: a piece's drawing set for ALL of its torn ends (TearAtlas.h): params rows (index frame, tear headers, 4 cm index),
// the tears' maps stacked (float4 texels) and their muscle fields stacked along z (R8)
struct TearAtlas {std::uint32_t K=0,paramsRows=0;std::vector<std::uint32_t> params;std::vector<float> map;std::uint32_t mapW=0,mapH=0;
    std::vector<std::uint8_t> field;std::uint32_t fieldDim[3]{};std::size_t bytes=0;
    // cutfix: packing by reference (TearAtlas.h): block k's map goes to (place[k][0], place[k][1]) of the map texture, its field to
    // (place[k][2], place[k][3], place[k][4]) of the field texture; map / field above stay empty then (the renderer uploads the blocks)
    std::vector<std::shared_ptr<const TearBlock>> blocks;std::vector<std::array<std::uint32_t,5>> place;};
// melee (2026-11-12, value 3): a blunt blow's dent, rest space: the exterior is pushed in along `push` (unit, the blow's travel) by
// up to `depth` within `radius` of `centre` (a smooth bowl), on the GPU (painter_dent.h, the painter_vmaskd_* vertex programs); the
// anatomy volume is read from the pushed surface (the exterior's rest position is the pushed one).
struct Dent {float centre[3]{},radius{},push[3]{},depth{};};
// melee (round 2): a blade slab's box (rest space; centre, unit side along the cut, unit normal of the blade's plane, half extents
// with a 4-8 mm margin; the depth axis is normal x side): near a slit the value-3 exterior is cut by the removal field's smooth
// surface (painter_slit.h), so a slash's edge is smooth instead of 2 mm cell steps.
struct Slit {float centre[3]{},half_side{},side[3]{},half_normal{},normal[3]{},half_depth{};};
struct Mesh {
    std::shared_ptr<const painter_detail::SkinOwnershipField> skin_ownership;
    std::shared_ptr<const painter_detail::AnalyticChannelField> analytic_channels;
    bool analytic_coverage_admitted=false; // Producer proof, never inferred from supported history.
    std::shared_ptr<const painter_detail::TissueField> tissue_field;
    std::shared_ptr<const painter_detail::DamageField> removal_field;
    // Mask-only wounds (painter_voxel_mask_only): no wound geometry; the skin_clip exterior steps into the anatomy volume.
    bool mask_volume=false;
    // Mask-only wounds, slice 2: a detached piece (original exterior + coverage box; its removal field marks every cell that is
    // not this piece's). The renderer uses painter_vmaskp_* programs (the box sentinel).
    bool mask_piece=false;
    // Mask-only wounds, slice 3: the decide-once tear this mesh borders and its side (0 = A, 1 = B; 255 none). Pieces with a tear
    // use painter_vmaskt_* programs.
    std::vector<TearRef> tears; // slice 4: every decide-once tear this mesh holds
    std::shared_ptr<const TearAtlas> tear_atlas; // slice 4: a piece's drawing set of its tears with a side (the renderer binds it)
    std::vector<Dent> dents; // melee (value 3): the dents of the body this mesh is (or came from), oldest first
    std::vector<Slit> slits; // melee (value 3): its blade slits (the smooth slash edge), oldest first
    std::vector<Child> children;
    std::size_t reduced_skin_vertices{};
    double max_discarded_skin_weight{};
};
inline void validate_skin_ownership(const Mesh&mesh){
 if(mesh.skin_ownership)mesh.skin_ownership->validate();
 for(const auto&child:mesh.children){const auto*entry=mesh.skin_ownership?mesh.skin_ownership->child(child.child_index):nullptr;
  if(child.skin_owned!=bool(entry))throw std::invalid_argument("skin_owner_required_table");
  if(entry&&(child.material||child.indices.size()!=std::size_t(entry->count)*3))throw std::invalid_argument("skin_owner_mesh_manifest");}
 if(mesh.skin_ownership)for(const auto&entry:mesh.skin_ownership->children){unsigned found=0;
  for(const auto&child:mesh.children)found+=child.child_index==entry.child;
  if(found!=1)throw std::invalid_argument("skin_owner_duplicate_or_missing_mesh_child");}
}
// Producers call this only after rebuilding all primitive rows from validated inputs.
inline void set_skin_ownership(Mesh&mesh,std::shared_ptr<const painter_detail::SkinOwnershipField>field){
 mesh.skin_ownership=std::move(field);
 for(auto&child:mesh.children)child.skin_owned=mesh.skin_ownership&&mesh.skin_ownership->child(child.child_index);
 validate_skin_ownership(mesh);
}
// Sparse immutable union of region ranges for CPU ray contacts. An original
// source face may occur in more than one retained primitive: these are a union,
// while each primitive's planes remain an intersection.
struct SkinOwnershipQuery {
 std::shared_ptr<const painter_detail::SkinOwnershipField> field;
 std::map<std::uint64_t,std::vector<std::array<std::uint32_t,2>>> rows;
 bool accepts(std::uint64_t key,const std::array<float,3>&p)const noexcept {
  const auto it=rows.find(key);if(it==rows.end())return true;
  for(const auto&r:it->second){bool keep=true;
   for(std::uint32_t n=0;n<r[1];++n){const auto&q=field->planes[r[0]+n];
    const float d=((q[0]*p[0]+q[1]*p[1])+q[2]*p[2])+q[3];if(!(d>=0)){keep=false;break;}}
   if(keep)return true;}
  return false;
 }
};
inline std::shared_ptr<const SkinOwnershipQuery> skin_ownership_query(const Mesh&mesh,bool originalFaces){
 validate_skin_ownership(mesh);if(!mesh.skin_ownership)return {};
 auto result=std::make_shared<SkinOwnershipQuery>();result->field=mesh.skin_ownership;
 std::uint64_t flat=0;
 for(const auto&child:mesh.children){const auto*table=mesh.skin_ownership->child(child.child_index);
  for(std::size_t t=0;t<child.indices.size()/3;++t,++flat){
   if(originalFaces&&(child.material||child.original_faces.size()!=child.indices.size()/3||child.original_faces[t]==~std::uint32_t(0)))continue;
   const auto key=originalFaces?(std::uint64_t(child.material_source_child)<<32)|child.original_faces[t]:flat;
   if(!table||!table->data[t][1])continue;
   result->rows[key].push_back(table->data[t]);
  }}
 // Do not allocate a map entry for every ordinary body face. Only a duplicate
 // unrestricted primitive of an already constrained original face matters.
 if(originalFaces)for(const auto&child:mesh.children){if(child.material)continue;
  const auto*table=mesh.skin_ownership->child(child.child_index);
  if(child.original_faces.size()!=child.indices.size()/3)continue;
  for(std::size_t t=0;t<child.original_faces.size();++t)if(!table||!table->data[t][1])
   result->rows.erase((std::uint64_t(child.material_source_child)<<32)|child.original_faces[t]);}
 return result->rows.empty()?std::shared_ptr<const SkinOwnershipQuery>{}:result;
}
} // namespace gfy_packed
