#pragma once
#include "RuntimePackedMesh.h"
// CPU-only immutable request. Geometry arrays are in original model space.
// Caller pins exact GFY source bytes and supplies its live spawn/visual cookie.
struct RuntimeSkinVertex
{
    Fvector position, normal, tangent, binormal;
    Fvector2 uv;
    u16 bone[4];
    float weight[4];
};
struct RuntimeSkinBone
{
    shared_str name;
    u16 parent;
    Fmatrix model_to_bone;
};
struct RuntimeSkinChild
{
    std::shared_ptr<const painter_detail::TissueField> tissue_field;
    u16 child_index; // Stable source slot; contiguous appended cavity slots.
    u16 material_source_child;
    bool receives_wallmarks = true; // Exterior/gear only; new internal cavity is excluded.
    bool remove_surface = false; // Explicit empty original slot; never hides a rig bone.
    shared_str shader, texture; // Empty means use original material.
    xr_vector<RuntimeSkinVertex> vertices;
    xr_vector<u16> indices;
};
struct RuntimeSkinUpdate
{
    u64 owner_cookie, expected_geometry_generation, damage_revision;
    bool separation_packet = false;
    bool flight_preview = false, flight_detail = false;
    xr_vector<RuntimeSkinBone> exact_rig;
    xr_vector<RuntimeSkinChild> replaced_children;
};
