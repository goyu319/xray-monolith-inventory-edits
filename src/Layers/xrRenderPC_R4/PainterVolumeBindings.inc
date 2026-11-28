static constexpr Bound wallSlots[]={
{"smp_base",0u,1u,D3D_SIT_SAMPLER},
{"s_painter_removal_pages",0u,1u,D3D_SIT_TEXTURE},
{"s_painter_removal_data",1u,1u,D3D_SIT_TEXTURE},
{"s_painter_slits",2u,1u,D3D_SIT_TEXTURE},
{"s_anatomy_pages",3u,1u,D3D_SIT_TEXTURE},
{"s_anatomy_data",4u,1u,D3D_SIT_TEXTURE},
{"s_painter_blood_cells",5u,1u,D3D_SIT_TEXTURE},
{"s_tissue_1",6u,1u,D3D_SIT_TEXTURE},
{"s_tissue_2",7u,1u,D3D_SIT_TEXTURE},
{"s_tissue_3",8u,1u,D3D_SIT_TEXTURE},
{"s_tissue_4",9u,1u,D3D_SIT_TEXTURE},
{"s_tissue_5",10u,1u,D3D_SIT_TEXTURE},
{"s_tissue_6",11u,1u,D3D_SIT_TEXTURE},
{"s_tissue_7",12u,1u,D3D_SIT_TEXTURE},
{"s_tissue_8",13u,1u,D3D_SIT_TEXTURE},
{"s_tissue_9",14u,1u,D3D_SIT_TEXTURE},
{"s_tissue_10",15u,1u,D3D_SIT_TEXTURE},
{"s_tissue_11",16u,1u,D3D_SIT_TEXTURE},
{"s_tissue_12",17u,1u,D3D_SIT_TEXTURE},
{"s_tissue_13",18u,1u,D3D_SIT_TEXTURE},
{"s_tissue_14",19u,1u,D3D_SIT_TEXTURE},
{"s_tissue_15",20u,1u,D3D_SIT_TEXTURE},
{"s_tissue_bone_bump",21u,1u,D3D_SIT_TEXTURE},
{"s_painter_burn_field",22u,1u,D3D_SIT_TEXTURE},
{"s_painter_burn_faces",23u,1u,D3D_SIT_TEXTURE},
{"s_painter_burn_walls",24u,1u,D3D_SIT_TEXTURE},
{"s_painter_burn_baked",25u,1u,D3D_SIT_TEXTURE},
{"dynamic_transforms",1u,1u,D3D_SIT_CBUFFER},
{"static_globals",2u,1u,D3D_SIT_CBUFFER},
{"PainterTissueState",0u,1u,D3D_SIT_CBUFFER}
};
static constexpr Bound pieceSlots[]={
{"smp_base",0u,1u,D3D_SIT_SAMPLER},
{"s_painter_removal_pages",0u,1u,D3D_SIT_TEXTURE},
{"s_painter_removal_data",1u,1u,D3D_SIT_TEXTURE},
{"s_painter_slits",2u,1u,D3D_SIT_TEXTURE},
{"s_anatomy_pages",3u,1u,D3D_SIT_TEXTURE},
{"s_anatomy_data",4u,1u,D3D_SIT_TEXTURE},
{"s_painter_blood_cells",5u,1u,D3D_SIT_TEXTURE},
{"s_tissue_1",6u,1u,D3D_SIT_TEXTURE},
{"s_tissue_2",7u,1u,D3D_SIT_TEXTURE},
{"s_tissue_3",8u,1u,D3D_SIT_TEXTURE},
{"s_tissue_4",9u,1u,D3D_SIT_TEXTURE},
{"s_tissue_5",10u,1u,D3D_SIT_TEXTURE},
{"s_tissue_6",11u,1u,D3D_SIT_TEXTURE},
{"s_tissue_7",12u,1u,D3D_SIT_TEXTURE},
{"s_tissue_8",13u,1u,D3D_SIT_TEXTURE},
{"s_tissue_9",14u,1u,D3D_SIT_TEXTURE},
{"s_tissue_10",15u,1u,D3D_SIT_TEXTURE},
{"s_tissue_11",16u,1u,D3D_SIT_TEXTURE},
{"s_tissue_12",17u,1u,D3D_SIT_TEXTURE},
{"s_tissue_13",18u,1u,D3D_SIT_TEXTURE},
{"s_tissue_14",19u,1u,D3D_SIT_TEXTURE},
{"s_tissue_15",20u,1u,D3D_SIT_TEXTURE},
{"s_tissue_bone_bump",21u,1u,D3D_SIT_TEXTURE},
{"s_painter_burn_field",22u,1u,D3D_SIT_TEXTURE},
{"s_painter_burn_faces",23u,1u,D3D_SIT_TEXTURE},
{"s_painter_burn_walls",24u,1u,D3D_SIT_TEXTURE},
{"s_painter_burn_baked",25u,1u,D3D_SIT_TEXTURE},
{"dynamic_transforms",1u,1u,D3D_SIT_CBUFFER},
{"static_globals",2u,1u,D3D_SIT_CBUFFER},
{"PainterTissueState",0u,1u,D3D_SIT_CBUFFER}
};
inline const char* volumeProfile(unsigned samples,const std::string& semantic) {
    if(samples==1 && semantic=="b82bbc160de6764040a3d371ff30744061c21d0541b42d81d410505e53145d79")return "0f20c0b9c4f48094b87ac02cb54dfd9a3cb47c5c165853bb112177d113e2e3a4";
    if(samples==1 && semantic=="9b8f67cb073419822de5a8b47a57233e32926761db5e43004a0095b7e18d25e0")return "6972f4d6e49becc1cbc634f2695657205a8cdca3d6825cf18cfc196dd35e02aa";
    if(samples==2 && semantic=="06fc6c7dec3e9604b49ceb8df2bd32a0319131b61a6f70dc2a926aac06fb3142")return "0f20c0b9c4f48094b87ac02cb54dfd9a3cb47c5c165853bb112177d113e2e3a4";
    if(samples==2 && semantic=="81939c2803fe4507a58707833dc7e32f7d70f89eb92d43f2fa75e1f00fc271a4")return "6972f4d6e49becc1cbc634f2695657205a8cdca3d6825cf18cfc196dd35e02aa";
    if(samples==4 && semantic=="864cab5495872a986f2f427dfd3ae0b3b8ac21cf06ca75666fe793c492ecd1cd")return "0f20c0b9c4f48094b87ac02cb54dfd9a3cb47c5c165853bb112177d113e2e3a4";
    if(samples==4 && semantic=="c52cfe0091193c81d28b4af58da77e756bf0ebb917affdaea8a677dd0357d03c")return "6972f4d6e49becc1cbc634f2695657205a8cdca3d6825cf18cfc196dd35e02aa";
    if(samples==1 && semantic=="639dd64fbed0a9d233759641a44d14403d597c2f74a76a281dca49c6f5582bd0")return "4abab796ef8fbb2f063bc29554a971b35684a5fc0f74a155198636e8988cd9c7"; // skinfast
    if(samples==2 && semantic=="d25266250ca3f3aa6d9ddfddf731f156d60ab0b4921707d1b39874675de86f67")return "4abab796ef8fbb2f063bc29554a971b35684a5fc0f74a155198636e8988cd9c7"; // skinfast
    if(samples==4 && semantic=="f132578cf053e74d29d85fc339d9527f45ece23648bd361614af963207a7fc6b")return "4abab796ef8fbb2f063bc29554a971b35684a5fc0f74a155198636e8988cd9c7"; // skinfast
    if(samples==1 && semantic=="7050ac5d3c12a17377812906bc1d4bae5e65c2db7b1b643c6d25d1152d01d68c")return "1a61960d46e51c314be6fd3324084ce4374466a4107e9f7430102751a668d9ab"; // skinfast
    if(samples==2 && semantic=="ad74188674fe0c236e5fb06014c659d0b979fa9c94460c3de26b736b5a85257d")return "1a61960d46e51c314be6fd3324084ce4374466a4107e9f7430102751a668d9ab"; // skinfast
    if(samples==4 && semantic=="f4c6ebfce8cddecc4461bc899e3c56cfcd23a2ace505f9ccc27e87ca7e49c7c9")return "1a61960d46e51c314be6fd3324084ce4374466a4107e9f7430102751a668d9ab"; // skinfast
    return nullptr;
}
