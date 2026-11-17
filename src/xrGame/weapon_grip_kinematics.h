#pragma once

#include "../Include/xrRender/Kinematics.h"
#include "../xrEngine/bone.h"

// Hidden ancestors can make a still-visible grip's transform singular.
// Both rendering and death-time weapon collision must apply the same rule.
inline bool painter_weapon_grip_chain_visible(IKinematics& kinematics, int bone)
{
    const u16 count = kinematics.LL_BoneCount();
    if (bone < 0 || bone >= count)
        return false;
    u16 current = u16(bone);
    for (u16 visited = 0; visited < count; ++visited)
    {
        if (!kinematics.LL_GetBoneVisible(current))
            return false;
        if (current == kinematics.LL_GetBoneRoot())
            return true;
        const u16 parent = kinematics.LL_GetData(current).GetParentID();
        if (parent == BI_NONE || parent >= count)
            return false;
        current = parent;
    }
    return false;
}
