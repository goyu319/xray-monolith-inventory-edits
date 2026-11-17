#pragma once
#include <cstdint>
namespace gfy_live { namespace eye_remnants {
// Recognize actual surviving ocular anatomy, never textures, names, nearest
// bones, or every accessory. A mixed head fragment is not an isolated eye.
inline bool ocular(std::uint64_t cells,std::uint64_t eyes,std::uint64_t bone) noexcept {
    return cells&&eyes<=cells&&bone<=cells&&bone==0&&eyes>=cells/2+cells%2;
}
inline bool retain(bool newly_detached,std::uint64_t cells,std::uint64_t eyes,
    std::uint64_t bone,int mode) noexcept {
    return newly_detached&&mode==1&&ocular(cells,eyes,bone);
}
}}
