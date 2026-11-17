#pragma once
#include "painter_voxel/GoreSettingsLua.h"

namespace painter_deformation {
inline bool settings(const ::luabind::object& table){
    if(!table||table.type()!=LUA_TTABLE)return false;
    lua_State* L=table.lua_state();if(!L)return false;
    const int top=lua_gettop(L);table.pushvalue();
    const bool accepted=settings_table(L,-1);lua_settop(L,top);return accepted;
}
}

