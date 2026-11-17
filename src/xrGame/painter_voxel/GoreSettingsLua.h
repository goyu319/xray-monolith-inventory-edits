#pragma once
// Strict raw-table reader. Validation completes before the one atomic native commit.
#include "GoreSettings.h"
extern "C" {
#include <lua.h>
}
#include <cstring>
#include <cmath>

namespace painter_deformation {
inline unsigned api_version(){return 1;}
inline bool settings_table(lua_State* L,int index){
    if(!L||lua_type(L,index)!=LUA_TTABLE)return false;
    if(index<0)index=lua_gettop(L)+index+1;
    struct Stack {lua_State* L;int top;~Stack(){lua_settop(L,top);}} stack{L,lua_gettop(L)};
    gfy_live::GoreControls next;unsigned seen=0;
    lua_pushnil(L);
    while(lua_next(L,index)){
        if(lua_type(L,-2)!=LUA_TSTRING)return false;
        size_t length=0;const char* key=lua_tolstring(L,-2,&length);
        int field=-1;
        static const char* const names[]={"version","surface_burst_frequency_mult","surface_burst_intensity_mult","head_blowout_frequency_mult","head_blowout_intensity_mult","lateral_vent_frequency_mult","lateral_vent_intensity_mult","cavitation_mult","exit_tearing_mult","wound_variation_mult","head_remnants_enabled","head_tether_strength_mult","eye_remnants_mode","intestines_enabled","intestine_amount_mult","intestine_frequency_mult","intestine_tether_strength_mult"};
        for(unsigned i=0;i<sizeof(names)/sizeof(names[0]);++i)
            if(std::strlen(names[i])==length&&std::memcmp(key,names[i],length)==0){field=int(i);break;}
        if(field<0||(seen&(1u<<field)))return false;
        seen|=1u<<field;
        switch(field){
        case 0:if(lua_type(L,-1)!=LUA_TNUMBER||lua_tonumber(L,-1)!=1.)return false;break;
        case 1:if(lua_type(L,-1)!=LUA_TNUMBER)return false;next.surface_burst_frequency_mult=lua_tonumber(L,-1);break;
        case 2:if(lua_type(L,-1)!=LUA_TNUMBER)return false;next.surface_burst_intensity_mult=lua_tonumber(L,-1);break;
        case 3:if(lua_type(L,-1)!=LUA_TNUMBER)return false;next.head_blowout_frequency_mult=lua_tonumber(L,-1);break;
        case 4:if(lua_type(L,-1)!=LUA_TNUMBER)return false;next.head_blowout_intensity_mult=lua_tonumber(L,-1);break;
        case 5:if(lua_type(L,-1)!=LUA_TNUMBER)return false;next.lateral_vent_frequency_mult=lua_tonumber(L,-1);break;
        case 6:if(lua_type(L,-1)!=LUA_TNUMBER)return false;next.lateral_vent_intensity_mult=lua_tonumber(L,-1);break;
        case 7:if(lua_type(L,-1)!=LUA_TNUMBER)return false;next.cavitation_mult=lua_tonumber(L,-1);break;
        case 8:if(lua_type(L,-1)!=LUA_TNUMBER)return false;next.exit_tearing_mult=lua_tonumber(L,-1);break;
        case 9:if(lua_type(L,-1)!=LUA_TNUMBER)return false;next.wound_variation_mult=lua_tonumber(L,-1);break;
        case 10:if(lua_type(L,-1)!=LUA_TBOOLEAN)return false;next.head_remnants_enabled=lua_toboolean(L,-1)!=0;break;
        case 11:if(lua_type(L,-1)!=LUA_TNUMBER)return false;next.head_tether_strength_mult=lua_tonumber(L,-1);break;
        case 12:{if(lua_type(L,-1)!=LUA_TNUMBER)return false;const double value=lua_tonumber(L,-1);if(!std::isfinite(value)||value<0||value>1||std::floor(value)!=value)return false;next.eye_remnants_mode=int(value);break;}
        case 13:if(lua_type(L,-1)!=LUA_TBOOLEAN)return false;next.intestines_enabled=lua_toboolean(L,-1)!=0;break;
        case 14:if(lua_type(L,-1)!=LUA_TNUMBER)return false;next.intestine_amount_mult=lua_tonumber(L,-1);break;
        case 15:if(lua_type(L,-1)!=LUA_TNUMBER)return false;next.intestine_frequency_mult=lua_tonumber(L,-1);break;
        case 16:if(lua_type(L,-1)!=LUA_TNUMBER)return false;next.intestine_tether_strength_mult=lua_tonumber(L,-1);break;
        default:return false;
        }
        lua_pop(L,1);
    }
    if(seen!=((1u<<17)-1u))return false;
    return commit(next);
}
}
