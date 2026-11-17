#pragma once
#include <cstddef>
#include <cstring>
#include <luabind/object.hpp>

namespace painter_wound_event {
// Diagnostics only: no converters, new strings, __index calls or unbounded copy.
// Raw iteration avoids allocating/interning a lookup-key string in the hit path.
inline void read_skip_reason(const ::luabind::object& source,char (&why)[161])
{
    std::memcpy(why,"unreported",11);
    lua_State* L=source.lua_state();
    if(!L||!lua_checkstack(L,3))return;
    const int top=lua_gettop(L);
    source.pushvalue();
    if(lua_type(L,-1)==LUA_TTABLE){
        lua_pushnil(L);
        unsigned visited=0;
        while(visited++<128&&lua_next(L,-2)!=0){
            std::size_t keyLength=0;
            const char* key=lua_type(L,-2)==LUA_TSTRING?lua_tolstring(L,-2,&keyLength):nullptr;
            static const char name[]="painter_wound_skip_reason";
            if(key&&keyLength==sizeof(name)-1&&std::memcmp(key,name,sizeof(name)-1)==0){
                if(lua_type(L,-1)==LUA_TSTRING){
                    std::size_t length=0;const char* value=lua_tolstring(L,-1,&length);
                    const std::size_t count=length<160?length:160;
                    for(std::size_t n=0;n<count;++n){const unsigned char c=static_cast<unsigned char>(value[n]);
                        why[n]=(c<32||c==127)?' ':static_cast<char>(c);}
                    why[count]=0;
                }
                break;
            }
            lua_pop(L,1);
        }
    }
    lua_settop(L,top);
}
}
