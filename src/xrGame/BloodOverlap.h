#pragma once

struct lua_State;

// CPU geometry only. Lua owns ordering, ledgers, callbacks, RNG and work limits.
void RegisterBloodOverlap(lua_State* state);
