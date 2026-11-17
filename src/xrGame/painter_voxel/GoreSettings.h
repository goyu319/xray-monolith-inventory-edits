#pragma once
#include "GoreControls.h"
#include <mutex>

namespace painter_deformation {
struct Store {std::mutex mutex;gfy_live::GoreControls value;};
// Inline function-local storage has one program-wide identity across owner TUs.
inline Store& store(){static Store value;return value;}
inline gfy_live::GoreControls snapshot(){
    auto& state=store();std::lock_guard<std::mutex> guard(state.mutex);return state.value;
}
inline bool commit(const gfy_live::GoreControls& value){
    if(!gfy_live::valid_gore_controls(value))return false;
    auto& state=store();std::lock_guard<std::mutex> guard(state.mutex);state.value=value;return true;
}
}

