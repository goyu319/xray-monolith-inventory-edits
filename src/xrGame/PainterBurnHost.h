#pragma once
#include "painter_voxel/LiveDamageWorker.h"
class CObject;
namespace painter_burn_host {
void bind(CObject&,const gfy_live::Identity&,const std::shared_ptr<gfy_live::Session>&,bool piece,bool exposure=false) noexcept;
bool active() noexcept;
void refresh(CObject&,const std::shared_ptr<gfy_live::Session>&) noexcept;
void forget(CObject&,const std::shared_ptr<gfy_live::Session>&) noexcept;
void split(CObject&,const std::vector<CObject*>&) noexcept;
void releaseWorld(u64) noexcept;
void frame() noexcept;
void disableMaster() noexcept;
void quit() noexcept;
}
