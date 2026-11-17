#pragma once
#include "PainterSkinProxy.h"
#include "../Include/xrRender/OriginalSkinProxyProof.h"
namespace painter_skin_proxy {
// Called on the genuine shell before Activate/Build. Prepared source and proof
// are immutable CPU data; no world contacts, filtering, mass or sleep edits.
inline bool append_fitted(CPhysicsShell& shell,IKinematics& k,const std::vector<PHFittedBone>& fitted,
 const gfy_packed::Mesh& mesh,ProofStats* proof_stats=nullptr,AppendStats* append_stats=nullptr) noexcept {
 try{auto source=k.LL_OriginalSkinProxySource();if(!source||source->rig.size()!=k.LL_BoneCount())return false;
  std::vector<u16> retained,owner_ids(source->rig.size(),u16(-1));BONE_P_MAP owners;
  for(const auto& b:fitted)if(b.retain_original_shape)retained.push_back(b.bone);
  for(u16 b=0;b<source->rig.size();++b){physicsBone physical{};const u16 owner=source->rig[b].owner;if(owner!=u16(-1))physical.element=shell.get_Element(owner);if(physical.element)owner_ids[b]=physical.element->m_SelfID;owners.emplace(b,physical);}
  auto groups=prove_groups(*source,mesh,retained,owner_ids,proof_stats);if(groups.empty())return false;
  return append(shell,k,owners,&fitted,&groups,append_stats);
 }catch(...){return false;}
}
}
