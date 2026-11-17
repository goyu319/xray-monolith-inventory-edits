#pragma once
#ifndef GFY_SEP_SURFACE_OPTIONS_INCLUDED
#define GFY_SEP_SURFACE_OPTIONS_INCLUDED
// Frozen interface hook of the separation surface stage (separation-track plan P1.0a).
// bound_surface::prepare(...,const SurfaceOptions& = {}) takes these options. Every
// field defaults to "absent"; with the defaults the surface stage is byte-identical
// to int14t. Consumers: P2 (surface stage), P1 (pipeline), P3 (threads/cancel).
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>
#include "ThreadBudget.h"
namespace gfy_packed {struct Mesh;}
namespace gfy_live { namespace sep {
// (child, child-local face) -> global source face. Sorted flat replacement of the
// std::map<std::pair<uint16_t,uint32_t>,uint32_t> formerly built per separation in
// LiveSeparationWorker_impl.h. Keys are ordered exactly like that map's pairs, and
// find() returns the same global face (or null where the map returned end()).
// Lookup semantics are unchanged: callers still look up Child::child_index, so the
// latent overflow-chunk defect of sourceSkinAnchor is preserved under the exact gate.
class SourceFaceIndex {
 std::vector<std::uint64_t> keys_; // (child<<32)|face, ascending
 std::vector<std::uint32_t> faces_; // global face of keys_[i]
 static std::uint64_t key(std::uint16_t child,std::uint32_t face)noexcept{return (std::uint64_t(child)<<32)|face;}
public:
 SourceFaceIndex()=default;
 // children[f],faces[f] name global face f for f<count. Throws
 // "separation_duplicate_source_face_identity" when two global faces share a key:
 // same point of prepare() and same message as the former std::map emplace check.
 SourceFaceIndex(const std::vector<std::uint16_t>&children,const std::vector<std::uint32_t>&faces,std::size_t count){
  if(children.size()<count||faces.size()<count||count>std::size_t(UINT32_MAX))throw std::runtime_error("separation_source_face_index_size");
  std::vector<std::pair<std::uint64_t,std::uint32_t>>entries(count);
  for(std::size_t f=0;f<count;++f)entries[f]={key(children[f],faces[f]),std::uint32_t(f)};
  std::sort(entries.begin(),entries.end(),[](const auto&a,const auto&b){return a.first<b.first;});
  keys_.resize(count);faces_.resize(count);
  for(std::size_t i=0;i<count;++i){
   if(i&&entries[i].first==entries[i-1].first)throw std::runtime_error("separation_duplicate_source_face_identity");
   keys_[i]=entries[i].first;faces_[i]=entries[i].second;}
 }
 const std::uint32_t* find(std::uint16_t child,std::uint32_t face)const noexcept{
  const auto k=key(child,face);const auto i=std::lower_bound(keys_.begin(),keys_.end(),k);
  return i!=keys_.end()&&*i==k?&faces_[std::size_t(i-keys_.begin())]:nullptr;
 }
 std::size_t size()const noexcept{return keys_.size();}
 std::size_t bytes()const noexcept{return keys_.capacity()*sizeof(std::uint64_t)+faces_.capacity()*sizeof(std::uint32_t);}
};
struct SurfaceOptions {
 // Pieces whose surfaces are wanted (owned continuation: descendants of the owner).
 // Null = all. The union path still partitions the complete virtual parent.
 const std::set<std::uint32_t>* wanted{};
 // Already-published wound mesh of the snapshot (P2 S4, SEMANTIC, default off:
 // set only when SeparationInput::published_surface is true and geometry exists).
 // Null selects the exact union path.
 const gfy_packed::Mesh* published{};
 // Source face identity index shared with the caller (read-only).
 const SourceFaceIndex* source_faces{};
 // Optional per-source-face certified-keep flags (P2 S3b), 1 byte per face.
 const std::vector<std::uint8_t>* pristine_keep{};
 // Progress checkpoint (forwards to the job's SeparationCheckpoint, which throws
 // "separation_canceled" when the job was canceled).
 std::function<void(const char*)> checkpoint;
 // The job's cancel flag (normally sep::cancelFlag() of the calling thread).
 const std::atomic<bool>* cancel{};
 // Thread cap for every parallel section of the stage (sep::parallel clamps it to 4).
 unsigned max_threads{max_job_threads};
};
}
namespace bound_surface {using sep::SurfaceOptions;using sep::SourceFaceIndex;}
}
#endif
