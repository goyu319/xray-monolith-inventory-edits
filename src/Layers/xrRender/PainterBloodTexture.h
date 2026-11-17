#pragma once
#ifdef USE_DX11
#include "PainterBloodTextureWorker.h"
#include "ResourceManager.h"
#include <memory>
#include <mutex>
#include <cstdint>
#include <vector>
#include <stdexcept>
namespace PainterBloodTexture {
using Snapshot=PainterBloodTextureSource::Snapshot;
using SnapshotPtr=PainterBloodTextureSource::SnapshotPtr;
struct Cached {SnapshotPtr snapshot;PainterBloodTextureCodec::Image image;};
// The shader and engine reference handles are inspected on their owner thread.
// No texture loading, D3D access, reader, driver call or wait occurs here.
// A null result means the existing background engine load is still pending.
inline SnapshotPtr selected(const ref_shader& shader){
 if(!shader||!shader->E[0]||shader->E[0]->passes.size()!=1||!shader->E[0]->passes[0])throw std::runtime_error("blood_texture_pass_invalid");
 const auto&pass=shader->E[0]->passes[0];auto*c=pass->constants?pass->constants->get("s_base"):nullptr;
 if(!c||c->type!=RC_dx10texture||!pass->T)throw std::runtime_error("blood_texture_base_binding_missing");
 CTexture* chosen=nullptr;unsigned count=0;for(const auto&b:*pass->T)if(b.first==c->samp.index){chosen=b.second._get();++count;}
 if(count!=1||!chosen)throw std::runtime_error("blood_texture_base_binding_ambiguous");
 return PainterBloodTextureSource::capture(&chosen->painter_blood_snapshot);
}
// Worker-only cache. Its context cannot be reached from the render publisher.
struct WorkerCache {
 std::unique_ptr<PainterBloodTextureWorker::Decoder> decoder;
 std::vector<std::shared_ptr<const Cached>> entries;
};
// quitfix (2026-11-13): the cache lives on the heap and is released by clear() on the owner thread, after the worker has
// been joined. As a thread_local OBJECT it was destroyed inside the worker's thread exit, under the OS loader lock:
// releasing the private device there waits for driver threads that need the same lock to end, so the join at quit never
// returned (the game hung after "Destroying Direct3D..."). The thread keeps a plain pointer: nothing runs at its exit.
// `issue` makes a thread that outlived a clear() start a fresh cache instead of using a released one.
struct WorkerCaches {std::mutex mutex;std::vector<WorkerCache*> live;std::uint64_t issue=1;};
inline WorkerCaches& workerCaches(){static auto* value=new WorkerCaches;return *value;}
inline WorkerCache& workerCache(){
 thread_local WorkerCache* mine=nullptr;thread_local std::uint64_t mineIssue=0;
 auto& all=workerCaches();std::lock_guard<std::mutex> lock(all.mutex);
 if(!mine||mineIssue!=all.issue){auto fresh=std::make_unique<WorkerCache>();all.live.push_back(fresh.get());mine=fresh.release();mineIssue=all.issue;}
 return *mine;
}
inline std::shared_ptr<const Cached> prepare(const SnapshotPtr& snapshot){
 if(!snapshot)throw std::runtime_error("blood_texture_snapshot_pending");
 if(!snapshot->supported||!snapshot->source.p)throw std::runtime_error("blood_texture_immutable_dds_required");
 auto& cache=workerCache();
 for(const auto& c:cache.entries)if(c->snapshot->source.p==snapshot->source.p&&std::memcmp(&c->snapshot->view,&snapshot->view,sizeof(snapshot->view))==0)return c;
 const auto& desc=snapshot->texture;const unsigned mip=snapshot->view.Texture2D.MostDetailedMip;
 if(mip>=desc.MipLevels)throw std::runtime_error("blood_texture_mip");
 const unsigned width=(std::max)(1u,desc.Width>>mip),height=(std::max)(1u,desc.Height>>mip);
 const std::size_t needed=std::size_t(width)*height*sizeof(PainterBloodTextureCodec::Pixel),budget=128u*1024u*1024u;
 std::size_t used=0;for(const auto& c:cache.entries)used+=c->image.pixels.size()*sizeof(PainterBloodTextureCodec::Pixel);
 if(needed<=budget)for(auto it=cache.entries.begin();used>budget-needed&&it!=cache.entries.end();){if(it->use_count()==1){used-=(*it)->image.pixels.size()*sizeof(PainterBloodTextureCodec::Pixel);it=cache.entries.erase(it);}else ++it;}
 if(needed>budget||used>budget-needed)throw std::runtime_error("blood_texture_cache_budget");
 struct Reader {IReader* p=nullptr;~Reader(){if(p)FS.r_close(p);}} reader;
 reader.p=FS.r_open(snapshot->source.p->path.c_str());if(!reader.p)throw std::runtime_error("blood_texture_source_unavailable");
 if(reader.p->length()!=snapshot->source.p->byteCount)throw std::runtime_error("blood_texture_source_size_changed");
 if(reader.p->length()>128u*1024u*1024u)throw std::runtime_error("blood_texture_source_budget");
 const auto* data=static_cast<const unsigned char*>(reader.p->pointer());std::vector<unsigned char> bytes(data,data+reader.p->length());
 if(!cache.decoder||!cache.decoder->matches(snapshot->device.p))cache.decoder.reset(new PainterBloodTextureWorker::Decoder(snapshot->device.p));
 auto value=std::make_shared<Cached>();value->snapshot=snapshot;value->image=cache.decoder->decode(*snapshot,bytes);cache.entries.push_back(value);return value;
}
// Owner thread, after the bake worker has been joined (PainterBloodShutdown): every worker cache goes here - its decoded
// images, its references on the main device, its private device and context, which no thread uses any more. Never at
// thread exit (see workerCache).
inline void clear(){
 std::vector<WorkerCache*> gone;
 {auto& all=workerCaches();std::lock_guard<std::mutex> lock(all.mutex);gone.swap(all.live);++all.issue;}
 for(auto* cache:gone)delete cache;
}
}
#endif
