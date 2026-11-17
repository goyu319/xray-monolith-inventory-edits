#pragma once
// Mask-only wounds, speed step (painter_voxel_mask_only 3, 2026-11-12): a decide-once tear's BAKE (TearDecide.h bakeBox: the 2-D map
// and the 3-D muscle field the pixel programs read) runs beside the rest of the wound and the separation instead of inside the sever.
// Only the drawing set reads it (TearAtlas.h build, at the separation that makes the piece; the probe's dump): build() waits for
// it here first, and helps - it runs the bake's remaining chunks on its own thread. The bake's numbers are the same (the same code
// per cell, cut into row / slice chunks; every cell written once by one chunk). Its arrays, header and sizes are made before the
// sever returns; a chunk only fills values.
// The chunks run on this file's own threads (as many as the region threads, started once, normal priority): the worker pool is
// first-in first-out, a queued bake would hold the next sever's parallel sections back.
// GFY_MS_BAKE_SYNC=1 (probe knob): the bake inside the sever as before.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>
namespace tear_bake {
struct Pending {
 std::vector<std::function<void()>> chunks;std::function<void(double)> finish; // finish(ms): once, after the last chunk
 std::shared_ptr<const void> keep; // what the chunks read (the decision, the closure field, the block)
 std::atomic<std::size_t> next{0};std::mutex m;std::condition_variable cv;std::size_t done=0;bool finished=false;
 std::chrono::steady_clock::time_point t0=std::chrono::steady_clock::now();
 bool exhausted()const{return next.load(std::memory_order_relaxed)>=chunks.size();}
 // claims and runs one chunk; false when none was left
 bool runOne(){const std::size_t k=next.fetch_add(1,std::memory_order_relaxed);if(k>=chunks.size())return false;
  try{chunks[k]();}catch(...){} // the chunks write preallocated arrays only
  bool last=false;{std::lock_guard<std::mutex> l(m);last=++done==chunks.size();}
  if(last){try{if(finish)finish(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count());}catch(...){}
   {std::lock_guard<std::mutex> l(m);finished=true;}cv.notify_all();}
  return true;}
 void help(){while(runOne()){}}
 void wait(){help();std::unique_lock<std::mutex> l(m);cv.wait(l,[&]{return finished;});}
};
struct Registry {std::mutex m;std::unordered_map<const void*,std::shared_ptr<Pending>> pending;};
inline Registry&registry(){static Registry*r=new Registry;return *r;} // never destroyed: bake threads may outlive static destruction
inline std::atomic<unsigned long long>&waits(){static std::atomic<unsigned long long> n{0};return n;}
// waits for (and helps) the bake of this block, if one is still running; returns the ms waited
inline double await(const void*block){std::shared_ptr<Pending> p;
 {auto&r=registry();std::lock_guard<std::mutex> l(r.m);const auto it=r.pending.find(block);if(it!=r.pending.end())p=it->second;}
 if(!p)return 0;const auto t=std::chrono::steady_clock::now();p->wait();++waits();
 return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t).count();}
class Service {
 std::mutex m;std::condition_variable cv;std::deque<std::shared_ptr<Pending>> q;
 void loop(){core::enterPainterThread();
  for(;;){std::shared_ptr<Pending> p;
   {std::unique_lock<std::mutex> l(m);cv.wait(l,[&]{while(!q.empty()&&q.front()->exhausted())q.pop_front();return !q.empty();});p=q.front();}
   p->runOne();}}
public:
 explicit Service(std::size_t n){for(std::size_t i=0;i<n;++i)std::thread([this]{loop();}).detach();}
 void submit(const std::shared_ptr<Pending>&p){{std::lock_guard<std::mutex> l(m);q.push_back(p);}cv.notify_all();}
};
inline Service&service(){static Service*s=new Service(std::max<std::size_t>(1,core::regionThreadCount()));return *s;} // never destroyed
// registers the pending bake of `block` (its entry leaves the registry when the bake finishes) and starts it
inline void start(const void*block,const std::shared_ptr<Pending>&p){
 auto prevFinish=std::move(p->finish);
 p->finish=[block,prevFinish](double ms){if(prevFinish)prevFinish(ms);auto&r=registry();std::lock_guard<std::mutex> l(r.m);r.pending.erase(block);};
 {auto&r=registry();std::lock_guard<std::mutex> l(r.m);r.pending[block]=p;}
 service().submit(p);}
inline bool syncOnly(){static const bool s=std::getenv("GFY_MS_BAKE_SYNC")!=nullptr;return s;}
// The closure field the current job's severs read, kept alive for their bakes (set by the damage worker for the job; a sever with
// another field or none bakes inside the sever as before)
inline std::shared_ptr<const void>&jobKeep(){thread_local std::shared_ptr<const void> k;return k;}
struct JobKeep {std::shared_ptr<const void> previous;explicit JobKeep(std::shared_ptr<const void> k):previous(std::move(jobKeep())){jobKeep()=std::move(k);}
 ~JobKeep(){jobKeep()=std::move(previous);}JobKeep(const JobKeep&)=delete;JobKeep&operator=(const JobKeep&)=delete;};
} // namespace tear_bake
