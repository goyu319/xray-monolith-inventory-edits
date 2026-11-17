#pragma once
#ifndef GFY_SEP_SEPARATION_CACHE_INCLUDED
#define GFY_SEP_SEPARATION_CACHE_INCLUDED
// Per-(model, quality) separation caches and the warm path (separation-track plan P1.3).
//
// BandJob: one cache build split into independent bands (any order, any thread) and one
// serial finish(). BandTask runs a BandJob: any thread may help by claiming bands from a
// shared cursor; the thread that completes the last band runs finish(), so the value is
// produced exactly once, by the same code as a solo build. A band that throws is recorded;
// the error of the lowest failing band is reported to every waiter and nothing is finished.
//
// FlightCache: single-flight LRU of immutable values. A missing key creates ONE task; every
// concurrent request for the key joins it (claims bands) and then waits for it. Ready
// entries are bounded by an entry cap and a byte budget; in-flight entries are never
// evicted. A failed build is not cached (the next request rebuilds and fails the same way,
// as the uncached code did). Values are exactly what the uncached build returns.
//
// Threads: a real job joins with at most jobThreads() threads (itself included), or one
// fewer when another thread already works on the task, so the job's threads plus the warm
// thread never exceed the job's cap (gearpar: the cap of ThreadBudget.h's JobThreads). A canceled job (sep::cancelRequested) stops claiming
// bands (between bands, never inside one) and throws "separation_canceled"; the task stays
// in flight and the next request for the key completes it. Jobs own the data they read.
//
// Warm: a warm request runs a MISSING key's task on the calling (warm) thread only, one band
// at a time, pausing between bands while a real separation job is active and nobody waits
// for the key. A ready key is returned; a key already in flight is left to its builders
// (null). Everything here is non-template on purpose: the kernel translation unit is close
// to the COFF section limit of a build without /bigobj.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include "ThreadBudget.h"
#if defined(_WIN32)
extern "C" __declspec(dllimport) void* __stdcall GetCurrentThread();
extern "C" __declspec(dllimport) int __stdcall SetThreadPriority(void*,int);
#endif
namespace gfy_live { namespace sep {
// Real separation work in progress (separation jobs and piece candidates).
inline std::atomic<int>& activeJobs()noexcept{static std::atomic<int>count{0};return count;}
// The warm thread's native handle (null while there is none; set by the warm service).
inline std::atomic<void*>& warmThreadHandle()noexcept{static std::atomic<void*>handle{nullptr};return handle;}
// Warm-thread priority: THREAD_PRIORITY_LOWEST while it builds alone, normal as soon as a
// real job waits for a task it works on, so that job never waits behind a starved
// low-priority band or finish() (no priority inversion). No-op off Windows.
inline void setWarmPriority(bool lowest)noexcept{
#if defined(_WIN32)
 if(void*handle=warmThreadHandle().load())(void)SetThreadPriority(handle,lowest?-2:0);
#else
 (void)lowest;
#endif
}
// Set once at shutdown: warm builds stop claiming bands (their tasks stay in flight).
inline std::atomic<bool>& warmStop()noexcept{static std::atomic<bool>stop{false};return stop;}
struct ActiveJob {const JobThreads threads; // gearpar: every real separation job takes its thread cap here
 ActiveJob()noexcept{++activeJobs();} ~ActiveJob(){--activeJobs();} ActiveJob(const ActiveJob&)=delete;ActiveJob&operator=(const ActiveJob&)=delete;};
class BandJob {
public:
 virtual ~BandJob()=default;
 virtual std::size_t bands()const=0;
 virtual void band(std::size_t)=0;
 // The value (and its size for the cache budget), after every band ran.
 virtual std::shared_ptr<const void> finish(std::size_t&bytes)=0;
};
enum class Request {job,warm};
class BandTask {
 std::shared_ptr<BandJob>job_;std::size_t bands_;
 std::atomic<std::size_t>cursor_{0},done_{0};
 std::mutex mutex_;std::condition_variable wake_;bool finished_{};std::exception_ptr error_;std::size_t errorBand_{SIZE_MAX};
 std::shared_ptr<const void>value_;std::size_t bytes_{};
 void complete(){std::exception_ptr error;{std::lock_guard<std::mutex>lock(mutex_);error=error_;}
  std::shared_ptr<const void>value;std::size_t bytes=0;
  if(!error){try{value=job_->finish(bytes);}catch(...){error=std::current_exception();}}
  {std::lock_guard<std::mutex>lock(mutex_);error_=error;value_=std::move(value);bytes_=bytes;finished_=true;}wake_.notify_all();}
public:
 std::atomic<int>waiters{0},workers{0},warmWorkers{0};
 explicit BandTask(std::shared_ptr<BandJob>job):job_(std::move(job)),bands_(job_->bands()){}
 // Claims and runs bands until none is left (true) or this thread must stop (false):
 // a job request stops when its job is canceled; a warm request pauses while a real job is
 // active and nobody waits for this task, and stops at shutdown.
 bool work(Request request){
  ++workers;struct Leave{std::atomic<int>&w;~Leave(){--w;}}leave{workers};
  const bool warm=request==Request::warm;if(warm)++warmWorkers;
  struct LeaveWarm{std::atomic<int>*w;~LeaveWarm(){if(w)--*w;}}leaveWarm{warm?&warmWorkers:nullptr};
  if(bands_==0){std::size_t expected=0;if(done_.compare_exchange_strong(expected,1))complete();return true;}
  for(;;){
   if(warm){
    for(;;){if(warmStop().load())return false;if(activeJobs().load()<=0||waiters.load()>0)break;std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    if(waiters.load()>0)setWarmPriority(false); // a job waits for this task
   }else if(cancelRequested())return false;
   const std::size_t band=cursor_.fetch_add(1);if(band>=bands_)return true;
   try{job_->band(band);}catch(...){std::lock_guard<std::mutex>lock(mutex_);if(band<errorBand_){errorBand_=band;error_=std::current_exception();}}
   if(done_.fetch_add(1)+1==bands_)complete();}
 }
 bool finished(){std::lock_guard<std::mutex>lock(mutex_);return finished_;}
 // Waits for the value; rethrows the task's error.
 std::shared_ptr<const void> wait(std::size_t*bytes=nullptr){std::unique_lock<std::mutex>lock(mutex_);
  while(!finished_)wake_.wait(lock);
  if(error_)std::rethrow_exception(error_);
  if(bytes)*bytes=bytes_;return value_;}
 // Helps on up to maxThreads threads (caller included, nesting-aware), then waits.
 // A cancel request of the calling job stops the helping and throws "separation_canceled".
 std::shared_ptr<const void> join(unsigned maxThreads,std::size_t*bytes=nullptr){
  const auto chunks=std::max<std::size_t>(1,std::min<std::size_t>(maxThreads,bands_));
  parallel(chunks,1,maxThreads,[this](std::size_t,std::size_t,std::size_t){(void)work(Request::job);});
  if(!finished()&&cancelRequested())throw std::runtime_error("separation_canceled");
  return wait(bytes);
 }
};
// A band job run solo on <= maxThreads threads: exactly the value the cache builds.
inline std::shared_ptr<const void> runBands(std::shared_ptr<BandJob>job,unsigned maxThreads=jobThreads()){
 BandTask task(std::move(job));return task.join(maxThreads);}
// Keys are byte strings built field by field (no padding bytes).
class KeyBytes {
 std::string bytes_;
public:
 template<class T>KeyBytes&add(const T&v){bytes_.append(reinterpret_cast<const char*>(&v),sizeof v);return *this;}
 const std::string&str()const{return bytes_;}
};
class FlightCache {
 struct Entry {std::string key;std::shared_ptr<BandTask>task;std::size_t bytes{};bool ready{};std::shared_ptr<const void>value;};
 std::mutex mutex_;std::vector<Entry>entries_; // LRU order: least recently used first
 std::size_t cap_,budget_;
 void evictLocked(){
  std::size_t used=0,ready=0;for(const auto&e:entries_)if(e.ready){used+=e.bytes;++ready;}
  for(std::size_t i=0;i<entries_.size()&&(used>budget_||ready>cap_);){
   if(!entries_[i].ready){++i;continue;}used-=entries_[i].bytes;--ready;entries_.erase(entries_.begin()+std::ptrdiff_t(i));}
 }
 void touchLocked(std::size_t i){Entry e=std::move(entries_[i]);entries_.erase(entries_.begin()+std::ptrdiff_t(i));entries_.push_back(std::move(e));}
public:
 FlightCache(std::size_t cap,std::size_t budget):cap_(cap),budget_(budget){}
 // make(context) creates the build job for a missing key; it runs under the cache mutex
 // (cheap setup only). Returns null only for a warm request of a key in flight elsewhere
 // (or stopped at shutdown).
 using Make=std::shared_ptr<BandJob>(*)(const void*context);
 std::shared_ptr<const void> get(const std::string&key,Make make,const void*context,Request request=Request::job,unsigned helpers=jobThreads()){
  std::shared_ptr<BandTask>task;bool created=false;
  {std::lock_guard<std::mutex>lock(mutex_);
   for(std::size_t i=0;i<entries_.size();++i)if(entries_[i].key==key){
    if(entries_[i].ready){auto value=entries_[i].value;touchLocked(i);return value;}
    if(request==Request::warm)return {};
    task=entries_[i].task;break;}
   if(!task){task=std::make_shared<BandTask>(make(context));Entry e;e.key=key;e.task=task;entries_.push_back(std::move(e));created=true;}}
  std::exception_ptr error;bool canceled=false;std::shared_ptr<const void>value;std::size_t bytes=0;
  try{
   if(request==Request::warm){if(!task->work(Request::warm))return {};value=task->wait(&bytes);}
   else{const unsigned threads=created||task->workers.load()==0?helpers:std::max(1u,helpers-1);
    struct Waiting{std::atomic<int>&w;explicit Waiting(std::atomic<int>&x):w(x){++w;}~Waiting(){--w;}}waiting(task->waiters);
    if(task->warmWorkers.load()>0)setWarmPriority(false); // the warm thread holds a band or runs finish()
    value=task->join(threads,&bytes);}
  }catch(const std::exception&e){error=std::current_exception();canceled=std::strcmp(e.what(),"separation_canceled")==0;}
  catch(...){error=std::current_exception();}
  if(!canceled){std::lock_guard<std::mutex>lock(mutex_);
   for(std::size_t i=0;i<entries_.size();++i)if(entries_[i].task==task){
    if(error)entries_.erase(entries_.begin()+std::ptrdiff_t(i));
    else if(!entries_[i].ready){
     // A value larger than the whole budget is returned but not kept (as before).
     if(bytes>budget_)entries_.erase(entries_.begin()+std::ptrdiff_t(i));
     else{entries_[i].ready=true;entries_[i].bytes=bytes;entries_[i].value=value;entries_[i].task.reset();touchLocked(i);evictLocked();}}
    break;}}
  if(error)std::rethrow_exception(error);
  return value;
 }
};
}}
#endif
