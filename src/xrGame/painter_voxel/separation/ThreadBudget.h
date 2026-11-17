#pragma once
#ifndef GFY_SEP_THREAD_BUDGET_INCLUDED
#define GFY_SEP_THREAD_BUDGET_INCLUDED
// Separation thread budget and cooperative cancel scope (separation-track plan P1.0b).
//
// sep::parallel(n,grain,maxThreads,body) has the semantics of core::parallelChunks
// (core/join_cavity.hpp): body(first,last,chunk) over contiguous chunks of [0,n),
// the caller runs chunk 0, a helper that cannot be started is run inline by the
// caller, and the first failing chunk IN CHUNK ORDER rethrows after all chunks
// finished. With maxThreads>=4 and no nesting its chunk boundaries are exactly
// those of core::parallelChunks(n,grain,body).
//
// Budget: one call never uses more than min(maxThreads,jobThreads()) threads including
// the caller (gearpar: jobThreads() is the cap of the separation job on this thread, see
// JobThreads below; 4 outside a separation job). Every chunk of a parallel (>=2 chunk) call runs under a thread_local
// nesting token, and a sep::parallel issued while the token is set runs its body
// serially as one chunk. Code that must not fan out below an existing parallel
// section checks sep::nested(). A separation job that routes all of its parallel
// sections through sep::parallel (or installs sep::NestedScope inside the bodies
// of the parallel sections it does not own) therefore never exceeds its cap.
// A serial (1 chunk) call does not set the token: its body runs on the calling
// thread only, so a nested call below it may still use the full budget.
//
// Cancel: sep::CancelScope installs a thread_local pointer to the job's atomic
// cancel flag; sep::cancelRequested()/sep::throwIfCanceled() read it. sep::parallel
// propagates the caller's flag into its helper threads. Cancellation only ever
// aborts (throws "separation_canceled"); it never produces a partial result.
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <memory>
#include <type_traits>
#include <stdexcept>
#include <thread>
#include <vector>
namespace gfy_live { namespace sep {
constexpr unsigned max_job_threads=16;
constexpr unsigned default_job_threads=4; // outside a separation job (no JobThreads on the thread): the former fixed cap
namespace detail {
inline unsigned& nestingDepth()noexcept{thread_local unsigned depth=0;return depth;}
inline const std::atomic<bool>*& cancelSlot()noexcept{thread_local const std::atomic<bool>* flag=nullptr;return flag;}
inline unsigned& jobCapSlot()noexcept{thread_local unsigned cap=0;return cap;}
inline std::atomic<int>& capOverride()noexcept{static std::atomic<int> value{0};return value;}
}
// Console painter_sep_threads (not saved): 0 automatic, 1..16 a fixed cap for the separations that start after it.
inline void setJobThreadsOverride(int threads)noexcept{detail::capOverride().store(threads>=1&&threads<=int(max_job_threads)?threads:0,std::memory_order_relaxed);}
// The cap a separation job takes when it starts: the console value when set, else GFY_SEP_THREADS=1..16 (offline runs), else 4 on a
// machine with up to 8 hardware threads and half of them, at most 8, on a larger one (16 hardware threads: 8).
inline unsigned configuredJobThreads()noexcept{
 const int fixed=detail::capOverride().load(std::memory_order_relaxed);if(fixed>=1)return unsigned(fixed);
 static const unsigned automatic=[]()noexcept{const char*e=std::getenv("GFY_SEP_THREADS");const int n=e?std::atoi(e):0;
  if(n>=1&&n<=int(max_job_threads))return unsigned(n);
  const unsigned hw=std::thread::hardware_concurrency();return hw<=8?default_job_threads:std::min(hw/2,8u);}();
 return automatic;}
// The cap of the separation job running on this thread; default_job_threads on any other thread.
inline unsigned jobThreads()noexcept{const unsigned cap=detail::jobCapSlot();return cap?cap:default_job_threads;}
// RAII, at the start of every separation job (sep::ActiveJob, DamageSnapshot::prepare_separation): the job takes its cap once
// (an enclosing scope's cap is kept), so every section of one job sees the same cap however the console value changes meanwhile.
// The helpers of a parallel section run nested (one chunk each) and never read it.
struct JobThreads {
 unsigned previous_;
 JobThreads()noexcept:previous_(detail::jobCapSlot()){if(!previous_)detail::jobCapSlot()=configuredJobThreads();}
 ~JobThreads(){detail::jobCapSlot()=previous_;}
 JobThreads(const JobThreads&)=delete;JobThreads&operator=(const JobThreads&)=delete;
};
// True while this thread runs inside a parallel sep::parallel chunk or a NestedScope.
inline bool nested()noexcept{return detail::nestingDepth()!=0;}
// RAII nesting token. Install it at the top of a parallel body that is NOT
// sep::parallel (for example core::parallelChunks or a helper std::thread).
struct NestedScope {
 NestedScope()noexcept{++detail::nestingDepth();}
 ~NestedScope(){--detail::nestingDepth();}
 NestedScope(const NestedScope&)=delete;NestedScope&operator=(const NestedScope&)=delete;
};
// RAII: install the job's cancel flag (may be null) for the current thread.
class CancelScope {
 const std::atomic<bool>* previous_;
public:
 explicit CancelScope(const std::atomic<bool>* flag)noexcept:previous_(detail::cancelSlot()){detail::cancelSlot()=flag;}
 ~CancelScope(){detail::cancelSlot()=previous_;}
 CancelScope(const CancelScope&)=delete;CancelScope&operator=(const CancelScope&)=delete;
};
// The flag installed on this thread (null when the job has none).
inline const std::atomic<bool>* cancelFlag()noexcept{return detail::cancelSlot();}
inline bool cancelRequested()noexcept{const auto flag=detail::cancelSlot();return flag&&flag->load(std::memory_order_relaxed);}
inline void throwIfCanceled(){if(cancelRequested())throw std::runtime_error("separation_canceled");}
// Number of chunks (= threads, caller included) sep::parallel uses for these arguments.
inline std::size_t chunkCount(std::size_t n,std::size_t grain,unsigned maxThreads)noexcept{
 if(nested())return 1;
 const std::size_t cap=std::max(1u,std::min(maxThreads,jobThreads())); // gearpar: the job's cap
 if(grain==0)grain=1;
 return n>=2*grain?std::min<std::size_t>(cap,n/grain):1;
}
namespace detail {
// The multi-chunk path, compiled ONCE (the body is called through a plain function
// pointer): every sep::parallel call site shares one std::thread instantiation. The
// kernel translation unit is close to the COFF section limit of a build without
// /bigobj (separation-track speed2/p1). Same semantics as before, statement for statement.
using ChunkBody=void(*)(void*,std::size_t,std::size_t,std::size_t);
inline void parallelChunks(std::size_t n,std::size_t chunks,ChunkBody body,void*context){
 const auto cancel=detail::cancelSlot();
 std::vector<std::exception_ptr>failure(chunks);
 auto run=[&](std::size_t c){const NestedScope token;const CancelScope scope(cancel);
  try{body(context,n*c/chunks,n*(c+1)/chunks,c);}catch(...){failure[c]=std::current_exception();}};
 std::vector<std::thread>helpers;helpers.reserve(chunks-1);std::size_t started=1;
 try{for(;started<chunks;++started)helpers.emplace_back(run,started);}catch(...){}
 run(0);for(auto&t:helpers)t.join();for(std::size_t c=started;c<chunks;++c)run(c);
 for(auto&f:failure)if(f)std::rethrow_exception(f);
}
}
template<class F>void parallel(std::size_t n,std::size_t grain,unsigned maxThreads,F&&body){
 const std::size_t chunks=chunkCount(n,grain,maxThreads);
 if(chunks<=1){body(std::size_t(0),n,std::size_t(0));return;}
 using Body=std::remove_reference_t<F>;
 detail::parallelChunks(n,chunks,[](void*context,std::size_t first,std::size_t last,std::size_t chunk){(*static_cast<Body*>(context))(first,last,chunk);},
                        const_cast<void*>(static_cast<const volatile void*>(std::addressof(body))));
}
}}
#endif
