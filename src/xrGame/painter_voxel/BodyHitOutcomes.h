#pragma once
// Diagnostic metadata only. No event policy, damage or dispatch decisions live here.
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
namespace body_hit_outcomes {
struct Tag {std::uint64_t receipt{};std::uint32_t event{},id{65535},bullet{},bone{65535},alive{};};
using Sink=void(*)(const char*) noexcept;
inline std::atomic<Sink> sink{nullptr};
inline int log_on=0; // hitlogquiet: console painter_hit_log 1 writes these lines (off by default: 3+ lines a bullet)
inline std::atomic<std::uint64_t> next{1};
inline thread_local Tag current{};
inline void set_sink(Sink s)noexcept{sink.store(s,std::memory_order_relaxed);}
inline void report(const char*stage,const char*why,std::uint64_t cookie=0,std::uint64_t sequence=0,Tag tag=current,
 std::uint64_t revision=0,std::size_t pending=0,unsigned inflight=0,std::size_t solid=0,std::size_t mask=0,std::size_t sever=0,unsigned miss=0)noexcept{
 if(!log_on)return;const auto out=sink.load(std::memory_order_relaxed);if(!out)return;
 char reason[241]{};if(!why)why="none";unsigned n=0;for(;n<240&&why[n];++n){const unsigned char c=static_cast<unsigned char>(why[n]);reason[n]=(c>=33&&c<=126&&c!='"'&&c!='\\')?char(c):'_';}
 char line[896];std::snprintf(line,sizeof line,"[painter_body_hit] receipt=%llu event=%u id=%u bullet=%u bone=%u alive=%u cookie=%llu sequence=%llu revision=%llu stage=%s reason=%s pending=%llu inflight=%u solid_cells=%llu mask_cells=%llu sever_cells=%llu miss=%u",
 (unsigned long long)tag.receipt,tag.event,tag.id,tag.bullet,tag.bone,tag.alive,(unsigned long long)cookie,(unsigned long long)sequence,(unsigned long long)revision,stage,reason,
 (unsigned long long)pending,inflight,(unsigned long long)solid,(unsigned long long)mask,(unsigned long long)sever,miss);out(line);
}
struct Scope {Tag previous;bool owns;
 Scope(unsigned id,unsigned bullet,unsigned bone,bool alive,bool onlyMissing=false)noexcept:previous(current),owns(!onlyMissing||!current.receipt){if(owns){current={next.fetch_add(1,std::memory_order_relaxed),0,id,bullet,bone,unsigned(alive)};report("receipt","in_hit");}}
 ~Scope(){if(owns){report("receipt_end","capture_returned");current=previous;}}
};
template<class Events>inline void tag_events(Events&events)noexcept{unsigned n=0;for(auto&e:events){e.hit_trace=current;e.hit_trace.event=++n;report("built","policy_accepted",0,e.sequence,e.hit_trace);}}
template<class Events>inline void batch(const char*stage,const char*why,std::uint64_t cookie,const Events&events,std::size_t start=0)noexcept{for(std::size_t n=start;n<events.size();++n){const auto&e=events[n];report(stage,why,cookie,e.sequence,e.hit_trace);}}
template<class Result>inline const char* result_reason(const Result&r)noexcept{return !r.rejection.empty()?r.rejection.c_str():r.miss?"no_surviving_anatomy_on_ray":r.removed_solid_cells?(r.held_contact?"anatomy_removed_held_contact":"anatomy_removed"):r.removed_mask_cells?"mask_only_cells_removed":r.removed?"cap_hold_no_new_anatomy":"no_new_cells";}
template<class Out>inline void output(const char*stage,const char*why,std::uint64_t cookie,const Out&o)noexcept{for(const auto&r:o.impacts)report(stage,why?why:result_reason(r),cookie,r.sequence,r.hit_trace,o.stamp.revision,0,0,r.removed_solid_cells,r.removed_mask_cells,r.sever_cells,unsigned(r.miss));}
inline const char* enqueue_reason(bool canceled,bool failed,std::size_t pending,std::uint64_t sequence,std::uint64_t submitted)noexcept{return canceled?"owner_canceled":failed?"owner_failed":pending>=256?"pending_queue_256":sequence!=submitted+1?"sequence_mismatch":nullptr;}
struct WaitMemo {const char*reason=nullptr;std::uint64_t first=0,last=0;
 bool changed(const char*r,std::uint64_t a,std::uint64_t b)noexcept{if(reason&&std::strcmp(reason,r)==0&&first==a&&last==b)return false;reason=r;first=a;last=b;return true;}
 void reset()noexcept{reason=nullptr;first=last=0;}
};
}
