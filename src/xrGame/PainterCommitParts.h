#pragma once
// piececommit (2026-11-16): the parts of the main-thread frame in which pieces come off (log only).
// part("name") starts the next part of the innermost open timer on this thread (none: nothing happens). The body's apply timer
// (painter_voxel_native::SlowScope, triggerhitch) is one; Scope below is the same timer for the piece / stump poll
// (painter_piece_continuation::poll), which runs outside the body's apply call. Resume puts the caller's part back at its end.
#include <chrono>
#include <cstdio>
#include <cstring>
#include "../Layers/xrRender/PainterHitMain.h"
namespace painter_commit_parts {
struct Sink {virtual void enter(const char* name)=0;protected:~Sink()=default;};
inline thread_local Sink* sink=nullptr;
inline void part(const char* name){if(sink)sink->enter(name);}
struct Resume {const char* name;explicit Resume(const char* n):name(n){}~Resume(){part(name);}
    Resume(const Resume&)=delete;Resume& operator=(const Resume&)=delete;};
// One line per call of 2 ms or more, in the apply timer's format (stage=<stage>, the parts' ms in the order first reached), the
// 30-lines-a-second cap of the hit_main lines, and the slow_main line from 8 ms.
class Scope final : public Sink {
    using Clock=std::chrono::steady_clock;
    const char* stage_;unsigned id_;Sink* outer_;Clock::time_point start_,last_;
    const char* current_="start";const char* names_[32]{};double spent_[32]{};unsigned used_=0;
    static double ms(Clock::time_point from,Clock::time_point to){return std::chrono::duration<double,std::milli>(to-from).count();}
    void add(const char* n,double v){for(unsigned i=0;i<used_;++i)if(names_[i]==n||!std::strcmp(names_[i],n)){spent_[i]+=v;return;}
        if(used_<32){names_[used_]=n;spent_[used_++]=v;}}
public:
    Scope(const char* stage,unsigned id):stage_(stage),id_(id),outer_(sink){start_=last_=Clock::now();sink=this;}
    void enter(const char* next)override{const auto now=Clock::now();add(current_,ms(last_,now));last_=now;current_=next;}
    ~Scope(){sink=outer_;const auto now=Clock::now();const double total=ms(start_,now);
        if(total>=8.)Msg("[painter_perf] id=%u phase=slow_main scope=%s part=%s ms=%.1f",id_,stage_,current_,total);
        if(total<painter_hit_main::threshold_ms||painter_hit_main::current||!painter_hit_main::admit_line())return;
        add(current_,ms(last_,now));unsigned top=0;for(unsigned i=1;i<used_;++i)if(spent_[i]>spent_[top])top=i;
        char parts[1400];int at=0;
        for(unsigned i=0;i<used_&&at>=0&&at<int(sizeof(parts));++i)at+=std::snprintf(parts+at,sizeof(parts)-at," %s=%.2f",names_[i],spent_[i]);
        if(at<0)at=0;if(at>=int(sizeof(parts)))at=int(sizeof(parts))-1;parts[at]=0;
        Msg("[painter_perf] phase=hit_main id=%u stage=%s part=%s ms=%.2f skipped=%u%s",id_,stage_,used_?names_[top]:current_,total,painter_hit_main::take_skipped(),parts);}
    Scope(const Scope&)=delete;Scope& operator=(const Scope&)=delete;
};
}
