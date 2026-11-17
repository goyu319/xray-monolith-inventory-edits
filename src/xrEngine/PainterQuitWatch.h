#pragma once
#include <windows.h>
#include <tlhelp32.h>
#include <atomic>
namespace PainterQuitWatch {
struct State {
    std::atomic<const char*> stage{"main_loop_ended"};
    std::atomic<unsigned> reached{0};
    std::atomic<bool> armed{false};
    DWORD mainThread=0,stallMs=0;
    HANDLE stalled=nullptr;
    std::atomic<void (*)(const char*)> tell{nullptr}; // one best-effort copy of the head of the report for the engine log; may be null
    void (*hooks[8])()={};
    std::atomic<unsigned> hookCount{0};
    char path[MAX_PATH]={};
    char text[3072]={};   // stage + main thread: also told to the engine log
    char more[61440]={};  // the other threads: file only
};
inline State& state()noexcept{static State value;return value;}
// A teardown stage begins. `name` is a string literal. Costs two atomic stores; does nothing visible before arm().
inline void stage(const char* name)noexcept{auto& s=state();s.stage.store(name,std::memory_order_release);s.reached.fetch_add(1,std::memory_order_release);}
// Any thread, any time before the quit (static initialisation included): `stop` ends this module's worker threads.
inline void atQuit(void (*stop)())noexcept{auto& s=state();const unsigned at=s.hookCount.fetch_add(1,std::memory_order_acq_rel);if(at<8)s.hooks[at]=stop;}
// Main thread, once, when the game objects are gone and before the device goes.
inline void runQuitHooks()noexcept{
    auto& s=state();unsigned count=s.hookCount.load(std::memory_order_acquire);if(count>8)count=8;
    for(unsigned at=0;at<count;++at)if(const auto stop=s.hooks[at]){s.hooks[at]=nullptr;stop();}
}
namespace detail {
inline char* put(char* p,char* end,const char* s)noexcept{while(s&&*s&&p<end)*p++=*s++;return p;}
inline char* number(char* p,char* end,unsigned long long v,unsigned base,unsigned width=0)noexcept{
    char digits[24];unsigned n=0;do{digits[n++]="0123456789abcdef"[v%base];v/=base;}while(v&&n<sizeof(digits));
    while(n<width&&n<sizeof(digits))digits[n++]='0';
    while(n&&p<end)*p++=digits[--n];return p;
}
// The module's own name from its export table, read from memory (no loader call); empty for the game's exe.
inline const char* moduleName(const unsigned char* base)noexcept{
    __try{
        const auto* dos=reinterpret_cast<const IMAGE_DOS_HEADER*>(base);if(!base||dos->e_magic!=IMAGE_DOS_SIGNATURE)return "";
        const auto* nt=reinterpret_cast<const IMAGE_NT_HEADERS*>(base+dos->e_lfanew);if(nt->Signature!=IMAGE_NT_SIGNATURE)return "";
        const auto& dir=nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_EXPORT];if(!dir.VirtualAddress||!dir.Size)return "";
        const auto* exports=reinterpret_cast<const IMAGE_EXPORT_DIRECTORY*>(base+dir.VirtualAddress);
        return exports->Name?reinterpret_cast<const char*>(base+exports->Name):"";
    }__except(EXCEPTION_EXECUTE_HANDLER){return "";}
}
// The thread's call stack, innermost call first, one "module+0xoffset" per line. The thread is suspended meanwhile.
inline char* stack(char* p,char* end,DWORD thread,int frames=64)noexcept{
    HANDLE h=OpenThread(THREAD_SUSPEND_RESUME|THREAD_GET_CONTEXT|THREAD_QUERY_INFORMATION,FALSE,thread);
    if(!h)return put(p,end,"  (the thread could not be opened)\r\n");
    if(SuspendThread(h)==DWORD(-1)){CloseHandle(h);return put(p,end,"  (the thread could not be suspended)\r\n");}
#if defined(_M_X64)
    __try{
        CONTEXT c;c.ContextFlags=CONTEXT_CONTROL|CONTEXT_INTEGER;
        if(GetThreadContext(h,&c)){
            for(int frame=0;frame<frames&&c.Rip;++frame){
                PVOID base=nullptr;RtlPcToFileHeader(reinterpret_cast<PVOID>(c.Rip),&base);
                const char* name=moduleName(static_cast<const unsigned char*>(base));
                p=put(p,end,"  ");p=put(p,end,base?(name[0]?name:"exe"):"?");p=put(p,end,"+0x");
                p=number(p,end,c.Rip-reinterpret_cast<DWORD64>(base),16);p=put(p,end,"\r\n");
                DWORD64 image=0;PRUNTIME_FUNCTION function=RtlLookupFunctionEntry(c.Rip,&image,nullptr);
                if(!function){c.Rip=*reinterpret_cast<const DWORD64*>(c.Rsp);c.Rsp+=8;} // a leaf: the return address is on top
                else{PVOID handlerData=nullptr;DWORD64 establisher=0;RtlVirtualUnwind(UNW_FLAG_NHANDLER,image,c.Rip,function,&c,&handlerData,&establisher,nullptr);}
            }
        }else p=put(p,end,"  (the thread's registers could not be read)\r\n");
    }__except(EXCEPTION_EXECUTE_HANDLER){p=put(p,end,"  (the stack ends here: unreadable)\r\n");}
#else
    p=put(p,end,"  (no stack on this architecture)\r\n");
#endif
    ResumeThread(h);CloseHandle(h);return p;
}
// Every other thread of the process, eight innermost calls each (the thread a join waits for is among them).
inline char* others(char* p,char* end,DWORD mainThread)noexcept{
    const DWORD self=GetCurrentThreadId(),process=GetCurrentProcessId();
    HANDLE snapshot=CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD,0);
    if(snapshot==INVALID_HANDLE_VALUE)return put(p,end,"(the thread list could not be read)\r\n");
    THREADENTRY32 entry;entry.dwSize=sizeof(entry);
    if(Thread32First(snapshot,&entry))do{
        if(entry.th32OwnerProcessID!=process||entry.th32ThreadID==self||entry.th32ThreadID==mainThread)continue;
        if(end-p<400)break;
        p=put(p,end,"thread ");p=number(p,end,entry.th32ThreadID,10);p=put(p,end,":\r\n");p=stack(p,end,entry.th32ThreadID,8);
    }while(Thread32Next(snapshot,&entry));
    CloseHandle(snapshot);return p;
}
// Started with the watch, so it exists before anything can stall: three seconds after a stall is seen the process ends,
// whatever the report below runs into.
inline DWORD WINAPI backstop(void*)noexcept{
    WaitForSingleObject(state().stalled,INFINITE);Sleep(3000);TerminateProcess(GetCurrentProcess(),0);return 0;
}
inline DWORD WINAPI watch(void*)noexcept{
    auto& s=state();unsigned seen=s.reached.load(std::memory_order_acquire);DWORD quiet=0;
    for(;;){
        Sleep(250);const unsigned now=s.reached.load(std::memory_order_acquire);
        if(now!=seen){seen=now;quiet=0;continue;}
        quiet+=250;if(quiet>=s.stallMs)break;
    }
    SetEvent(s.stalled);
    char* p=s.text;char* const end=s.text+sizeof(s.text)-1;SYSTEMTIME t;GetLocalTime(&t);
    p=number(p,end,t.wYear,10,4);p=put(p,end,"-");p=number(p,end,t.wMonth,10,2);p=put(p,end,"-");p=number(p,end,t.wDay,10,2);p=put(p,end," ");
    p=number(p,end,t.wHour,10,2);p=put(p,end,":");p=number(p,end,t.wMinute,10,2);p=put(p,end,":");p=number(p,end,t.wSecond,10,2);
    p=put(p,end," quit stalled: stage \"");p=put(p,end,s.stage.load(std::memory_order_acquire));p=put(p,end,"\" did not end within ");
    p=number(p,end,quiet,10);p=put(p,end," ms; the process is ended here. Main thread, innermost call first (module+offset):\r\n");
    p=stack(p,end,s.mainThread);*p=0;
    HANDLE file=s.path[0]?CreateFileA(s.path,GENERIC_WRITE,FILE_SHARE_READ,nullptr,CREATE_ALWAYS,FILE_ATTRIBUTE_NORMAL,nullptr):INVALID_HANDLE_VALUE;
    DWORD written=0;
    if(file!=INVALID_HANDLE_VALUE){WriteFile(file,s.text,DWORD(p-s.text),&written,nullptr);FlushFileBuffers(file);} // the head is on disk first
    if(const auto tell=s.tell.load(std::memory_order_acquire))tell(s.text);
    if(file!=INVALID_HANDLE_VALUE){
        char* q=s.more;char* const last=s.more+sizeof(s.more)-1;
        q=put(q,last,"Other threads, eight innermost calls each:\r\n");q=others(q,last,s.mainThread);
        WriteFile(file,s.more,DWORD(q-s.more),&written,nullptr);CloseHandle(file);
    }
    Sleep(300); // the log thread writes the told lines
    TerminateProcess(GetCurrentProcess(),0);return 0;
}
} // namespace detail
// Main thread, once, when the main loop has ended. stallFile: where the report goes (may be null); stallMs: how long
// one stage may take. A stage that is merely slow must stay below it.
inline void arm(const char* stallFile,DWORD stallMs=15000,void (*tell)(const char*)=nullptr)noexcept{
    auto& s=state();if(s.armed.exchange(true,std::memory_order_acq_rel))return;
    s.mainThread=GetCurrentThreadId();s.stallMs=stallMs<1000?1000:stallMs;s.tell.store(tell,std::memory_order_release);
    if(stallFile)lstrcpynA(s.path,stallFile,MAX_PATH);
    s.stalled=CreateEventA(nullptr,TRUE,FALSE,nullptr);if(!s.stalled)return;
    if(HANDLE h=CreateThread(nullptr,0,&detail::backstop,nullptr,0,nullptr))CloseHandle(h);
    if(HANDLE h=CreateThread(nullptr,0,&detail::watch,nullptr,0,nullptr))CloseHandle(h);
}
// The engine log is about to close: a later stall is still written to the stall file, but no longer told to the log.
inline void silence()noexcept{state().tell.store(nullptr,std::memory_order_release);}
} // namespace PainterQuitWatch
