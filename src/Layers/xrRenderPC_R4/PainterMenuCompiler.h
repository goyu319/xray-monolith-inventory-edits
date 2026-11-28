#pragma once
#include "PainterShaderWorker.h"
#include <chrono>
#include <condition_variable>
#include <map>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>

namespace painter_menu {
class Handle {
    HANDLE h=nullptr;
public:
    Handle()=default;
    explicit Handle(HANDLE v):h(v){}
    Handle(const Handle&)=delete;
    Handle& operator=(const Handle&)=delete;
    Handle(Handle&& o) noexcept:h(o.release()){}
    Handle& operator=(Handle&& o) noexcept {reset(o.release());return *this;}
    ~Handle(){reset();}
    HANDLE get() const{return h;}
    explicit operator bool()const{return h && h!=INVALID_HANDLE_VALUE;}
    HANDLE release(){auto v=h;h=nullptr;return v;}
    void reset(HANDLE v=nullptr){if(*this)CloseHandle(h);h=v;}
};
inline unsigned allowedCores() {
    DWORD_PTR mask=0,system=0;
    if(!GetProcessAffinityMask(GetCurrentProcess(),&mask,&system)||!mask)
        return (std::max)(1u,std::thread::hardware_concurrency());
    unsigned count=0;while(mask){mask&=mask-1;++count;}return count;
}
inline unsigned menuCores(unsigned n){return (std::max)(1u,n/2);}
namespace process {
struct Pipe {Handle reader,writer;};
inline bool pipe(Pipe& p,bool childReads){
    SECURITY_ATTRIBUTES attr{sizeof(attr),nullptr,TRUE};
    HANDLE a=nullptr,b=nullptr;if(!CreatePipe(&a,&b,&attr,16384))return false;
    p.reader.reset(a);p.writer.reset(b);
    return SetHandleInformation(childReads?b:a,HANDLE_FLAG_INHERIT,0)!=FALSE;
}
// Resolve these Vista+ interfaces at runtime because the engine headers target XP.
struct Startup {STARTUPINFOW base;void* attributes;};
struct Attributes {
    using Init=BOOL(WINAPI*)(void*,DWORD,DWORD,SIZE_T*);
    using Update=BOOL(WINAPI*)(void*,DWORD,DWORD_PTR,void*,SIZE_T,void*,SIZE_T*);
    using Drop=void(WINAPI*)(void*);
    std::vector<unsigned char> storage;
    Init init=nullptr;Update update=nullptr;Drop drop=nullptr;
    bool built=false;
    Attributes(){
        HMODULE k=GetModuleHandleW(L"kernel32.dll");
        init=reinterpret_cast<Init>(GetProcAddress(k,"InitializeProcThreadAttributeList"));
        update=reinterpret_cast<Update>(GetProcAddress(k,"UpdateProcThreadAttribute"));
        drop=reinterpret_cast<Drop>(GetProcAddress(k,"DeleteProcThreadAttributeList"));
    }
    ~Attributes(){if(built)drop(storage.data());}
    bool set(HANDLE (&handles)[3],HANDLE& job,bool& atCreation){
        if(!init||!update||!drop)return false;
        SIZE_T bytes=0;init(nullptr,2,0,&bytes);if(!bytes)return false;
        storage.resize(bytes);
        if(!init(storage.data(),2,0,&bytes))return false;
        built=true;
        if(!update(storage.data(),0,0x20002,handles,sizeof(handles),nullptr,nullptr))return false;
        // On Windows 10+ add the job atomically at process creation. Older versions
        // get the same ownership before ResumeThread, via AssignProcessToJobObject.
        atCreation=update(storage.data(),0,0x2000d,&job,sizeof(job),nullptr,nullptr)!=FALSE;
        return true;
    }
};
class Group {
    std::mutex lock;
    Handle job;
    bool closed=false,foreground=false;
    std::vector<HANDLE> children;
public:
    Group():job(CreateJobObjectW(nullptr,nullptr)){
        JOBOBJECT_EXTENDED_LIMIT_INFORMATION i{};
        i.BasicLimitInformation.LimitFlags=JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
        if(job&&!SetInformationJobObject(job.get(),JobObjectExtendedLimitInformation,&i,sizeof(i)))job.reset();
    }
    bool valid() const{return bool(job);}
    HANDLE get() const{return job.get();}
    bool start(PROCESS_INFORMATION& p,bool assigned) {
        std::lock_guard<std::mutex> l(lock);
        if(closed||(!assigned && !AssignProcessToJobObject(job.get(),p.hProcess)))return false;
        if(foreground)SetPriorityClass(p.hProcess,NORMAL_PRIORITY_CLASS);
        if(ResumeThread(p.hThread)==DWORD(-1))return false;
        children.push_back(p.hProcess);
        return true;
    }
    void remove(HANDLE child){
        std::lock_guard<std::mutex> l(lock);
        const auto pos=std::find(children.begin(),children.end(),child);
        if(pos!=children.end())children.erase(pos);
    }
    void promote(){
        std::lock_guard<std::mutex> l(lock);
        foreground=true;
        for(auto child:children)SetPriorityClass(child,NORMAL_PRIORITY_CLASS);
    }
    void close(){
        std::lock_guard<std::mutex> l(lock);
        closed=true;
        if(job)TerminateJobObject(job.get(),ERROR_CANCELLED);
    }
};
class Worker {
    Handle proc,sendTo,recvFrom;
    Group* owner=nullptr;
public:
    bool connected()const{return bool(proc);}
    void reset(){
        if(owner){owner->remove(proc.get());owner=nullptr;}
        sendTo.reset();recvFrom.reset();
        if(proc&&WaitForSingleObject(proc.get(),0)==WAIT_TIMEOUT)TerminateProcess(proc.get(),ERROR_CANCELLED);
        proc.reset();
    }
    ~Worker(){reset();}
    bool open(Group& group,bool load) {
        if(!group.valid())return false;
        Pipe input,output;if(!pipe(input,true)||!pipe(output,false))return false;
        SECURITY_ATTRIBUTES sa{sizeof(sa),nullptr,TRUE};
        Handle err(CreateFileW(L"NUL",GENERIC_WRITE,FILE_SHARE_READ|FILE_SHARE_WRITE,&sa,OPEN_EXISTING,0,nullptr));
        if(!err)return false;
        HANDLE handles[]={input.reader.get(),output.writer.get(),err.get()};
        HANDLE job=group.get();bool assigned=false;
        Attributes a;if(!a.set(handles,job,assigned))return false;
        Startup info{};info.base.cb=sizeof(info);info.base.dwFlags=STARTF_USESTDHANDLES;
        info.base.hStdInput=handles[0];info.base.hStdOutput=handles[1];info.base.hStdError=handles[2];
        info.attributes=a.storage.data();
        wchar_t image[32768];const DWORD n=GetModuleFileNameW(nullptr,image,DWORD(sizeof(image)/sizeof(*image)));
        if(!n||n>=sizeof(image)/sizeof(*image))return false;
        std::wstring cmd=L"\"";cmd+=image;cmd+=L"\" --painter-compile-input-v1";
        DWORD flags=CREATE_SUSPENDED|CREATE_NO_WINDOW|0x00080000u|(load?NORMAL_PRIORITY_CLASS:BELOW_NORMAL_PRIORITY_CLASS);
        PROCESS_INFORMATION pi{};
        if(!CreateProcessW(image,&cmd[0],nullptr,nullptr,TRUE,flags,nullptr,nullptr,&info.base,&pi))return false;
        proc.reset(pi.hProcess);Handle thread(pi.hThread);
        if(!group.start(pi,assigned)){reset();return false;}
        owner=&group;
        input.reader.reset();output.writer.reset();err.reset();
        sendTo=std::move(input.writer);recvFrom=std::move(output.reader);
        std::array<DWORD,7> here{},there{};std::uint32_t v=0;
        const ULONGLONG began=GetTickCount();
        for(;;){
            DWORD available=0;
            if(!PeekNamedPipe(recvFrom.get(),nullptr,0,nullptr,&available,nullptr))return false;
            if(available>=sizeof(v)+sizeof(there))break;
            if(WaitForSingleObject(proc.get(),10)!=WAIT_TIMEOUT || DWORD(GetTickCount()-DWORD(began))>=10000u)return false;
        }
        return wire::compilerFile(here) && wire::receive(recvFrom.get(),&v,sizeof(v)) && v==wire::version &&
               wire::receive(recvFrom.get(),there.data(),sizeof(there)) && here==there;
    }
    bool compile(const Input& input,Result& result) {
        std::uint32_t hr=0,sz=0;
        if(!wire::sendInput(sendTo.get(),input)||!wire::receive(recvFrom.get(),&hr,sizeof(hr))||
           !wire::receive(recvFrom.get(),&sz,sizeof(sz))||sz>wire::maxPacket)return false;
        result.status=static_cast<HRESULT>(hr);result.bytes.resize(sz);
        if(!wire::receive(recvFrom.get(),result.bytes.data(),sz))return false;
        return FAILED(result.status)?sz==0:wire::validDXBC(result.bytes.data(),sz);
    }
};
} // namespace process

// All public operations are called by the renderer owner. Worker threads own no
// references to renderer, VFS, device, D3D handles, or mutable capture buffers.
class Service {
    enum class State{Queued,Running,Done};
    struct Entry {
        Input input;Result result;unsigned cost=0,order=0;State state=State::Queued;
    };
    using Ptr=std::shared_ptr<Entry>;
    struct Earlier {
        bool operator()(const Ptr& a,const Ptr& b)const{
            return a->cost==b->cost ? a->order>b->order : a->cost<b->cost;
        }
    };
    mutable std::mutex lock;
    std::condition_variable wake,drain;
    std::map<std::string,Ptr> entries;
    std::priority_queue<Ptr,std::vector<Ptr>,Earlier> queue;
    std::vector<std::thread> threads;
    std::unique_ptr<process::Group> group;
    unsigned full=allowedCores(),budget=menuCores(full),outstanding=0,totalCount=0;
    bool stopped=false,loading=false,unavailable=false;
    void loop() {
        SetThreadPriority(GetCurrentThread(),THREAD_PRIORITY_BELOW_NORMAL);
        process::Worker worker;
        for(;;){
            Ptr job;bool load;
            {
                std::unique_lock<std::mutex> l(lock);
                if(worker.connected()){
                    if(!wake.wait_for(l,std::chrono::seconds(2),[&]{return stopped||!queue.empty();})){
                        l.unlock();worker.reset();continue;
                    }
                }else wake.wait(l,[&]{return stopped||!queue.empty();});
                if(stopped)return;
                job=queue.top();queue.pop();job->state=State::Running;load=loading;
            }
            Result r;
            bool startupFailure=false;
            try{
                if(!worker.connected()&&!worker.open(*group,load)){worker.reset();startupFailure=true;}
                if(worker.connected()){
                    if(!worker.compile(job->input,r)){worker.reset();r=Result{};}
                }
            }catch(...){worker.reset();r=Result{};}
            {
                std::lock_guard<std::mutex> l(lock);
                if(stopped)return;
                job->result=std::move(r);job->state=State::Done;
                --outstanding;
                if(startupFailure){
                    // A blocked child launch or unmatched compiler applies to this epoch.
                    // Existing healthy workers may finish; the level owns the ordinary fallback.
                    unavailable=true;
                    while(!queue.empty()){queue.top()->state=State::Done;queue.pop();--outstanding;}
                }
                if(!outstanding)drain.notify_all();
            }
        }
    }
    void growLocked(){
        const unsigned count=(std::min)(budget,outstanding);
        while(threads.size()<count) {
            try {threads.emplace_back([this]{loop();});}catch(const std::system_error&){break;}
        }
        if(threads.empty()){
            while(!queue.empty()){queue.top()->state=State::Done;queue.pop();}
            outstanding=0;drain.notify_all();
        }
        wake.notify_all();
    }
public:
    Service()=default;
    Service(const Service&)=delete;Service& operator=(const Service&)=delete;
    ~Service(){cancel();}
    bool submit(Input input,unsigned cost){
        // GOYU: r__painter_shader_workers 0 (default) keeps every compile in this process (the level's own threads).
        // The helper processes are copies of the game exe started with a command line; under Mod Organizer and with
        // the DX12 bridge's dxgi proxy they did not answer, and the level load waited on them for ever.
        extern ECORE_API int ps_r__painter_shader_workers;
        if(!ps_r__painter_shader_workers)return false;
        if(!wire::fitsPacket(input))return false;
        std::lock_guard<std::mutex> l(lock);
        if(unavailable)return false;
        const auto found=entries.find(input.path);
        if(found!=entries.end())return found->second->input.same(input);
        if(!group){group.reset(new process::Group);if(!group->valid()){group.reset();return false;}if(loading)group->promote();}
        auto entry=std::make_shared<Entry>();entry->input=std::move(input);entry->cost=cost;entry->order=totalCount++;
        entries.emplace(entry->input.path,entry);queue.push(entry);++outstanding;growLocked();return true;
    }
    bool ready(const std::string& path)const{
        std::lock_guard<std::mutex> l(lock);auto i=entries.find(path);
        return i!=entries.end()&&i->second->state==State::Done;
    }
    bool take(const Input& current,Result& out){
        std::lock_guard<std::mutex> l(lock);auto i=entries.find(current.path);
        if(i==entries.end()||i->second->state!=State::Done||!i->second->input.same(current))return false;
        out=std::move(i->second->result);entries.erase(i);return true;
    }
    void promote(){std::lock_guard<std::mutex> l(lock);loading=true;budget=full;if(group)group->promote();growLocked();}
    unsigned pending()const{std::lock_guard<std::mutex> l(lock);return outstanding;}
    unsigned total()const{std::lock_guard<std::mutex> l(lock);return totalCount;}
    bool waitFor(unsigned milliseconds){std::unique_lock<std::mutex> l(lock);return drain.wait_for(l,std::chrono::milliseconds(milliseconds),[&]{return outstanding==0||stopped;});}
    void wait(){std::unique_lock<std::mutex> l(lock);drain.wait(l,[&]{return outstanding==0||stopped;});}
    void cancel(){
        {
            std::lock_guard<std::mutex> l(lock);stopped=true;
            if(group)group->close();
            wake.notify_all();drain.notify_all();
        }
        for(auto& t:threads)if(t.joinable())t.join();
        std::lock_guard<std::mutex> l(lock);
        threads.clear();entries.clear();queue={};group.reset();
        outstanding=totalCount=0;loading=false;stopped=false;unavailable=false;budget=menuCores(full);
    }
    void discard(){cancel();}
};
} // namespace painter_menu
