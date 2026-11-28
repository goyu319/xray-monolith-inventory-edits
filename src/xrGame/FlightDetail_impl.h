namespace flight_detail {
using Mesh=gfy_packed::Mesh;
struct TimingGate {
    std::atomic<std::uint64_t> frame{UINT64_MAX};
    bool take(u32 current) {
        if(frame.load(std::memory_order_relaxed)==current)return false;
        frame.store(current,std::memory_order_relaxed);
        return true;
    }
};
TimingGate prepares,commits;
enum class Phase { Waiting, Recovering, Admitted };
struct Pending {
    painter_flight::Member member;
    Phase phase=Phase::Waiting;
    std::shared_ptr<const Mesh> display;
    std::shared_ptr<painter_damaged_fragment::RenderTicket> ticket;
    u32 retryAfter=0,firstFrame=0;
    bool errorReported=false;
    unsigned validationFailures=0,displayFailures=0;
    void retry(u32 now,const char* why,u16 id){
        if(!errorReported)Msg("[painter_flight] id=%u phase=detail_retry component=%u why=%s",id,member.component,why);
        errorReported=true;retryAfter=now+250;ticket.reset();
    }
};
}
