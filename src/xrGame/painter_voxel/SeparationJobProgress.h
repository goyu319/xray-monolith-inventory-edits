#pragma once
// Small CPU-only progress state. Cancellation is cooperative between exact
// preparation phases; it never publishes a partial or approximate separation.
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <stdexcept>
namespace gfy_live {
struct SeparationProgress {
    const char* phase{"queued"}; // Static phase name; valid after a job is retired.
    double elapsed_ms{},queue_ms{},phase_ms{};
    std::uint32_t component{};
    std::uint16_t bone{65535};
    bool started{},finished{},canceled{};
};
class SeparationJobProgress {
    using Clock=std::chrono::steady_clock;
    mutable std::mutex mutex_;
    const Clock::time_point queued_=Clock::now();
    Clock::time_point started_{},phase_started_=queued_,finished_{};
    SeparationProgress state_;
    // Lock-free copy of state_.canceled for the job's cooperative cancel scope
    // (separation-track P1.5: sep::CancelScope / sep::cancelRequested()).
    std::atomic<bool> cancel_flag_{false};
    static double ms(Clock::duration duration){return std::chrono::duration<double,std::milli>(duration).count();}
public:
    void checkpoint(const char* phase,std::uint32_t component=0,std::uint16_t bone=65535){
        std::lock_guard<std::mutex> lock(mutex_);
        if(state_.canceled)throw std::runtime_error("separation_canceled");
        const auto now=Clock::now();
        if(!state_.started){state_.started=true;started_=now;}
        state_.phase=phase;state_.component=component;state_.bone=bone;phase_started_=now;
    }
    void cancel(){std::lock_guard<std::mutex> lock(mutex_);state_.canceled=true;cancel_flag_.store(true);}
    const std::atomic<bool>* cancel_flag()const noexcept{return &cancel_flag_;}
    bool canceled()const{std::lock_guard<std::mutex> lock(mutex_);return state_.canceled;}
    void finish(){std::lock_guard<std::mutex> lock(mutex_);state_.finished=true;finished_=Clock::now();}
    SeparationProgress progress()const{
        std::lock_guard<std::mutex> lock(mutex_);auto out=state_;
        const auto now=state_.finished?finished_:Clock::now();
        out.elapsed_ms=ms(now-queued_);out.queue_ms=ms((state_.started?started_:now)-queued_);
        out.phase_ms=ms(now-phase_started_);return out;
    }
};
}
