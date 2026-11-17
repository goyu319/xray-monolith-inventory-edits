#pragma once
#include <atomic>
#include <cstddef>
// Analytic channels and triangle-local skin ownership share the existing
// 256 MiB field budget. A lease remains charged through queued, current and
// fenced-retired packets; the cache releases the last resource on its owner.
namespace PainterFieldBudget {
constexpr std::size_t capacity=256u<<20;
inline std::atomic<std::size_t>& used(){static auto*p=new std::atomic<std::size_t>{0};return *p;}
struct Lease {
    std::size_t bytes{};
    Lease()=default;Lease(const Lease&)=delete;Lease&operator=(const Lease&)=delete;
    ~Lease(){if(bytes)used().fetch_sub(bytes,std::memory_order_acq_rel);}
    bool Acquire(std::size_t need)noexcept {
        if(bytes||!need||need>capacity)return false;
        auto current=used().load(std::memory_order_acquire);
        while(current<=capacity-need){
            if(used().compare_exchange_weak(current,current+need,std::memory_order_acq_rel)){bytes=need;return true;}
        }
        return false;
    }
};
}
