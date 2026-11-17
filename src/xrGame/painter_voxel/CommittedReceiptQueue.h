#pragma once
#include "LiveDamageWorker.h"
#include <cassert>
#include <utility>
#include <vector>

namespace gfy_live {
// At most one committed output per owner. While admission is blocked, the
// native owner does not publish another output or split into new owners.
// Store only effect metadata, not the output's mesh or GPU resources.
class CommittedReceiptQueue {
    std::vector<ImpactResult> receipts_;
    std::size_t next_=0;
    static bool eligible(const ImpactResult& r){
        return r.removed&&r.rejection.empty()&&r.entry_surface.valid;
    }
public:
    bool empty()const noexcept{return next_==receipts_.size();}
    std::size_t size()const noexcept{return receipts_.size()-next_;}
    void clear(){receipts_.clear();next_=0;}
    void assign(const std::vector<ImpactResult>& batch){
        assert(empty());clear();
        for(const auto& r:batch)if(eligible(r))receipts_.push_back(r);
    }
    void assign(const ImpactResult& r){
        assert(empty());clear();if(eligible(r))receipts_.push_back(r);
    }
    template<class Deliver> bool drain(Deliver&& deliver){
        while(!empty()){
            if(!deliver(receipts_[next_]))return false;
            ++next_;
        }
        clear();return true;
    }
};
}
