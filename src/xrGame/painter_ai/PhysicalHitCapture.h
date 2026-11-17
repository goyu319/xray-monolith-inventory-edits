#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace painter { namespace combat { namespace engine {
// Per-observer value ring, populated ONLY by the physical nonzero damage path.
// Stock hit memory also holds heard gunshots and squad-shared records and must
// never act as this producer. Native pointers remain authority-thread-local;
// remove_links erases the source reference, preserving the actual hurt fact.
struct PhysicalHitSample {
    std::uint32_t at=0,source_id=0xffff;
    const void* source=nullptr;
    float x=0,y=0,z=0;
    bool hostile_attack=true;
};
class PhysicalHitCapture {
    std::array<PhysicalHitSample,4> rows_{};
    std::size_t next_=0,count_=0;
    std::uint64_t overflow_=0;
public:
    void clear(){rows_={};next_=count_=0;overflow_=0;}
    bool add(float damage,std::uint32_t at,const void* source,std::uint32_t id,float x,float y,float z,bool hostile_attack=true){
        if(!std::isfinite(damage)||damage<=0||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z))return false;
        if(count_==rows_.size())++overflow_;else ++count_;
        rows_[next_]={at,source?id:0xffff,source,x,y,z,hostile_attack};next_=(next_+1)%rows_.size();return true;
    }
    void forget_source(const void* source){for(auto& row:rows_)if(row.source==source){row.source=nullptr;row.source_id=0xffff;}}
    std::uint64_t overflow()const{return overflow_;}
    template<class Sink> void since(std::uint32_t previous,std::uint32_t now,Sink sink)const{
        // Unsigned ages preserve an actual u32 engine clock wrap. No replayed
        // sample is restamped; backward/future records cannot become fresh hits.
        const auto span=std::uint32_t(now-previous);if(span>=0x80000000u)return;
        for(std::size_t i=0;i<count_;++i){const auto& row=rows_[(next_+rows_.size()-count_+i)%rows_.size()];
            const auto age=std::uint32_t(now-row.at);if(age<span&&age<0x80000000u)sink(row);}
    }
};
}}}
