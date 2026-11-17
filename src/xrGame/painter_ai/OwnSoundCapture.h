#pragma once
#include <array>
#include <cmath>
#include <cstdint>
namespace painter { namespace combat { namespace engine {
// Filled only by this observer's accepted sound callback, never the squad table
// or save/load. A row retains the actual acoustic position and original clock.
struct OwnSoundSample {
    std::uint32_t at=0,type=0,source_id=0xffff;
    const void* source=nullptr;
    float x=0,y=0,z=0,power=0;
    bool retained=false;
};
class OwnSoundCapture {
    std::array<OwnSoundSample,8> rows_{};
    std::size_t next_=0,count_=0;
public:
    void clear(){rows_={};next_=count_=0;}
    bool add(std::uint32_t at,std::uint32_t type,const void* source,std::uint32_t id,
        float x,float y,float z,float power){
        if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||!std::isfinite(power)||power<0)return false;
        if(count_<rows_.size())++count_;
        rows_[next_]={at,type,source?id:0xffff,source,x,y,z,power,true};next_=(next_+1)%rows_.size();return true;
    }
    void forget_source(const void* source){if(source)for(auto& r:rows_)if(r.source==source)r={};}
    template<class Sink> void recent(std::uint32_t now,std::uint32_t age_limit,Sink sink)const{
        for(std::size_t i=0;i<count_;++i){const auto& r=rows_[(next_+rows_.size()-count_+i)%rows_.size()];
            const auto age=std::uint32_t(now-r.at);if(r.retained&&age<=age_limit&&age<0x80000000u)sink(r);}
    }
};
}}}
