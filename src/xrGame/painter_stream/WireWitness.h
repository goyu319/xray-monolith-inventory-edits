#pragma once
#include "Stamp.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace painter_stream {
// Called under the native bridge metadata lock. Storage is bounded; exhaustion
// never overwrites an older emission and takes unchanged synchronous fallback.
template<std::size_t Capacity>
class WireWitnesses final {
public:
    bool Remember(const std::uint8_t* bytes,std::size_t size,const Stamp& stamp)noexcept{
        if(!bytes||size<2||size>16384||!stamp.authenticated||next_==UINT64_MAX)return false;
        for(auto& row:rows_)if(!row.used){
            try{row.bytes.assign(bytes,bytes+size);}catch(...){return false;}
            row.stamp=stamp;row.sequence=++next_;row.used=true;return true;
        }
        return false;
    }
    bool Take(const std::uint8_t* bytes,std::size_t size,Stamp& out)noexcept{
        if(!bytes||size<2||size>16384)return false;
        Row* selected=nullptr;
        for(auto& row:rows_)if(row.used&&row.bytes.size()==size&&std::memcmp(row.bytes.data(),bytes,size)==0&&
            (!selected||row.sequence<selected->sequence))selected=&row;
        if(!selected)return false;
        out=selected->stamp;selected->used=false;selected->bytes.clear();return true;
    }
    void Clear()noexcept{for(auto& row:rows_){row.used=false;row.bytes.clear();}}
private:
    struct Row {std::vector<std::uint8_t> bytes;Stamp stamp;std::uint64_t sequence{};bool used{};};
    std::array<Row,Capacity> rows_;
    std::uint64_t next_{};
};
}
