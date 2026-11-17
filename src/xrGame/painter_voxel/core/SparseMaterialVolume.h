#pragma once
// Immutable anatomical storage shared by owners; damage remains in sparse tiles.
// Explicit little-endian decoding of the offline ANATCOL1 format.
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <vector>
namespace material_volume {
struct Run {std::uint16_t lo,hi,label;};
struct Sparse {
    std::array<std::uint32_t,3> size{};
    std::array<double,3> origin{};
    double pitch{};
    std::vector<std::uint32_t> offsets;
    std::vector<Run> runs;
    std::uint16_t at(int x,int y,int z)const noexcept {
        if(x<0||y<0||z<0||std::uint32_t(x)>=size[0]||std::uint32_t(y)>=size[1]||std::uint32_t(z)>=size[2])return 0;
        const auto c=std::size_t(y)*size[0]+x;
        auto first=runs.begin()+offsets[c],last=runs.begin()+offsets[c+1];
        auto i=std::upper_bound(first,last,z,[](int z,const Run&r){return z<int(r.lo);});
        if(i==first)return 0;--i;return z<i->hi?i->label:0;
    }
    std::uint16_t point(double x,double y,double z)const noexcept {
        return at(int(std::floor((x-origin[0])/pitch)),int(std::floor((y-origin[1])/pitch)),int(std::floor((z-origin[2])/pitch)));
    }
};
struct Cursor {
    const std::vector<std::uint8_t>& bytes;std::size_t pos{};
    void require(bool ok,const char* why)const {if(!ok)throw std::runtime_error(why);}
    template<class T>T take(){require(pos<=bytes.size()&&sizeof(T)<=bytes.size()-pos,"anatomy_truncated");T value;std::memcpy(&value,bytes.data()+pos,sizeof(value));pos+=sizeof(value);return value;}
};
inline std::shared_ptr<const Sparse> decode(const std::vector<std::uint8_t>& bytes){
    Cursor c{bytes};auto magic=c.take<std::array<char,8>>();c.require(!std::memcmp(magic.data(),"ANATCOL1",8),"anatomy_magic");
    auto out=std::make_shared<Sparse>();out->pitch=c.take<double>();
    c.require(std::isfinite(out->pitch)&&out->pitch>=.0005&&out->pitch<=.01,"anatomy_pitch");
    for(auto& o:out->origin){o=c.take<double>();c.require(std::isfinite(o)&&std::abs(o)<100,"anatomy_origin");}
    for(auto& n:out->size){n=c.take<std::uint32_t>();c.require(n&&n<=65535,"anatomy_dimension");}
    const auto active=c.take<std::uint32_t>();const auto count=c.take<std::uint64_t>();
    const auto columns=std::uint64_t(out->size[0])*out->size[1];
    c.require(columns<=30000000&&active<=columns&&count<=64000000,"anatomy_storage_limit");
    c.require(bytes.size()-c.pos==std::uint64_t(active)*8+count*6,"anatomy_storage_size");
    out->offsets.resize(std::size_t(columns)+1);out->runs.reserve(std::size_t(count));
    std::uint32_t previous=0;std::size_t filled=0;
    for(std::uint32_t n=0;n<active;++n){
        const auto column=c.take<std::uint32_t>(),nr=c.take<std::uint32_t>();
        c.require(column<columns&&(!n||column>previous)&&nr&&nr<=count-out->runs.size(),"anatomy_column_order_or_count");
        while(filled<=column)out->offsets[filled++]=std::uint32_t(out->runs.size());
        std::uint16_t end=0;
        for(std::uint32_t k=0;k<nr;++k){Run r{c.take<std::uint16_t>(),c.take<std::uint16_t>(),c.take<std::uint16_t>()};
            c.require(r.label&&r.lo<r.hi&&r.hi<=out->size[2]&&r.lo>=end,"anatomy_run_invalid");
            out->runs.push_back(r);end=r.hi;
        }previous=column;
    }
    while(filled<out->offsets.size())out->offsets[filled++]=std::uint32_t(out->runs.size());
    c.require(out->runs.size()==count&&c.pos==bytes.size(),"anatomy_final_count");
    return out;
}
}
