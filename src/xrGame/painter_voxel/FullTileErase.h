#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace painter_detail {
// Columns are visited in (u,v,axis) order, u=(axis+1)%3. Return the
// first local (u*8+v) column containing a fresh bit, or 255 if none.
// A full-tile write at this column preserves the old tile first-touch order,
// including old partially removed tiles whose first columns are already full.
inline unsigned char fullTileFirstFreshColumn(const std::array<std::uint64_t,8>* old,int axis) noexcept {
    if(!old)return 0;
    if(axis==0){std::uint64_t fresh=0;for(auto w:*old)fresh|=~w;
        if(!fresh)return 255;unsigned b=0;while(!(fresh&(std::uint64_t(1)<<b)))++b;return static_cast<unsigned char>(b);}
    if(axis==1){for(unsigned z=0;z<8;++z)for(unsigned x=0;x<8;++x)
        if(~(*old)[x]&(0x0101010101010101ull<<z))return static_cast<unsigned char>(z*8+x);return 255;}
    for(unsigned x=0;x<8;++x){const std::uint64_t fresh=~(*old)[x];if(!fresh)continue;
        unsigned b=0;while(!(fresh&(std::uint64_t(1)<<b)))++b;return static_cast<unsigned char>(x*8+b/8);}
    return 255;
}

// lo/hi use the exact source-double Grid sampler at a lattice tile's first
// and last centers. Coordinate monotonicity proves all centers have one slot
// when they lie in a single uniform source page (or entirely outside anatomy).
// Mixed pages and coordinate/page boundaries retain the old exact counters.
template<class Field>
inline int fullTileUniformSlot(const Field& field,const std::array<int,3>& lo,const std::array<int,3>& hi) noexcept {
    for(int a=0;a<3;++a)if(hi[a]<0||lo[a]>=int(field.size[a]))return 0;
    for(int a=0;a<3;++a)if(lo[a]<0||hi[a]<lo[a]||hi[a]>=int(field.size[a]))return -1;
    // A lattice tile that straddles source pages (the damage lattice is offset from the anatomy lattice) is uniform
    // when every page it touches is uniform with one slot: by monotonicity all its centers lie in those pages.
    int slot=-2;
    for(int z=lo[2]>>3;z<=(hi[2]>>3);++z)for(int y=lo[1]>>3;y<=(hi[1]>>3);++y)for(int x=lo[0]>>3;x<=(hi[0]>>3);++x){
        const std::size_t page=(std::size_t(z)*field.pages[1]+unsigned(y))*field.pages[0]+unsigned(x)+Field::headerWords;
        const auto value=field.page[page];if(!(value&Field::uniformBit))return -1;const int s=int(value&63u);
        if(slot==-2)slot=s;else if(slot!=s)return -1;}
    return slot;
}
} // namespace painter_detail
