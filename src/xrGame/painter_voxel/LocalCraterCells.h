#pragma once

// A temporary, one-byte mask per cell, scoped to the actual cut. For unusual
// large sparse regions the caller falls back to the original tile lookups.
struct LocalCraterCells {
    Cell lo{}, hi{};
    std::size_t nx{}, ny{}, nz{};
    std::vector<unsigned char> flags;

    void initialize(const std::vector<Cell>& cells) {
        if(cells.empty()) return;
        lo=hi=cells.front();
        for(Cell c:cells) {
            for(int axis=0;axis<3;++axis) {
                (&lo.x)[axis]=std::min(lo[axis],c[axis]);
                (&hi.x)[axis]=std::max(hi[axis],c[axis]);
            }
        }
        // One sentinel cell at each end lets the local six-neighbour
        // flood use fixed strides without testing grid or window bounds.
        for(int a=0;a<3;++a){--(&lo.x)[a];++(&hi.x)[a];}
        // All supplied coordinates are valid model grid indices.
        nx=std::size_t(hi.x-lo.x)+1;
        ny=std::size_t(hi.y-lo.y)+1;
        nz=std::size_t(hi.z-lo.z)+1;
        const std::size_t limit=2u<<20;
        if(ny>limit/nx || nz>limit/(nx*ny)) return;
        flags.assign(nx*ny*nz,0);
    }
    bool contains(Cell c)const {
        return unsigned(c.x-lo.x)<nx && unsigned(c.y-lo.y)<ny && unsigned(c.z-lo.z)<nz;
    }
    std::size_t index(Cell c)const {
        return (std::size_t(c.x-lo.x)*ny+std::size_t(c.y-lo.y))*nz+std::size_t(c.z-lo.z);
    }
    bool marked(Cell c,unsigned char flag)const {
        return contains(c) && (flags[index(c)]&flag)!=0;
    }
};

// Stable counting passes implement the same integer x/y/z ordering as Cell::<.
// Unlike a bit-set traversal, this retains multiplicity for arbitrary callers.
inline void sortCraterCells(std::vector<Cell>& cells) {
    if(cells.size()<256){std::sort(cells.begin(),cells.end());return;}
    Cell lo=cells.front(),hi=lo;
    for(Cell c:cells) for(unsigned axis=0;axis<3;++axis) {
        (&lo.x)[axis]=std::min(lo[axis],c[axis]);
        (&hi.x)[axis]=std::max(hi[axis],c[axis]);
    }
    const int maxSpan=std::max({hi.x-lo.x,hi.y-lo.y,hi.z-lo.z});
    if(maxSpan>4095){std::sort(cells.begin(),cells.end());return;}
    std::vector<Cell> scratch(cells.size());
    std::vector<std::size_t> offsets(std::size_t(maxSpan)+1);
    for(int axis=2;axis>=0;--axis) {
        std::fill(offsets.begin(),offsets.end(),0);
        for(Cell c:cells)++offsets[c[axis]-lo[axis]];
        std::size_t pos=0;
        for(auto& offset:offsets){const auto count=offset;offset=pos;pos+=count;}
        for(Cell c:cells)scratch[offsets[c[axis]-lo[axis]]++]=c;
        cells.swap(scratch);
    }
}
