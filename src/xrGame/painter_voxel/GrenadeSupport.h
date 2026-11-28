// All source material inside the ordinary grenade envelopes that the shaped
// floor left standing must remain face-connected to surviving material outside
// those envelopes. Work on the original label lattice, within changed bites
// and their one-brick halo only. No exterior mesh or whole-body scan.
#pragma once

struct GrenadeStockCut {
    using Bits=std::vector<std::pair<Cell,core::Tile>>;
    Tunnel envelope;
    double depthLimit{};
    std::size_t impact{};
    std::shared_ptr<const Bits> raster;
};


// Decode eight native x samples from one compressed source page.
static unsigned grenadePageRow(const painter_detail::TissueField& f,int x,int y,int z){
    if(y<0||z<0||unsigned(y)>=f.size[1]||unsigned(z)>=f.size[2]||x+7<0||x>=int(f.size[0]))return 0;
    if(x<0||x+7>=int(f.size[0])){
        unsigned row=0;
        for(int i=0;i<8;++i){unsigned s=f.at(x+i,y,z)&127u;if(s&&s!=16)row|=1u<<i;}
        return row;
    }
    const auto page=f.page[16+(std::size_t(z>>3)*f.pages[1]+unsigned(y>>3))*f.pages[0]+unsigned(x>>3)];
    auto present=[](unsigned s){s&=127u;return unsigned(s!=0&&s!=16);};
    if(page&0x80000000u)return present(page)?255:0;
    const unsigned bits=1u<<((page>>28)&3u),base=page&0x0fffffffu;
    const unsigned row=(unsigned(z&7)*8+unsigned(y&7));
    const auto* p=f.data.data()+base;
    if(bits==1){
        const unsigned a=present(p[0]),b=present(p[1]);
        return a==b?(a?255u:0u):unsigned(p[2+row])^(a?255u:0u);
    }
    if(bits==8){unsigned m=0;for(unsigned i=0;i<8;++i)m|=present(p[row*8+i])<<i;return m;}
    const unsigned palette=1u<<bits;
    const auto* packed=p+palette+row*bits;unsigned m=0;
    for(unsigned i=0;i<8;++i)m|=present(p[(packed[(i*bits)>>3]>>((i*bits)&7))&(palette-1)])<<i;
    return m;
}
template<int Ratio>static unsigned grenadeNativeRow(const painter_detail::TissueField& f,int x,int y,int z){
    const int shift=int(unsigned(x)&7u);const int first=x-shift;
    // The fourth ratio occupies 32 bits; use a wider intermediate for unaligned rows.
    std::uint64_t bits=0;
    for(int offset=0;offset<8*Ratio+shift;offset+=8)
        bits|=std::uint64_t(grenadePageRow(f,first+offset,y,z))<<offset;
    return unsigned(bits>>shift);
}

#include "GrenadePresenceRows.h"

// Convert the existing x-major removal words into eight consecutive x-lane
// masks, one per z, at the supplied coarse y. No native samples are inferred.
static std::uint64_t grenadeXRows(const core::Tile& tile,int y) {
    std::uint64_t rows=0;
    for(unsigned x=0;x<8;++x)rows|=((tile.removed[x]>>(y*8))&255ull)<<(8*x);
    auto swap=[&](int shift,std::uint64_t mask){
        const auto t=(rows^(rows>>shift))&mask;
        rows^=t^(t<<shift);
    };
    swap(7,0x00aa00aa00aa00aaull);swap(14,0x0000cccc0000ccccull);
    swap(28,0x00000000f0f0f0f0ull);
    return rows;
}

template<int Ratio> static unsigned grenadeExpandRow(unsigned coarse) {
    static const auto table=[]{
        std::array<unsigned,256> result{};
        for(unsigned mask=0;mask<256;++mask)
            for(int x=0;x<8;++x)if(mask&(1u<<x))result[mask]|=((1u<<Ratio)-1u)<<(x*Ratio);
        return result;
    }();
    return table[coarse];
}

template<int Ratio>
static void resolveGrenadeSupport(Grid& grid,const painter_detail::TissueField& field,Cell nativeOrigin,
                                  const std::vector<GrenadeStockCut>& cuts,
                                  std::vector<ImpactResult>& impacts) {
    using Word=std::conditional_t<(Ratio<=2),std::uint16_t,std::uint32_t>;
    constexpr int N=8*Ratio, Rows=N*N;
    struct Brick {
        Cell key{};
        std::vector<std::pair<std::uint16_t,signed char>> cutters;
        std::array<Word,Rows> full{},linked{};
        core::Tile candidates{};
        std::array<std::uint64_t,8> cutRows{};
        unsigned haloFaces{};
        std::array<int,6> next{{-1,-1,-1,-1,-1,-1}};
        bool eligible{},queued{};
    };
    static_assert(Ratio>=1 && Ratio<=4,"native support resolution");
    if(cuts.empty()||cuts.size()>=65535)return;
    std::unordered_map<Cell,int,core::CellHash> index;
    std::deque<Brick> bricks;
    auto idOf=[&](Cell key) {
        auto entry=index.emplace(key,int(bricks.size()));
        if(entry.second){bricks.emplace_back();bricks.back().key=key;}
        return entry.first->second;
    };
    const Vec half{grid.pitch*3.5,grid.pitch*3.5,grid.pitch*3.5};
    const double gp=grid.pitch, voxel=gp*gp*gp;
    for(std::size_t i=0;i<cuts.size();++i) {
        const auto& stamp=cuts[i];const auto& cut=stamp.envelope;
        if(stamp.impact>=impacts.size()||!impacts[stamp.impact].removed||!impacts[stamp.impact].rejection.empty())continue;
        if(stamp.raster){
            for(const auto& pair:*stamp.raster){
                const Cell k=pair.first,base{k.x*8,k.y*8,k.z*8};
                const auto* old=grid.findTile(k);
                for(int x=0;x<8;++x){
                    auto bits=pair.second.removed[x]&~(old?old->removed[x]:0ull);
                    if(!bits)continue;
                    if(grid.ownership||stamp.depthLimit<cut.penetration){
                        for(auto todo=bits;todo;todo&=todo-1){
                            const unsigned bit=core::lowestBit(todo);
                            const Cell c{base.x+x,base.y+int(bit/8),base.z+int(bit%8)};
                            if((grid.ownership&&!grid.ownerCuttable(c))||
                               (stamp.depthLimit<cut.penetration&&dot(grid.center(c)-cut.entry,cut.direction)>stamp.depthLimit))
                                bits&=~(1ull<<bit);
                        }
                    }
                    if(bits){auto& b=bricks[idOf(k)];b.candidates.removed[x]|=bits;b.eligible=true;}
                }
            }
            continue;
        }
        const auto a=cut.entry-cut.direction*cut.inlet, b=cut.entry+cut.direction*std::min(cut.penetration,stamp.depthLimit);
        const double r=cut.maxRadius();
        Cell lo{},hi{};
        for(int k=0;k<3;++k) {
            (&lo.x)[k]=std::max(0,int(std::floor((std::min(a[k],b[k])-r-grid.origin[k])/(8*gp))));
            (&hi.x)[k]=std::min((grid.size[k]-1)/8,int(std::floor((std::max(a[k],b[k])+r-grid.origin[k])/(8*gp))));
        }
        for(int x=lo.x;x<=hi.x;++x)for(int y=lo.y;y<=hi.y;++y)for(int z=lo.z;z<=hi.z;++z) {
            const Cell k{x,y,z},base{x*8,y*8,z*8};
            const Vec mid=grid.center(base)+half;
            const int box=cut.envelopeBox(mid,half);
            if(box<0)continue;
            auto& brick=bricks[idOf(k)];
            brick.cutters.emplace_back(std::uint16_t(i),static_cast<signed char>(box));
        }
    }
    // The legacy round-tool raster has certified eight-cell SIMD columns.
    // Build its immutable angle tables once, then raster independent bricks.
    // The input order inside a brick gives each restored cell its original
    // owning event, even when two old envelopes cover that cell.
    using Filter=painter_detail::tile_tunnel_detail::TunnelCellFilter<Tunnel>;
    std::vector<Filter> filters;
#ifdef __AVX__
    using Columns=painter_detail::tile_tunnel_detail::TunnelColumnEval<Tunnel>;
    std::vector<Columns> columns;
#endif
    filters.reserve(cuts.size());
    for(const auto& stamp:cuts) {
        filters.emplace_back(stamp.envelope);
#ifdef __AVX__
        columns.emplace_back(stamp.envelope,2,gp);
#endif
    }
    core::parallelChunks(bricks.size(),32,[&](std::size_t first,std::size_t last,std::size_t){
        for(std::size_t bi=first;bi<last;++bi) {
            if(bricks[bi].cutters.empty())continue;
            auto& brick=bricks[bi];const Cell k=brick.key,base{k.x*8,k.y*8,k.z*8};
            const core::Tile* old=grid.findTile(k);
            const bool inner=base.x+7<grid.size.x&&base.y+7<grid.size.y&&base.z+7<grid.size.z;
            for(const auto& packed:brick.cutters) {
                const auto i=packed.first;const auto& stamp=cuts[i];const auto& cut=stamp.envelope;
                const bool cap=stamp.depthLimit<cut.penetration;
                for(int x=0;x<8;++x)for(int y=0;y<8;++y) {
                    const unsigned oldRow=old ? unsigned(old->removed[x]>>(8*y))&255u : 0u;
                    if(oldRow==255u)continue;
                    Cell c{base.x+x,base.y+y,base.z};
                    const Vec p0=grid.center(c);
                    unsigned inside=0,open=0;
                    if(packed.second>0)inside=255u;
                    else {
#ifdef __AVX__
                        if(columns[i].usable)columns[i].column(p0,8,inside,open);
                        else
#endif
                        {
                            const int span=cut.envelopeSegment(p0,Vec{0,0,gp},8);
                            if(span>0)inside=255u;else if(span==0)open=255u;
                        }
                    }
                    const unsigned already=unsigned(brick.candidates.removed[x]>>(8*y))&255u;
                    open&=~(oldRow|already);
                    for(;open;open&=open-1u) {
                        const unsigned z=core::lowestBit(open);c.z=base.z+int(z);

                        const Vec p=grid.center(c);int match=filters[i].classify(p);
                        if(match<0)match=cut.insideEnvelope(p)?1:0;
                        if(match)inside|=1u<<z;
                    }
                    inside&=~(oldRow|already);
                    if(!inner||grid.ownership||cap){
                        for(unsigned bits=inside;bits;bits&=bits-1){
                            const unsigned z=core::lowestBit(bits);c.z=base.z+int(z);
                            if((!inner&&!grid.valid(c))||(grid.ownership&&!grid.ownerCuttable(c))||
                               (cap&&dot(grid.center(c)-cut.entry,cut.direction)>stamp.depthLimit))inside&=~(1u<<z);
                        }
                    }
                    brick.candidates.removed[x]|=std::uint64_t(inside)<<(8*y);
                    brick.eligible|=inside!=0;
                }
            }
        }
    });
    // The halo supplies the native roots for cells at the edge of a stock cutter.
    // Every processed tile belongs to an admitted local cutter or to its halo.
    const std::size_t nCut=bricks.size();
    for(std::size_t i=0;i<nCut;++i)if(bricks[i].eligible) {
        for(int ax=0;ax<3;++ax)for(int side=0;side<2;++side) {
            Cell k=bricks[i].key;(&k.x)[ax]+=side?1:-1;
            if(k[ax]>=0&&k[ax]<=(grid.size[ax]-1)/8){
                const int id=idOf(k);
                bricks[i].next[2*ax+side]=id;
                bricks[id].haloFaces|=1u<<(2*ax+1-side);
            }
        }
    }
    // Every non-candidate present voxel is a root. Only roots sharing
    // a face with a candidate can change its answer. The native six-face
    // flood and the coarse any-survivor publication rule stay the same.
    for(std::size_t j=0;j<nCut;++j)if(bricks[j].eligible)
        for(int y=0;y<8;++y)bricks[j].cutRows[y]=grenadeXRows(bricks[j].candidates,y);
    core::parallelChunks(bricks.size(),32,[&](std::size_t first,std::size_t last,std::size_t){
        for(std::size_t j=first;j<last;++j) {
            auto& brick=bricks[j];
            const Cell base{brick.key.x*8,brick.key.y*8,brick.key.z*8};
            const auto* removed=grid.findTile(brick.key);
            const bool inBounds=base.x>=0&&base.y>=0&&base.z>=0&&base.x+7<grid.size.x&&base.y+7<grid.size.y&&base.z+7<grid.size.z;
            if(!brick.eligible&&!brick.haloFaces)continue;
            GrenadePresenceRows<Ratio> presence(field,nativeOrigin.x+base.x*Ratio,
                nativeOrigin.y+base.y*Ratio,nativeOrigin.z+base.z*Ratio);
            const Brick* neighbours[6]{};
            for(int dir=0;dir<6;++dir) {
                const int axis=dir/2;
                Cell nk=brick.key;
                const int step=dir%2?1:-1;
                if(axis==0)nk.x+=step;else if(axis==1)nk.y+=step;else nk.z+=step;
                const auto found=index.find(nk);
                if(found!=index.end())neighbours[dir]=&bricks[found->second];
            }
            auto neighbourRows=[&](int dir,int y)->std::uint64_t {
                return neighbours[dir]?neighbours[dir]->cutRows[y]:0;
            };
            constexpr std::uint64_t starts=0x0101010101010101ull,ends=starts<<7;
            for(int y=0;y<8;++y){
                const std::uint64_t cuts=brick.cutRows[y];
                const std::uint64_t around=cuts|((cuts&~ends)<<1)|((cuts&~starts)>>1)|
                    ((neighbourRows(0,y)&ends)>>7)|((neighbourRows(1,y)&starts)<<7)|
                    (y?brick.cutRows[y-1]:neighbourRows(2,7))|
                    (y<7?brick.cutRows[y+1]:neighbourRows(3,0))|
                    (cuts<<8)|(cuts>>8)|(neighbourRows(4,y)>>56)|(neighbourRows(5,y)<<56);
                const auto survivors=around&~(removed?grenadeXRows(*removed,y):0ull);
                for(int z=0;z<8;++z){
                unsigned coarse=unsigned(survivors>>(8*z))&255u;
                if(!coarse)continue;
                if(!inBounds||grid.ownership){
                    for(unsigned todo=coarse;todo;todo&=todo-1){
                        const int x=int(core::lowestBit(todo));
                        const Cell c{base.x+x,base.y+y,base.z+z};
                        if((!inBounds&&!grid.valid(c))||(grid.ownership&&!grid.ownerCuttable(c)))coarse&=~(1u<<x);
                    }
                }
                const unsigned kept=grenadeExpandRow<Ratio>(coarse);
                const unsigned anchors=grenadeExpandRow<Ratio>(coarse&~unsigned(cuts>>(8*z)));
                if(!kept)continue;
                const int nx=nativeOrigin.x+base.x*Ratio,ny=nativeOrigin.y+(base.y+y)*Ratio,nz=nativeOrigin.z+(base.z+z)*Ratio;
                for(int dz=0;dz<Ratio;++dz)for(int dy=0;dy<Ratio;++dy){
                    const unsigned samples=presence.get(ny+dy,nz+dz);
                    const auto row=(z*Ratio+dz)*N+y*Ratio+dy;
                    brick.full[row]=Word(samples&kept);
                    brick.linked[row]=Word(samples&anchors);
                }
            }}
        }
    });
    std::deque<int> queue;
    for(int j=0;j<int(nCut);++j)if(bricks[j].eligible){bricks[j].queued=true;queue.push_back(j);}
    auto across=[&](const Brick& b,int face,int row)->Word {
        const int n=b.next[face];return n<0?0:bricks[n].linked[row];
    };
    while(!queue.empty()) {
        const int id=queue.front();queue.pop_front();auto& b=bricks[id];b.queued=false;
        bool changed=false,iterate=true,forward=true;
        while(iterate) {
            iterate=false;
            for(int k=0;k<Rows;++k) {
                const int j=forward?k:Rows-1-k, y=j%N,z=j/N;
                const unsigned p=b.full[j],prev=b.linked[j];if(prev==p)continue;
                unsigned s=prev;
                s|=(y?b.linked[j-1]:across(b,2,j+N-1));
                s|=(y+1<N?b.linked[j+1]:across(b,3,j-N+1));
                s|=(z?b.linked[j-N]:across(b,4,j+N*(N-1)));
                s|=(z+1<N?b.linked[j+N]:across(b,5,j-N*(N-1)));
                s|=(across(b,0,j)>>(N-1))&1u;
                s|=(unsigned(across(b,1,j))&1u)<<(N-1);
                s&=p;
                if(!s)continue;
                for(;;){const unsigned next=(s|(s<<1)|(s>>1))&p;if(next==s)break;s=next;}
                if(s!=prev){b.linked[j]=Word(s);iterate=true;}
            }
            changed|=iterate;forward=!forward;
        }
        if(changed)for(int next:b.next)
            if(next>=0&&bricks[next].eligible&&!bricks[next].queued){bricks[next].queued=true;queue.push_back(next);}
    }

    for(std::size_t i=0;i<nCut;++i) {
        const auto& b=bricks[i];if(!b.eligible||b.full==b.linked)continue;
        for(int x=0;x<8;++x)for(int y=0;y<8;++y)for(int z=0;z<8;++z) {
            const Cell c{b.key.x*8+x,b.key.y*8+y,b.key.z*8+z};
            if(!core::tileBit(b.candidates,c))continue;
            const unsigned col=((1u<<Ratio)-1u)<<(x*Ratio);
            unsigned present=0,linked=0;
            for(int dz=0;dz<Ratio;++dz)for(int dy=0;dy<Ratio;++dy) {
                const int row=(z*Ratio+dz)*N+y*Ratio+dy;
                present|=b.full[row]&col;linked|=b.linked[row]&col;
            }
            // A coarse removal bit covers all its native subcells. Even one
            // attached subcell prevents removal, including attached padding.
            if(!present||linked||!grid.erase(c))continue;
            // Only this private removed bit feeds the visible floor. Physics has its own receipts.
        }
    }

}

static void resolveGrenadeSupport(Grid& grid,const painter_detail::TissueField* field,
                                  const std::vector<GrenadeStockCut>& cuts,
                                  std::vector<ImpactResult>& impacts) {
    if(cuts.empty()||!field||!field->available||field->pitch<=0)return;
    const double ratio=grid.pitch/field->pitch;const int r=int(std::llround(ratio));
    if(r<1||r>4||std::abs(ratio-r)>.0001)return;
    Cell start{};
    for(int a=0;a<3;++a) {
        const double s=(grid.origin[a]-field->origin[a])/field->pitch;(&start.x)[a]=int(std::llround(s));
        if(std::abs(s-start[a])>.001)return;
    }
    switch(r) {
    case 1:resolveGrenadeSupport<1>(grid,*field,start,cuts,impacts);break;
    case 2:resolveGrenadeSupport<2>(grid,*field,start,cuts,impacts);break;
    case 3:resolveGrenadeSupport<3>(grid,*field,start,cuts,impacts);break;
    case 4:resolveGrenadeSupport<4>(grid,*field,start,cuts,impacts);break;
    }
}
