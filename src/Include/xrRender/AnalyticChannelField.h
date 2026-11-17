#pragma once
// Immutable CPU payload for one candidate's analytic cavity. The source skin
// removal field and source anatomy field remain separate, mandatory resources.
// This describes analytic cutters, not voxel repairs or severed ownership.
#include <algorithm>
#include <array>
#include <atomic>
#include "RaggedBlade.h"
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>
namespace painter_detail {
// A cut with a role (the worker's ResolvedTunnel) that is a sever's tear: DamageTear.h / InstantSever caps.
template<class T,class=void> struct ChannelHasRole:std::false_type{};
template<class T> struct ChannelHasRole<T,std::void_t<decltype(std::declval<const T&>().role)>>:std::true_type{};
template<class T> inline bool channelTearRole(const T&c){
    if constexpr(ChannelHasRole<T>::value){using R=std::decay_t<decltype(c.role)>;return c.role==R::tear;}else return false;}
struct ChannelIdentity {
    std::array<std::uint8_t,32> source{},rig{};
    std::uint64_t cookie{},lineage{},topology{},generation{},revision{};
    // Immutable candidate HISTORY/token; never DamageSnapshot (which owns the
    // Output/mesh containing this field and would create a reference cycle).
    std::shared_ptr<const void> snapshot;
    bool operator==(const ChannelIdentity& v)const noexcept {
        return source==v.source&&rig==v.rig&&cookie==v.cookie&&lineage==v.lineage&&
            topology==v.topology&&generation==v.generation&&revision==v.revision&&snapshot==v.snapshot;
    }
};
struct ChannelPrimitiveField {
    static constexpr unsigned width=4096;
    unsigned child{},count{};
    std::vector<std::array<std::uint32_t,2>> data;
};
struct AnalyticChannelField {
    static constexpr unsigned schema=1,wordWidth=1024,profileWidth=4096,maxCuts=4096;
    // Only gfy_live::resolved_cut_edge_repairs: the closure coverage proof used the
    // actual repaired removal masks. Sever/owned/sphere/capacity/unwritten are refused.
    static constexpr unsigned renderable_limitations=1u;
    ChannelIdentity identity;
    unsigned cuts{},limitations{};
    // 0=not requested, 1=admitted exact-cell suffix, 2=refused baseline.
    // QA must reject 0/2 for a piece requiring this policy.
    unsigned pieceCellSupportState{};
    std::vector<std::array<float,4>> words; // header(count), followed by9 words/cut
    std::vector<float> profiles; // two128-knot periodic profiles/cut
    std::vector<ChannelPrimitiveField> children;
    std::size_t bytes()const noexcept {
        std::size_t n=words.size()*16+profiles.size()*4;
        for(const auto&c:children)n+=c.data.size()*8;
        return n;
    }
    const ChannelPrimitiveField* child(unsigned id)const noexcept {
        for(const auto&c:children)if(c.child==id)return &c;
        return nullptr;
    }
};
// R71 (2026-10-22): the cut cell grid. The wound-wall shaders test each pixel against cuts (its surface hidden by another cut;
// the tissue 3 mm behind it inside a cut or a torn end) - formerly every cut of the field, per pixel. A cut answers true only
// inside its region: a bore within its greatest radius (flare, swell and cracks at their largest) of its axis over
// [-inlet, penetration]; a slab inside its box; a torn tissue surface (contains nothing) inside its torn-end cylinder. Every
// region's box, grown by a margin far above float rounding, is listed in each cell of the grid it meets, and the shader visits
// only the cuts of the cell it looks in: the same answers as the full loops (it loops over every cut without a grid or for a
// non-finite point). Words from G = header word 0's z (0: no grid): (origin, cell size), (nx, ny, nz, 0), (offset table word,
// cut table word, entries, 1), the nx*ny*nz+1 entry offsets (4 per word; cell (z*ny+y)*nx+x), the cut indices (4 per word,
// ascending in each cell). Any doubt (a non-finite word, a size limit) leaves the field without a grid.
inline bool cutRegionBox(const AnalyticChannelField& f,unsigned id,double lo[3],double hi[3]){
    const auto* w=&f.words[1+std::size_t(id)*9];
    for(unsigned k=0;k<9;++k)for(unsigned c=0;c<4;++c)if(!std::isfinite(w[k][c]))return false;
    bool fine=true; // the shader reads a profile knot past the buffer as 0
    auto knot=[&](double at)->double{if(!(at>=0)||at>=double(f.profiles.size()))return 0.;const float v=f.profiles[std::size_t(at)];
        if(!std::isfinite(v)){fine=false;return 0.;}return std::abs(double(v));};
    const double e[3]={w[0][0],w[0][1],w[0][2]},d[3]={w[1][0],w[1][1],w[1][2]},u[3]={w[2][0],w[2][1],w[2][2]},v[3]={w[3][0],w[3][1],w[3][2]};
    const double mode=w[6][3],offset=std::floor(double(w[6][2]));
    const double tLo=mode==4?0.:-double(w[3][3]),tHi=w[1][3];
    // The shader tests with the stored (float) frame, which is unit and orthogonal only up to rounding: the regions below are bounded
    // for the stored vectors as they are (a frame off by more than 10 % gets no grid - every cut is visited, as before).
    const double dl=std::sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);
    if(!(dl>=.9&&dl<=1.1))return false;
    if(mode==0||mode==4){
        // radial2 = |q|^2 - t^2 <= R^2 with t = q.d: q = a d/|d| + w (w normal to d) has t = |d| a, so a lies in [tLo, tHi]/|d| and
        // |w|^2 <= R^2 + a^2 (|d|^2 - 1); the box spans the axis points e + d t/|d|^2 grown by that radius on every axis.
        double R=std::abs(double(w[0][3]));
        if(mode==0){
            const double flare=w[4][0]>=0?std::abs(double(w[4][1])*(1+double(w[5][1]))-1):0.;
            R*=1+flare+std::abs(double(w[4][2]));
            if(w[6][0]>0)R*=1+std::abs(double(w[5][1]));
        }
        const double T=std::max(std::abs(tLo),std::abs(tHi)),Rw=std::sqrt(R*R+std::max(0.,dl*dl-1)*T*T/(dl*dl));
        for(unsigned k=0;k<3;++k){const double a=e[k]+d[k]*tLo/(dl*dl),b=e[k]+d[k]*tHi/(dl*dl);lo[k]=std::min(a,b)-Rw;hi[k]=std::max(a,b)+Rw;}
    }else{
        const double V=std::abs(double(w[5][3]));double U;
        if(mode==2){double face=0,shear=0;
            for(unsigned st=0;st<9;++st){face=std::max(face,std::max(knot(offset+3*st),knot(offset+3*st+1)));shear=std::max(shear,knot(offset+3*st+2));}
            U=face+shear*V;}
        else if(mode==3)U=std::max(knot(offset+2),knot(offset+3));
        else U=std::abs(double(w[0][3]));
        // (q.d, q.u, q.v) lies in [tLo, tHi] x [-U, U] x [-V, V]: q = M^-1 (t, s, r) with M's rows the stored d, u, v - the box of
        // that parallelepiped exactly (|det M| outside [0.9, 1.1]: no grid).
        const double m[3][3]={{d[0],d[1],d[2]},{u[0],u[1],u[2]},{v[0],v[1],v[2]}};
        const double det=m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
        if(!(std::abs(det)>=.9&&std::abs(det)<=1.1))return false;
        double inv[3][3];
        for(unsigned r=0;r<3;++r)for(unsigned c=0;c<3;++c){
            const unsigned r1=(c+1)%3,r2=(c+2)%3,c1=(r+1)%3,c2=(r+2)%3; // adjugate: inv[r][c] = cofactor(c, r) / det
            inv[r][c]=(m[r1][c1]*m[r2][c2]-m[r1][c2]*m[r2][c1])/det;}
        const double mid=.5*(tLo+tHi),h[3]={.5*std::abs(tHi-tLo),U,V};
        for(unsigned k=0;k<3;++k){const double c=e[k]+inv[k][0]*mid;double r=0;for(unsigned j=0;j<3;++j)r+=std::abs(inv[k][j])*h[j];lo[k]=c-r;hi[k]=c+r;}
    }
    if(!fine)return false;
    for(unsigned k=0;k<3;++k){
        if(!std::isfinite(lo[k])||!std::isfinite(hi[k]))return false;
        const double m=.001+1.e-5*std::max(std::abs(lo[k]),std::abs(hi[k]));lo[k]-=m;hi[k]+=m;
    }
    return true;
}
inline void appendCutCellGrid(AnalyticChannelField& f,std::size_t budget){
    const unsigned n=f.cuts;
    if(!n||f.words.size()!=1+std::size_t(n)*9)return;
    std::vector<double> box(std::size_t(n)*6);
    double glo[3]={1.e300,1.e300,1.e300},ghi[3]={-1.e300,-1.e300,-1.e300};
    for(unsigned i=0;i<n;++i){double* b=&box[std::size_t(i)*6];if(!cutRegionBox(f,i,b,b+3))return;
        for(unsigned k=0;k<3;++k){glo[k]=std::min(glo[k],b[k]);ghi[k]=std::max(ghi[k],b[3+k]);}}
    // About 8k cells (never under 2.5 cm, never over 500 per axis); the shader computes the cell from these floats.
    double ext[3];for(unsigned k=0;k<3;++k)ext[k]=std::max(ghi[k]-glo[k],.001);
    double cell=std::max(.025,std::cbrt(ext[0]*ext[1]*ext[2]/8192.));
    for(unsigned k=0;k<3;++k)cell=std::max(cell,ext[k]/500.);
    const float fo[3]={float(glo[0]),float(glo[1]),float(glo[2])},fc=float(cell);
    if(!std::isfinite(fc)||!(fc>0))return;
    unsigned dims[3];
    for(unsigned k=0;k<3;++k){if(!std::isfinite(fo[k]))return;const double span=(ghi[k]-double(fo[k]))/double(fc);if(!(span>=0&&span<505))return;dims[k]=unsigned(span)+2;}
    const std::size_t cells=std::size_t(dims[0])*dims[1]*dims[2];
    if(cells>(std::size_t(1)<<18))return;
    auto range=[&](const double* b,unsigned k,unsigned& a,unsigned& z){
        const double top=double(dims[k]-1);
        a=unsigned(std::max(0.,std::min(top,std::floor((b[k]-double(fo[k]))/double(fc)))));
        z=unsigned(std::max(0.,std::min(top,std::floor((b[3+k]-double(fo[k]))/double(fc)))));};
    std::vector<std::uint32_t> offset(cells+1,0);std::size_t entries=0;
    for(unsigned pass=0;pass<2;++pass){
        std::vector<std::uint32_t> fill;if(pass){fill.assign(offset.begin(),offset.end()-1);}
        std::vector<std::uint32_t> list;if(pass)list.resize(entries);
        for(unsigned i=0;i<n;++i){const double* b=&box[std::size_t(i)*6];unsigned a[3],z[3];for(unsigned k=0;k<3;++k)range(b,k,a[k],z[k]);
            if(!pass){entries+=std::size_t(z[0]-a[0]+1)*(z[1]-a[1]+1)*(z[2]-a[2]+1);if(entries>(std::size_t(1)<<22))return;}
            for(unsigned zz=a[2];zz<=z[2];++zz)for(unsigned y=a[1];y<=z[1];++y)for(unsigned x=a[0];x<=z[0];++x){
                const std::size_t c=(std::size_t(zz)*dims[1]+y)*dims[0]+x;if(pass)list[fill[c]++]=i;else ++offset[c+1];}}
        if(!pass){for(std::size_t c=0;c<cells;++c)offset[c+1]+=offset[c];continue;}
        const std::size_t G=f.words.size(),cellsBase=G+3,listBase=cellsBase+(cells+1+3)/4,end=listBase+(entries+3)/4;
        // The payload as finish() will pad it (words and profiles to their texture widths; children are padded already): the grid
        // only when that stays within the budget, so a field accepted without the grid is never refused with it.
        auto pad=[](std::size_t v,std::size_t w){return ((v+w-1)/w)*w;};
        std::size_t children=0;for(const auto& c:f.children)children+=c.data.size()*8;
        const std::size_t padded=pad(end,AnalyticChannelField::wordWidth)*16+pad(f.profiles.size(),AnalyticChannelField::profileWidth)*4+children;
        if(end+AnalyticChannelField::wordWidth>=(std::size_t(1)<<24)||padded>budget)return;
        f.words.resize(end,std::array<float,4>{0.f,0.f,0.f,0.f});
        f.words[G]={fo[0],fo[1],fo[2],fc};
        f.words[G+1]={float(dims[0]),float(dims[1]),float(dims[2]),0.f};
        f.words[G+2]={float(cellsBase),float(listBase),float(entries),1.f};
        for(std::size_t c=0;c<=cells;++c)f.words[cellsBase+c/4][c%4]=float(offset[c]);
        for(std::size_t j=0;j<entries;++j)f.words[listBase+j/4][j%4]=float(list[j]);
        f.words[0][2]=float(G);
    }
}
// Immutable true kept-cell membership for analytic walls. Closure labels and
// garment padding never enter this record. Numeric floats transport only exact
// integers (24-bit data chunks); no NaN/denormal bit reinterpretation.
struct PieceCellBrick {std::array<int,3> key{};std::array<std::uint64_t,8> bits{};};
struct PieceCellSupport {
    bool admitted=false,dilated=false;
    std::vector<std::array<int,3>> full;
    std::vector<PieceCellBrick> mixed;
};
// Exactly the union of the raw cells translated by [-1,1]^3. Three separable
// bitwise passes retain the original one-cell Chebyshev policy while the shader
// makes ONE lookup. Full bricks remain compressed. Raw support is not serialized.
// Temporary pages+bit rows are bounded to 96 MiB; raw/final support vectors are
// separately bounded to 32 MiB logical bytes each (vector capacity is additional).
// The supplied cancellation callback must propagate; only allocation refusal
// becomes unadmitted. No runtime publication can reuse a stale parent mask.
template<class Cancel> inline PieceCellSupport dilatePieceCellSupport(const PieceCellSupport&raw,Cancel cancel){
    PieceCellSupport result;if(!raw.admitted||raw.dilated)return result;
    try{
        using Key=std::array<int,3>;using Bits=std::array<std::uint64_t,8>;
        if(raw.full.empty()&&raw.mixed.empty()){result.admitted=result.dilated=true;return result;}
        Key lo{{INT32_MAX,INT32_MAX,INT32_MAX}},hi{{INT32_MIN,INT32_MIN,INT32_MIN}};
        auto bound=[&](const Key&k){for(unsigned a=0;a<3;++a){if(k[a]<0||k[a]>8191)throw std::invalid_argument("piece_cell_raw_range");lo[a]=std::min(lo[a],k[a]-1);hi[a]=std::max(hi[a],k[a]+1);}};
        for(const auto&k:raw.full)bound(k);for(const auto&b:raw.mixed)bound(b.key);
        const std::size_t nx=std::size_t(hi[0]-lo[0]+1),ny=std::size_t(hi[1]-lo[1]+1),nz=std::size_t(hi[2]-lo[2]+1);
        const std::uint64_t count64=std::uint64_t(nx)*ny*nz;if(count64>(1u<<22))return result;
        const std::size_t count=std::size_t(count64),rowsCap=std::min(count,std::size_t(32u<<20)/sizeof(PieceCellBrick));
        constexpr std::size_t temporaryLimit=96u<<20;
        if(count*8>temporaryLimit||rowsCap>(temporaryLimit-count*8)/(2*sizeof(Bits))||raw.mixed.size()>rowsCap)return result;
        std::vector<std::uint32_t> pages(count,0),nextPages(count,0);
        // Fixed capacities avoid unbounded doubling and simultaneous old/new
        // reallocations. Every output mixed row is admitted before its write.
        std::unique_ptr<Bits[]> data(rowsCap?new Bits[rowsCap]:nullptr),nextData(rowsCap?new Bits[rowsCap]:nullptr);
        auto index=[&](const Key&k){return (std::size_t(k[0]-lo[0])*ny+std::size_t(k[1]-lo[1]))*nz+std::size_t(k[2]-lo[2]);};
        auto put=[&](const Key&k,std::uint32_t code){auto&v=pages[index(k)];if(v)throw std::invalid_argument("piece_cell_duplicate_raw");v=code;};
        for(const auto&k:raw.full)put(k,1);
        for(std::size_t n=0;n<raw.mixed.size();++n){put(raw.mixed[n].key,std::uint32_t(n+2));data[n]=raw.mixed[n].bits;}
        const std::size_t strides[3]={ny*nz,nz,1},dims[3]={nx,ny,nz};std::uint64_t work=0;
        auto unit=[&](){if(++work>64000000)return false;if((work&8191u)==0)cancel();return true;};
        auto plane=[&](std::uint32_t code,unsigned x){return code==0?std::uint64_t(0):code==1?~std::uint64_t(0):data[code-2][x];};
        for(unsigned axis=0;axis<3;++axis){
            cancel();std::size_t used=0;
            for(std::size_t p=0;p<count;++p){if(!unit())return PieceCellSupport{};
                const auto mid=pages[p];if(mid==1){nextPages[p]=1;continue;}
                const std::size_t coordinate=(p/strides[axis])%dims[axis];
                const auto before=coordinate?pages[p-strides[axis]]:0,after=coordinate+1<dims[axis]?pages[p+strides[axis]]:0;
                if(!mid&&!before&&!after){nextPages[p]=0;continue;}
                Bits bits{};bool any=false,all=true;
                for(unsigned x=0;x<8;++x){if(!unit())return PieceCellSupport{};const auto w=plane(mid,x);std::uint64_t v=0;
                    if(axis==0)v=w|(x?plane(mid,x-1):plane(before,7))|(x<7?plane(mid,x+1):plane(after,0));
                    else if(axis==1)v=w|(w<<8)|(w>>8)|(plane(before,x)>>56)|((plane(after,x)&0xffu)<<56);
                    else v=w|((w&0x7f7f7f7f7f7f7f7full)<<1)|((w&0xfefefefefefefefeull)>>1)|((plane(before,x)&0x8080808080808080ull)>>7)|((plane(after,x)&0x0101010101010101ull)<<7);
                    bits[x]=v;any|=v!=0;all&=v==~std::uint64_t(0);
                }
                if(!any)nextPages[p]=0;else if(all)nextPages[p]=1;
                else{if(used==rowsCap)return PieceCellSupport{};nextData[used]=bits;nextPages[p]=std::uint32_t(used+2);++used;}
            }
            pages.swap(nextPages);data.swap(nextData);
        }
        std::size_t fullCount=0,mixedCount=0;
        for(auto code:pages){fullCount+=code==1;mixedCount+=code>1;}
        if(fullCount>(32u<<20)/sizeof(Key)||mixedCount>((32u<<20)-fullCount*sizeof(Key))/sizeof(PieceCellBrick))return PieceCellSupport{};
        result.full.reserve(fullCount);result.mixed.reserve(mixedCount);
        for(std::size_t p=0;p<count;++p){if(!unit())return PieceCellSupport{};const auto code=pages[p];if(!code)continue;
            const Key k{{lo[0]+int(p/(ny*nz)),lo[1]+int((p/nz)%ny),lo[2]+int(p%nz)}};
            if(code==1)result.full.push_back(k);else result.mixed.push_back(PieceCellBrick{k,data[code-2]});
        }
        result.admitted=result.dilated=true;return result;
    }catch(const std::bad_alloc&){return PieceCellSupport{};}
}

// Typed executor has joined every body before returning/rethrowing; it owns
// thread policy. This header deliberately knows nothing about core or sep.
// Per piece scratch is exactly the serial helper's fixed pages+two bit arrays.
struct PieceCellDilationStats {std::size_t pages=0;std::uint64_t logicalWork=0;unsigned passes=0;};
template<class Execute,class Cancel> inline PieceCellSupport dilatePieceCellSupport(
    const PieceCellSupport&raw,Execute execute,Cancel cancel,PieceCellDilationStats*stats=nullptr){
    PieceCellSupport result;if(!raw.admitted||raw.dilated)return result;
    try{
        using Key=std::array<int,3>;using Bits=std::array<std::uint64_t,8>;
        if(raw.full.empty()&&raw.mixed.empty()){result.admitted=result.dilated=true;return result;}
        Key lo{{INT32_MAX,INT32_MAX,INT32_MAX}},hi{{INT32_MIN,INT32_MIN,INT32_MIN}};
        auto bound=[&](const Key&k){for(unsigned a=0;a<3;++a){if(k[a]<0||k[a]>8191)throw std::invalid_argument("piece_cell_raw_range");lo[a]=std::min(lo[a],k[a]-1);hi[a]=std::max(hi[a],k[a]+1);}};
        for(const auto&k:raw.full)bound(k);for(const auto&b:raw.mixed)bound(b.key);
        const std::size_t nx=std::size_t(hi[0]-lo[0]+1),ny=std::size_t(hi[1]-lo[1]+1),nz=std::size_t(hi[2]-lo[2]+1);
        const std::uint64_t count64=std::uint64_t(nx)*ny*nz;if(count64>(1u<<22))return result;
        const std::size_t count=std::size_t(count64),rowsCap=std::min(count,std::size_t(32u<<20)/sizeof(PieceCellBrick));
        if(stats)stats->pages=count;
        constexpr std::size_t temporaryLimit=96u<<20;
        if(count*8>temporaryLimit||rowsCap>(temporaryLimit-count*8)/(2*sizeof(Bits))||raw.mixed.size()>rowsCap)return result;
        std::vector<std::uint32_t> pages(count,0),nextPages(count,0);
        std::unique_ptr<Bits[]> data(rowsCap?new Bits[rowsCap]:nullptr),nextData(rowsCap?new Bits[rowsCap]:nullptr);
        auto index=[&](const Key&k){return (std::size_t(k[0]-lo[0])*ny+std::size_t(k[1]-lo[1]))*nz+std::size_t(k[2]-lo[2]);};
        auto put=[&](const Key&k,std::uint32_t code){auto&v=pages[index(k)];if(v)throw std::invalid_argument("piece_cell_duplicate_raw");v=code;};
        for(const auto&k:raw.full)put(k,1);
        for(std::size_t n=0;n<raw.mixed.size();++n){put(raw.mixed[n].key,std::uint32_t(n+2));data[n]=raw.mixed[n].bits;}
        const std::size_t strides[3]={ny*nz,nz,1},dims[3]={nx,ny,nz};
        // Reserve the original final-gather units before the three passes.
        // The extra classification scans are bounded to 3*count reads; they
        // are not original logical work units and allocate no per-page data.
        std::uint64_t work=count;constexpr std::uint64_t workLimit=64000000;
        auto plane=[&](std::uint32_t code,unsigned x){return code==0?std::uint64_t(0):code==1?~std::uint64_t(0):data[code-2][x];};
        for(unsigned axis=0;axis<3;++axis){
            cancel();std::atomic<std::uint64_t> cost{0};
            execute(count,[&](std::size_t first,std::size_t last){
                std::uint64_t local=0;
                for(std::size_t p=first;p<last;++p){
                    if(((p-first)&4095u)==0)cancel();
                    const auto mid=pages[p];++local;if(mid==1)continue;
                    const std::size_t coordinate=(p/strides[axis])%dims[axis];
                    const auto before=coordinate?pages[p-strides[axis]]:0,after=coordinate+1<dims[axis]?pages[p+strides[axis]]:0;
                    if(mid||before||after)local+=8;
                }
                cancel();cost.fetch_add(local,std::memory_order_relaxed);
            });
            const auto passCost=cost.load(std::memory_order_relaxed);
            if(passCost>workLimit-work)return PieceCellSupport{};work+=passCost;
            cancel();std::atomic<std::size_t> used{0};std::atomic<bool> refused{false};
            execute(count,[&](std::size_t first,std::size_t last){
                unsigned pending=0;
                for(std::size_t p=first;p<last;++p){
                    if((pending+=9)>=4096){cancel();pending=0;}
                    if(refused.load(std::memory_order_relaxed))return;
                    const auto mid=pages[p];if(mid==1){nextPages[p]=1;continue;}
                    const std::size_t coordinate=(p/strides[axis])%dims[axis];
                    const auto before=coordinate?pages[p-strides[axis]]:0,after=coordinate+1<dims[axis]?pages[p+strides[axis]]:0;
                    if(!mid&&!before&&!after){nextPages[p]=0;continue;}
                    Bits bits{};bool any=false,all=true;
                    for(unsigned x=0;x<8;++x){const auto w=plane(mid,x);std::uint64_t v=0;
                        if(axis==0)v=w|(x?plane(mid,x-1):plane(before,7))|(x<7?plane(mid,x+1):plane(after,0));
                        else if(axis==1)v=w|(w<<8)|(w>>8)|(plane(before,x)>>56)|((plane(after,x)&0xffu)<<56);
                        else v=w|((w&0x7f7f7f7f7f7f7f7full)<<1)|((w&0xfefefefefefefefeull)>>1)|((plane(before,x)&0x8080808080808080ull)>>7)|((plane(after,x)&0x0101010101010101ull)<<7);
                        bits[x]=v;any|=v!=0;all&=v==~std::uint64_t(0);
                    }
                    if(!any)nextPages[p]=0;else if(all)nextPages[p]=1;
                    else{const auto at=used.fetch_add(1,std::memory_order_relaxed);if(at>=rowsCap){refused.store(true,std::memory_order_relaxed);return;}
                        nextData[at]=bits;nextPages[p]=std::uint32_t(at+2);}
                }
                cancel();
            });
            if(refused.load(std::memory_order_relaxed))return PieceCellSupport{};
            pages.swap(nextPages);data.swap(nextData);
            if(stats){stats->logicalWork=work;stats->passes=axis+1;}
        }
        std::size_t fullCount=0,mixedCount=0;
        for(std::size_t p=0;p<count;++p){if((p&8191u)==0)cancel();const auto code=pages[p];fullCount+=code==1;mixedCount+=code>1;}
        if(fullCount>(32u<<20)/sizeof(Key)||mixedCount>((32u<<20)-fullCount*sizeof(Key))/sizeof(PieceCellBrick))return PieceCellSupport{};
        result.full.reserve(fullCount);result.mixed.reserve(mixedCount);
        // Atomic mixed indices above are temporary references only. Returning
        // rows in exact page order makes the final encoded field byte-identical.
        for(std::size_t p=0;p<count;++p){if((p&8191u)==0)cancel();const auto code=pages[p];if(!code)continue;
            const Key k{{lo[0]+int(p/(ny*nz)),lo[1]+int((p/nz)%ny),lo[2]+int(p%nz)}};
            if(code==1)result.full.push_back(k);else result.mixed.push_back(PieceCellBrick{k,data[code-2]});
        }
        cancel();result.admitted=result.dilated=true;return result;
    }catch(const std::bad_alloc&){return PieceCellSupport{};}
}

inline std::size_t pieceSupportPrefix(const AnalyticChannelField&field){
    if(field.words.empty())throw std::invalid_argument("piece_cell_no_words");
    if(field.words[0][3]==0.f)return field.words.size();const double at=field.words[0][3];
    if(!(std::isfinite(at)&&at==std::floor(at)&&at>=1.+9.*field.cuts&&at+4<field.words.size()))throw std::invalid_argument("piece_cell_prior_offset");
    const auto G=std::size_t(at);const auto&size=field.words[G+1],&storage=field.words[G+3];
    if(size[3]!=3.f||storage[3]!=1.f||!(storage[2]>=G+5&&storage[2]<=field.words.size()&&storage[2]==std::floor(storage[2])))throw std::invalid_argument("piece_cell_prior_header");
    return G;
}
inline void clearPieceCellSupport(AnalyticChannelField&field){const auto prefix=pieceSupportPrefix(field);field.words.resize(prefix);field.words[0][3]=0.f;field.pieceCellSupportState=2;}
// Admission is a transaction: first clear any inherited suffix (no stale parent
// membership survives a refused refresh), then build a separate field candidate.
// A budget/allocation refusal leaves the old analytic rules with NO support mask.
template<class Cancel> inline bool appendPieceCellSupport(AnalyticChannelField&field,const std::array<double,3>&origin,double pitch,
    const PieceCellSupport&support,Cancel cancel,std::size_t budget=32u<<20){
    cancel();
    clearPieceCellSupport(field);if(!support.admitted||!support.dilated)return false;
    try{
        std::uint64_t work=0;
        auto unit=[&](){if(++work>64000000)return false;if((work&8191u)==0)cancel();return true;};
        if(!(std::isfinite(pitch)&&pitch>0))return false;
        std::array<int,3> lo{{INT32_MAX,INT32_MAX,INT32_MAX}},hi{{INT32_MIN,INT32_MIN,INT32_MIN}};
        auto visit=[&](const std::array<int,3>&key){if(!unit())return false;for(unsigned a=0;a<3;++a){if(key[a]<-1||key[a]>8192)throw std::invalid_argument("piece_cell_brick_range");lo[a]=std::min(lo[a],key[a]);hi[a]=std::max(hi[a],key[a]);}return true;};
        for(const auto&key:support.full)if(!visit(key))return false;for(const auto&b:support.mixed)if(!visit(b.key))return false;
        const bool empty=support.full.empty()&&support.mixed.empty();std::array<unsigned,3> dim{};
        std::array<float,4> header{};
        // Cropping never changes the float lattice: raw-stencil and pre-dilated
        // queries use exactly the same origin/pitch, then subtract an integer
        // brick offset. This avoids a new rounding boundary on every refresh.
        for(unsigned a=0;a<3;++a){header[a]=float(origin[a]);dim[a]=empty?0u:unsigned(hi[a]-lo[a]+1);if(!std::isfinite(origin[a])||!std::isfinite(header[a]))return false;}
        header[3]=float(pitch);if(!(std::isfinite(header[3])&&header[3]>0))return false;
        // Encoded-lattice drift stays below 1/16 cell across the whole domain.
        for(unsigned a=0;a<3;++a){const double cells=empty?0.:std::max(std::abs(double(lo[a])*8),std::abs(double(hi[a]+1)*8));
            if(std::abs(double(header[a])-origin[a])+cells*std::abs(double(header[3])-pitch)>pitch/16)return false;}
        const std::uint64_t pages64=std::uint64_t(dim[0])*dim[1]*dim[2];if(pages64>(1u<<22)||support.mixed.size()>=(1u<<24)-2)return false;
        const std::size_t pages=std::size_t(pages64),G=field.words.size(),pageWords=(pages+3)/4,dataFloats=support.mixed.size()*22,dataWords=(dataFloats+3)/4;
        if(G>=(1u<<24)||pageWords>budget/16||dataWords>budget/16)return false;
        const std::size_t pageBase=G+5,dataBase=pageBase+pageWords,end=dataBase+dataWords;
        if(end>=(1u<<24)||end>std::numeric_limits<std::size_t>::max()-field.wordWidth)return false;
        const std::size_t padded=((end+field.wordWidth-1)/field.wordWidth)*field.wordWidth;
        const std::size_t other=field.bytes()-field.words.size()*16;
        if(padded>budget/16||other>budget-padded*16)return false;
        // One bounded allocation, then copy/init in 4096-word quanta. Large
        // baseline payloads never hide a whole vector copy/resize from cancel.
        cancel();std::vector<std::array<float,4>> words;words.reserve(padded);
        for(std::size_t p=0;p<padded;){cancel();const auto endBlock=std::min(padded,p+4096);
            for(;p<endBlock;++p){if(!unit())return false;words.push_back(p<field.words.size()?field.words[p]:std::array<float,4>{});}}
        cancel();
        words[G]=header;words[G+1]={float(dim[0]*8),float(dim[1]*8),float(dim[2]*8),3.f};
        words[G+2]={float(dim[0]),float(dim[1]),float(dim[2]),float(support.mixed.size())};
        words[G+3]={float(pageBase),float(dataBase),float(end),1.f};
        words[G+4]={empty?0.f:float(lo[0]*8),empty?0.f:float(lo[1]*8),empty?0.f:float(lo[2]*8),1.f};
        auto pageOf=[&](const std::array<int,3>&key){return (std::size_t(key[0]-lo[0])*dim[1]+std::size_t(key[1]-lo[1]))*dim[2]+std::size_t(key[2]-lo[2]);};
        auto setPage=[&](std::size_t p,float value){if(!unit())return false;if(p>=pages||words[pageBase+p/4][p%4]!=0.f)throw std::invalid_argument("piece_cell_duplicate_brick");words[pageBase+p/4][p%4]=value;return true;};
        for(const auto&key:support.full)if(!setPage(pageOf(key),1.f))return false;
        for(std::size_t n=0;n<support.mixed.size();++n){if((n&63u)==0)cancel();const auto&brick=support.mixed[n];if(!setPage(pageOf(brick.key),float(n+2)))return false;
            // Extract each exact 24-bit chunk directly, including chunks
            // crossing a uint64 boundary. The last has eight live bits.
            for(unsigned q=0;q<22;++q){if(!unit())return false;const unsigned bit=q*24,w=bit/64,shift=bit%64;
                std::uint64_t value=brick.bits[w]>>shift;if(shift>40&&w+1<8)value|=brick.bits[w+1]<<(64-shift);
                const auto f=n*22+q;words[dataBase+f/4][f%4]=float(std::uint32_t(value&0xffffffu));}}
        cancel();
        words[0][3]=float(G);field.words.swap(words);field.pieceCellSupportState=1;return true;
    }catch(const std::bad_alloc&){return false;}
}
inline bool pieceCellAt(const AnalyticChannelField&field,std::size_t G,int x,int y,int z){
        const auto&size=field.words[G+1],&dim=field.words[G+2],&store=field.words[G+3];const int c[3]={x,y,z};
        if(c[0]<0||c[1]<0||c[2]<0||c[0]>=size[0]||c[1]>=size[1]||c[2]>=size[2])return false;
        const std::size_t page=(std::size_t(c[0]>>3)*std::size_t(dim[1])+std::size_t(c[1]>>3))*std::size_t(dim[2])+std::size_t(c[2]>>3);
        const std::size_t address=std::size_t(store[0])+page/4;if(address>=field.words.size())throw std::invalid_argument("piece_cell_page_storage");
        const auto code=std::uint32_t(field.words[address][page%4]);if(code==0)return false;if(code==1)return true;
        const unsigned local=(unsigned(c[0]&7)*8+unsigned(c[1]&7))*8+unsigned(c[2]&7);
        const std::size_t at=std::size_t(code-2)*22+local/24,data=std::size_t(store[1])+at/4;
        if(code-2>=dim[3]||data>=field.words.size())throw std::invalid_argument("piece_cell_mixed_storage");
        const auto bits=std::uint32_t(field.words[data][at%4]);return (bits&(1u<<(local%24)))!=0;
}
inline bool pieceCellCoordinate(const AnalyticChannelField&field,std::size_t G,double x,double y,double z,int cell[3]){
    const auto&origin=field.words[G],&size=field.words[G+1],&crop=field.words[G+4];const double p[3]={x,y,z};
    for(unsigned a=0;a<3;++a){const double f=std::floor((p[a]-origin[a])/origin[3])-crop[a];
        if(!(std::isfinite(f)&&f>=-1&&f<=size[a]))return false;cell[a]=int(f);}return true;
}
inline bool pieceCellSupports(const AnalyticChannelField&field,double x,double y,double z){
    if(field.words.empty()||field.words[0][3]==0.f)return true;const auto G=pieceSupportPrefix(field);int cell[3];
    return pieceCellCoordinate(field,G,x,y,z,cell)&&pieceCellAt(field,G,cell[0],cell[1],cell[2]);
}

class ChannelFieldBuilder {
    AnalyticChannelField value;
    static float finite(double v){
        const float f=float(v);if(!std::isfinite(v)||!std::isfinite(f))throw std::invalid_argument("channel_nonfinite");return f;
    }
    static std::size_t padded(std::size_t n,unsigned width){return ((n+width-1)/width)*width;}
public:
    explicit ChannelFieldBuilder(ChannelIdentity identity,unsigned limitations=0){
        if(!identity.snapshot)throw std::invalid_argument("channel_snapshot_missing");
        value.identity=std::move(identity);value.limitations=limitations;value.words.resize(1);
    }
    // Capacity for n cuts up front: append's exact-size reserve is then a no-op (no per-cut reallocation).
    void reserveCuts(std::size_t n){value.words.reserve(1+n*9);value.profiles.reserve(n*256);}
    // Resolved is the worker's actual ResolvedTunnel, including exact profile
    // tables. Invalid parameters refuse this candidate instead of sanitizing it.
    template<class Resolved> void append(const Resolved& c){
        if(value.cuts==AnalyticChannelField::maxCuts)throw std::length_error("channel_cut_capacity");
        if(!(c.radius>0&&c.penetration>0&&c.cap>0&&c.cap<=c.penetration&&c.inlet>=0&&c.exit_mult>=1&&
             c.cav_amp>=0&&c.burst_amp>=0&&c.cracks<=8))throw std::invalid_argument("channel_parameter_range");
        if(c.cav_amp>0&&!(c.cav_t1>c.cav_t0))throw std::invalid_argument("channel_cavity_range");
        if(c.cracks&&!(c.exit_distance>=0&&c.burst_len>0&&c.crack_width>0))throw std::invalid_argument("channel_crack_range");
        if(c.slab&&!(c.slab_half_width>0&&c.cracks==0&&c.exit_distance>=0))throw std::invalid_argument("channel_slab_range"); // R24
        auto dot=[](const auto&a,const auto&b){return a.x*b.x+a.y*b.y+a.z*b.z;};
        const auto&d=c.direction;const auto&u=c.edge_u;const auto&v=c.edge_v;
        const double handed=(u.y*v.z-u.z*v.y)*d.x+(u.z*v.x-u.x*v.z)*d.y+(u.x*v.y-u.y*v.x)*d.z;
        if(std::abs(dot(d,d)-1)>1.e-8||std::abs(dot(u,u)-1)>1.e-8||std::abs(dot(v,v)-1)>1.e-8||
           std::abs(dot(d,u))>1.e-8||std::abs(dot(d,v))>1.e-8||std::abs(dot(u,v))>1.e-8||std::abs(handed-1)>1.e-8)
            throw std::invalid_argument("channel_frame_not_right_handed");
        std::array<std::array<float,4>,9> words{};
        auto word=[&](unsigned i,double a,double b,double z,double w){words[i]={finite(a),finite(b),finite(z),finite(w)};};
        word(0,c.entry.x,c.entry.y,c.entry.z,c.radius);word(1,d.x,d.y,d.z,c.penetration);
        word(2,u.x,u.y,u.z,c.cap);word(3,v.x,v.y,v.z,c.inlet);
        word(4,c.exit_distance,c.exit_mult,c.cav_amp,c.cav_t0);word(5,c.cav_t1,c.burst_amp,c.burst_len,c.slab?c.slab_half_width:c.crack_width);
        word(6,c.cracks,(c.edge?1:0)+(channelTearRole(c)&&!c.surface?2:0),value.profiles.size(),c.surface?4:c.slab?(c.slab_seed?3:1):0); // 0 bore, 1 flat slab, 3 torn slab (R60; 2 = former nine stations), 4 torn tissue surface (R62: contains nothing, walls drawn as stored)
        for(unsigned k=0;k<8;k+=4)word(7+k/4,c.crack_angle[k],c.crack_angle[k+1],c.crack_angle[k+2],c.crack_angle[k+3]);
        std::array<float,256> profile{};
        if(c.slab&&c.slab_seed){
            // R60: the torn sheet's parameters in the existing profile block (R62: the explosion blades that made torn slabs are
            // gone - BlastSevers.h - and the shader draws a torn slab flat). Seed halves are exact in float.
            const RaggedBlade blade(c.slab_seed,c.radius,c.slab_half_width,c.penetration);
            profile.fill(0.f);
            profile[0]=float(blade.seed32&0xffffu);profile[1]=float(blade.seed32>>16);
            profile[2]=finite(blade.upperFace);profile[3]=finite(blade.lowerFace);
            profile[4]=float(blade.nt);profile[5]=float(blade.ns);profile[6]=finite(blade.dt);profile[7]=finite(blade.ds);
        }else if(c.edge){
            for(unsigned a=0;a<2;++a){
                if(c.edge->radius[a][128]!=c.edge->radius[a][0])throw std::invalid_argument("channel_profile_periodic");
                for(unsigned k=0;k<128;++k){double p=c.edge->radius[a][k];
                    if(!(p>=.65&&p<=1.))throw std::invalid_argument("channel_profile_range");
                    profile[a*128+k]=finite(p);}
            }
        }else profile.fill(1.f);
        // Validation and allocations complete before counts describe this cut.
        value.words.reserve(value.words.size()+9);value.profiles.reserve(value.profiles.size()+256);
        value.words.insert(value.words.end(),words.begin(),words.end());
        value.profiles.insert(value.profiles.end(),profile.begin(),profile.end());++value.cuts;
    }
    template<class Primitive> void addChild(unsigned slot,const std::vector<Primitive>& primitives){
        if(slot>=1024||primitives.empty()||primitives.size()>65535||value.child(slot))throw std::invalid_argument("channel_child_manifest");
        ChannelPrimitiveField child;child.child=slot;child.count=unsigned(primitives.size());
        child.data.resize(padded(primitives.size(),child.width));
        for(std::size_t i=0;i<primitives.size();++i){
            auto owner=unsigned(primitives[i].owner),kind=unsigned(primitives[i].kind);
            // R62: kinds 3+slot only on a torn tissue surface (mode 4)
            if(owner>=value.cuts||(kind>2&&(kind>66||value.words[1+owner*9+6][3]!=4.f)))throw std::invalid_argument("channel_primitive_owner");
            if(kind==2&&value.words[1+owner*9+4][0]<0)throw std::invalid_argument("channel_closed_tip_cap");
            child.data[i]={owner,kind};
        }
        value.children.push_back(std::move(child));
    }
    std::shared_ptr<const AnalyticChannelField> finish(std::size_t budget=32u<<20){
        if(!value.cuts||value.children.empty())throw std::invalid_argument("channel_empty_field");
        value.words[0]={float(value.cuts),float(AnalyticChannelField::schema),0,0};
        appendCutCellGrid(value,budget); // R71: the cut cell grid (word 0's z), when it fits
        value.words.resize(padded(value.words.size(),value.wordWidth));
        value.profiles.resize(padded(value.profiles.size(),value.profileWidth));
        if(value.bytes()>budget)throw std::length_error("channel_payload_budget");
        return std::make_shared<const AnalyticChannelField>(std::move(value));
    }
};
} // namespace painter_detail
