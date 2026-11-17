#pragma once
// Damage-threshold severing. Geometry only: no health, ammo or death policy.
//
// A limb segment is a Lua joint-profile row (JointRegion: pivot bone, axis to the tip bone, radius). Stations lie
// every 12 mm along the axis from 0.9 R beyond the pivot (the shoulder or hip itself is never a station: torso
// tissue lies inside the radius there) to 0.5 R beyond the tip (the joint at the tip is one). A station is a slab
// 12 mm thick across the segment; at asset preparation it counts its intact anatomy volume (tissue, bone).
// After an accepted cut, the stations whose slab meets the cut's dirty tiles are re-measured from the removed
// cells. A station fails when what is left cannot hold the limb:
//   tear      remaining tissue <= T and remaining bone <= .4
//   dangling  remaining bone <= .25 and remaining tissue <= T + .15 (bone gone, some tissue holds): torn free the
//             same way, reported so the engine tethers the piece at the tear instead of dropping it
//   T = clamp(.45 - .1 (severScale - 1), .25, .6); severScale from the ammo (Impact::sever_scale) or its radius.
// The tear is a seeded ragged surface s = offset(tissue class, u, v) across the station: flap harmonics and
// noise for skin, cellular noise for muscle (bundles pulled out at different lengths), an oblique fracture plane
// per bone label, small noise elsewhere (failure-threshold study 1.5, at what a 1 mm lattice can show). The
// kerf is every piece-side cell within two cells of a stump-side cell (a 2 mm gap the display can render), the
// small solid islands the kerf isolates (shreds), and the anatomical air within two cells of the kerf (the skin
// mesh is cut along the tear, as the tunnel inlet does). The checkerboard repair every cut gets runs on the
// kerf. Connectivity then sees two pieces and the separation pipeline detaches the limb as today.
#include <array>
#include <cstdlib>
#if defined(__AVX__)
#include <immintrin.h>
#endif
#include <atomic>
#include <cmath>
#include <mutex>
#include <set>
#include <unordered_map>
#include <vector>

struct SeverStation {Vec center;double s{};double intactTissue{},intactBone{};Cell tileLo,tileHi;};
struct SeverSegment {std::uint16_t bone{};Vec pivot,axis,u,v;double length{},radius{},halfWidth{.006};std::vector<SeverStation> stations;};
// Per model (Asset::station_masks): the cells of a (station, tile) inside the station's slab and radius - exactly
// measureStationTile's per-cell test - for heavily damaged tiles, shared by every body of the model (a blast measures
// the same stations of all its bodies). Such a measurement is popcounts; its totals are sums[n], the repeated addition
// of the cell volume that the per-cell loop performs, so they are bit-identical. At most maxEntries masks (16 MB).
struct StationMasks {
    explicit StationMasks(double cellVolume):cell(cellVolume){sums[0]=0;for(unsigned n=1;n<=512;++n)sums[n]=sums[n-1]+cellVolume;}
    double cell;double sums[513];
    static constexpr std::size_t shardCount=32,maxEntries=std::size_t(1)<<18;
    struct Shard {std::mutex mutex;std::unordered_map<std::uint64_t,std::array<std::uint64_t,8>> map;};
    std::array<Shard,shardCount> shards;std::atomic<std::size_t> entries{0};
    static std::uint64_t key(std::size_t k,const Cell&t){return (std::uint64_t(k&0xffffu)<<48)|(std::uint64_t(std::uint32_t(t.x)&0xffffu)<<32)|
        (std::uint64_t(std::uint32_t(t.y)&0xffffu)<<16)|std::uint64_t(std::uint32_t(t.z)&0xffffu);}
    Shard& shard(std::uint64_t k){return shards[std::size_t(core::quantMix(k)%shardCount)];}
    bool find(std::uint64_t k,std::array<std::uint64_t,8>&out){auto&s=shard(k);std::lock_guard<std::mutex> l(s.mutex);
        const auto it=s.map.find(k);if(it==s.map.end())return false;out=it->second;return true;}
    void insert(std::uint64_t k,const std::array<std::uint64_t,8>&mask){if(entries.load(std::memory_order_relaxed)>=maxEntries)return;
        auto&s=shard(k);std::lock_guard<std::mutex> l(s.mutex);if(s.map.emplace(k,mask).second)entries.fetch_add(1,std::memory_order_relaxed);}
};
struct SeverState {std::vector<char> failed; // per station (all segments, in order): its tear has been applied
    std::vector<std::array<double,16>> seams; // seam (value 3, SeamTear.h): the seams this grid lineage holds (frame, seed, stretch done, split)
    std::uint32_t decided=0; // mask-only wounds, slices 3 + 4: decide-once tears this grid lineage holds (a piece: its birth tears + its own; limit 8)
    // Incremental measurement (stationRemoved): each station's removed tissue/bone totals and their per-tile shares, so a
    // cut re-measures only the tiles it changed instead of every tile of every station it touched.
    std::vector<char> measured;std::vector<std::uint16_t> epoch;std::vector<double> removedTissue,removedBone;
    // Solid/bone bit masks per 8^3 tile (static anatomy, shared by every copy of this state): no anatomy lookup per cell.
    struct TileMaterial {std::array<std::uint64_t,8> solid{},bone{};};
    std::shared_ptr<std::unordered_map<Cell,TileMaterial,core::CellHash>> material=std::make_shared<std::unordered_map<Cell,TileMaterial,core::CellHash>>();
    std::unordered_map<std::uint64_t,std::pair<double,double>> tileShare; // key: station, its epoch, tile
    // The model's static tissue/bone masks (Asset::tile_material), complete: an absent tile holds no tissue. Null:
    // `material` is filled on demand as before.
    const std::unordered_map<Cell,TileMaterial,core::CellHash>* table=nullptr;
    StationMasks* masks=nullptr; // the model's station-slab masks (Asset::station_masks); null: per-cell tests
    // the shares of each station's current epoch in its own copy-on-write map (key: tile), so a copy of this
    // state copies one pointer per station instead of every share; shareCount is what tileShare.size() would be
    // (old-epoch entries included) so the 1M reset happens at the same moment.
    struct StationShareMap {std::unordered_map<std::uint64_t,std::pair<double,double>> map;};
    std::vector<std::shared_ptr<StationShareMap>> stationShares;std::size_t shareCount=0;
    void invalidate(){measured.assign(measured.size(),0);tileShare.clear();stationShares.assign(stationShares.size(),nullptr);shareCount=0;}
    void ensure(std::size_t n){if(measured.size()!=n){measured.assign(n,0);epoch.assign(n,0);removedTissue.assign(n,0);removedBone.assign(n,0);tileShare.clear();stationShares.assign(n,nullptr);shareCount=0;}}
};
struct SeverVerdict {
    bool tear{},dangling{};std::uint16_t bone{};Vec point,axis;double remainingTissue{1},remainingBone{1};
    std::size_t cells{},slivers{};bool repairIncomplete{};double ms{};double stageMs[8]{};int rounds{};std::set<Cell> dirty;
};
namespace sever_detail {
inline double hash01(std::uint64_t seed,std::uint64_t a,std::uint64_t b,std::uint64_t c){
    const std::uint64_t h=core::quantMix(seed^core::quantMix(a*0x9E3779B97F4A7C15ull^core::quantMix(b*0xC2B2AE3D27D4EB4Full^(c*0x165667B19E3779F9ull))));
    return double(h>>11)*(1./9007199254740992.);}
// Smooth value noise in [-1,1] over a lattice of `cell` metres.
inline double valueNoise(std::uint64_t seed,unsigned salt,double u,double v,double cell){
    const double x=u/cell,y=v/cell;const double fx=std::floor(x),fy=std::floor(y);const double tx=x-fx,ty=y-fy;
    auto at=[&](double ix,double iy){return hash01(seed,salt,std::uint64_t(std::int64_t(ix)+(1<<20)),std::uint64_t(std::int64_t(iy)+(1<<20)))*2-1;};
    auto smooth=[](double t){return t*t*(3-2*t);};const double sx=smooth(tx),sy=smooth(ty);
    const double a=at(fx,fy)+(at(fx+1,fy)-at(fx,fy))*sx,b=at(fx,fy+1)+(at(fx+1,fy+1)-at(fx,fy+1))*sx;return a+(b-a)*sy;}
// Cellular (nearest feature point) noise in [-1,1]: bundles of `cell` metres.
inline double cellNoise(std::uint64_t seed,unsigned salt,double u,double v,double cell){
    const double x=u/cell,y=v/cell;const double fx=std::floor(x),fy=std::floor(y);double best=1e30;
    for(int i=-1;i<=1;++i)for(int j=-1;j<=1;++j){const double cx=fx+i,cy=fy+j;
        const double px=cx+hash01(seed,salt,std::uint64_t(std::int64_t(cx)+(1<<20)),std::uint64_t(std::int64_t(cy)+(1<<20))),py=cy+hash01(seed,salt+1,std::uint64_t(std::int64_t(cx)+(1<<20)),std::uint64_t(std::int64_t(cy)+(1<<20)));
        best=std::min(best,(px-x)*(px-x)+(py-y)*(py-y));}
    return std::clamp(std::sqrt(best)*2-1,-1.,1.);}
// R60: the nearest feature point's own value in [-1,1] (a Voronoi step): bundles of `cell` metres torn at different lengths.
inline double cellStep(std::uint64_t seed,unsigned salt,double u,double v,double cell){
    const double x=u/cell,y=v/cell;const double fx=std::floor(x),fy=std::floor(y);double best=1e30,value=0;
    for(int i=-1;i<=1;++i)for(int j=-1;j<=1;++j){const double cx=fx+i,cy=fy+j;
        const auto ix=std::uint64_t(std::int64_t(cx)+(1<<20)),iy=std::uint64_t(std::int64_t(cy)+(1<<20));
        const double px=cx+hash01(seed,salt,ix,iy),py=cy+hash01(seed,salt+1,ix,iy),d=(px-x)*(px-x)+(py-y)*(py-y);
        if(d<best){best=d;value=hash01(seed,salt+2,ix,iy)*2-1;}}
    return value;}
constexpr double tearBand=.040;
enum:std::uint8_t{C_OTHER=0,C_SKIN=1,C_DEEP=2,C_MUSCLE=3,C_BONE=4,C_ORGAN=5};
enum:std::uint8_t{SOLID=1,PIECE=2,TEAR=4,OPEN=8,AIR=16,CLS_SHIFT=5};
inline int floorDiv(int x,int d){return x>=0?x/d:-((-x+d-1)/d);}
}

// Asset preparation: the stations and their intact volumes from the immutable anatomy lattice.
inline std::vector<SeverSegment> prepareSeverSegments(const Grid&grid,const std::vector<JointRegion>&regions,unsigned bones){
    std::vector<SeverSegment> out;if(regions.empty()||!grid.anatomy||!grid.material_slots)return out;
    const auto&a=*grid.anatomy;const double cell=a.pitch*a.pitch*a.pitch;const Vec ao{a.origin[0],a.origin[1],a.origin[2]};
    for(const auto&region:regions){
        if(region.bone>=bones)continue;const Vec pivot=vec(region.center),axisFull=vec(region.axis);const double L=length(axisFull);
        if(!(L>.02)||!(region.radius>=.01)||!region.sever)continue;
        SeverSegment seg;seg.bone=region.bone;seg.pivot=pivot;seg.axis=axisFull*(1/L);seg.length=L;seg.radius=region.radius;
        const Vec ref=std::abs(seg.axis.z)<.9?Vec{0,0,1}:Vec{1,0,0};seg.u=unit(cross(ref,seg.axis));seg.v=cross(seg.axis,seg.u);
        for(double s=.9*region.radius;s<=L+.5*region.radius;s+=.012){SeverStation st;st.s=s;st.center=pivot+seg.axis*s;
            int lo[3],hi[3];bool empty=false;
            for(int k=0;k<3;++k){const double extent=std::abs(seg.axis[k])*seg.halfWidth+std::sqrt(std::max(0.,1-seg.axis[k]*seg.axis[k]))*seg.radius;
                lo[k]=std::max(0,int(std::floor((st.center[k]-extent-a.origin[k])/a.pitch)));hi[k]=std::min(int(a.size[k])-1,int(std::floor((st.center[k]+extent-a.origin[k])/a.pitch)));
                if(lo[k]>hi[k])empty=true;
                const double glo=st.center[k]-extent-grid.origin[k],ghi=st.center[k]+extent-grid.origin[k];
                (&st.tileLo.x)[k]=sever_detail::floorDiv(int(std::floor(glo/grid.pitch)),8);(&st.tileHi.x)[k]=sever_detail::floorDiv(int(std::floor(ghi/grid.pitch)),8);}
            if(empty)continue;
            for(int x=lo[0];x<=hi[0];++x)for(int y=lo[1];y<=hi[1];++y){const auto column=std::size_t(y)*a.size[0]+x;
                for(auto r=a.offsets[column];r<a.offsets[column+1];++r){const auto&run=a.runs[r];if(run.label>=grid.material_slots->size())continue;
                    const bool bone=(*grid.material_slots)[run.label]==2;
                    const int z0=std::max(lo[2],int(run.lo)),z1=std::min(hi[2],int(run.hi)-1);
                    for(int z=z0;z<=z1;++z){const Vec d=ao+Vec{x+.5,y+.5,z+.5}*a.pitch-st.center;const double ax=dot(d,seg.axis);
                        if(std::abs(ax)>seg.halfWidth||dot(d,d)-ax*ax>seg.radius*seg.radius)continue;st.intactTissue+=cell;if(bone)st.intactBone+=cell;}}}
            if(st.intactTissue<2e-6)continue; // under 2 cm3: nothing to hold
            seg.stations.push_back(st);}
        if(!seg.stations.empty())out.push_back(std::move(seg));}
    return out;
}

// speedfix (2026-11-12): the same table when the damage lattice is not the anatomy's (2 mm cells over the 1 mm anatomy:
// buildTileMaterial refused it, so every user - the station update, the tear's tissue scans, the thread tear, the low-poly
// pieces - fell back to one anatomy lookup per cell, and the station masks were never prepared). Bit = rawMaterialBase(c)
// != 0, bone = its slot is 2, computed with rawMaterialBase's own arithmetic: the cell centre origin + (c + .5) * pitch and
// SparseMaterialVolume::point's index floor((p - origin) / pitch) each depend on one axis, so the anatomy index of every
// grid column / row / layer is computed once (the same expressions) and each anatomy run is mapped to the grid cells whose
// centres fall in it (the index never decreases along an axis). GFY_SF_TABLE=0: no table off the anatomy lattice.
inline std::shared_ptr<const std::unordered_map<Cell,SeverState::TileMaterial,core::CellHash>> buildTileMaterialSampled(const Grid&g){
    static const bool on=[]{const char*e=std::getenv("GFY_SF_TABLE");return !e||std::atoi(e)!=0;}();
    if(!on||!g.anatomy||!g.material_slots||g.ownership||g.size.x<=0||g.size.y<=0||g.size.z<=0)return {};
    const auto&a=*g.anatomy;const auto&slots=*g.material_slots;
    if(!(a.pitch>0)||a.offsets.size()!=std::size_t(a.size[0])*a.size[1]+1)return {};
    std::vector<int> ix(std::size_t(g.size.x)),iy(std::size_t(g.size.y)),iz(std::size_t(g.size.z));
    for(int c=0;c<g.size.x;++c){const double p=g.origin.x+(c+.5)*g.pitch;ix[std::size_t(c)]=int(std::floor((p-a.origin[0])/a.pitch));}
    for(int c=0;c<g.size.y;++c){const double p=g.origin.y+(c+.5)*g.pitch;iy[std::size_t(c)]=int(std::floor((p-a.origin[1])/a.pitch));}
    for(int c=0;c<g.size.z;++c){const double p=g.origin.z+(c+.5)*g.pitch;iz[std::size_t(c)]=int(std::floor((p-a.origin[2])/a.pitch));}
    for(std::size_t c=1;c<iz.size();++c)if(iz[c]<iz[c-1])return {}; // never (monotone arithmetic); the run mapping needs it
    auto map=std::make_shared<std::unordered_map<Cell,SeverState::TileMaterial,core::CellHash>>();
    const int TX=(g.size.x+7)/8,TZ=(g.size.z+7)/8;std::vector<SeverState::TileMaterial> row(std::size_t(TX)*std::size_t(TZ));std::vector<char> used(row.size(),0);
    for(int ty=0;ty*8<g.size.y;++ty){
        for(int cy=ty*8;cy<std::min(g.size.y,ty*8+8);++cy){const int ay=iy[std::size_t(cy)];if(ay<0||std::uint32_t(ay)>=a.size[1])continue;
            for(int cx=0;cx<g.size.x;++cx){const int ax=ix[std::size_t(cx)];if(ax<0||std::uint32_t(ax)>=a.size[0])continue;
                const auto col=std::size_t(ay)*a.size[0]+std::size_t(ax);
                for(auto r=a.offsets[col];r<a.offsets[col+1];++r){const auto&run=a.runs[r];if(!run.label)continue;
                    const int z0=int(std::lower_bound(iz.begin(),iz.end(),int(run.lo))-iz.begin()),z1=int(std::lower_bound(iz.begin(),iz.end(),int(run.hi))-iz.begin());
                    if(z0>=z1)continue;const bool bone=run.label<slots.size()&&slots[run.label]==2;
                    for(int z=z0;z<z1;){const int zEnd=std::min(z1,(z&~7)+8);const unsigned n=unsigned(zEnd-z);
                        const std::uint64_t bits=((std::uint64_t(1)<<n)-1)<<(unsigned((cy&7)*8)+unsigned(z&7));
                        const std::size_t slot=std::size_t(cx>>3)*std::size_t(TZ)+std::size_t(z>>3);auto&m=row[slot];used[slot]=1;
                        m.solid[cx&7]|=bits;if(bone)m.bone[cx&7]|=bits;z=zEnd;}}}}
        for(std::size_t s=0;s<row.size();++s)if(used[s]){(*map)[Cell{int(s/std::size_t(TZ)),ty,int(s%std::size_t(TZ))}]=row[s];row[s]=SeverState::TileMaterial{};used[s]=0;}}
    return map;
}
// The model's tissue/bone bits per 8^3 tile of the damage lattice (SeverState::table), from the anatomy runs. Null when
// the anatomy is not on the damage lattice (another pitch or an unaligned origin): callers keep their lookups.
inline std::shared_ptr<const std::unordered_map<Cell,SeverState::TileMaterial,core::CellHash>> buildTileMaterial(const Grid&g){
    if(!g.anatomy||!g.material_slots||g.ownership)return {};
    const auto&a=*g.anatomy;const auto&slots=*g.material_slots;
    // tablefix (2026-11-11): the Final anatomy stores its pitch rounded to float (0.0010000000474974513 for 1 mm, 4.75e-11
    // off the damage pitch), and the former absolute test (1e-12) refused the table for every Final model, so each user
    // fell back to per-cell anatomy lookups. The lattices are the same cell for cell while the drift over the longest axis
    // stays far below half a cell: the table marks grid cell j+off for anatomy cell j, the per-cell rule samples the
    // anatomy at the grid cell's centre, (j+.5)*g.pitch/a.pitch, and names another cell only when that is half a cell off.
    // Allowed here: a thousandth of a cell over the whole grid (the Final data: 9.0e-5 cell over 1894 cells). Anything
    // more is refused as before (callers keep their lookups).
    {int cells=std::max(g.size.x,std::max(g.size.y,g.size.z));for(const auto n:a.size)cells=std::max(cells,int(n));
     if(!(std::abs(a.pitch-g.pitch)*double(cells+1)<=1e-3*g.pitch))return buildTileMaterialSampled(g);} // speedfix: was {}
    int off[3];for(int k=0;k<3;++k){const double o=(a.origin[k]-g.origin[k])/g.pitch;off[k]=int(std::lround(o));if(std::abs(o-off[k])>1e-6||off[k]<0)return buildTileMaterialSampled(g);} // speedfix: was {}
    // tablefix: no reserve(a.runs.size()) - that was one bucket per anatomy run (16,777,216 buckets, 268 MB, for about
    // 320,000 tiles); the map grows with its content.
    auto map=std::make_shared<std::unordered_map<Cell,SeverState::TileMaterial,core::CellHash>>();
    // tablefix: the tiles of the current row of tiles (8 anatomy rows) are remembered by (x, z) tile, so the map is asked
    // once per tile and row instead of once per run piece (node addresses of an unordered_map stay valid while it grows).
    const int TX=((int(a.size[0])-1+off[0])>>3)+1,TZ=((int(a.size[2])-1+off[2])>>3)+1;int rowY=-1;
    std::vector<SeverState::TileMaterial*> row(std::size_t(TX)*std::size_t(TZ),nullptr);
    for(std::uint32_t y=0;y<a.size[1];++y){if(((int(y)+off[1])>>3)!=rowY){rowY=(int(y)+off[1])>>3;std::fill(row.begin(),row.end(),nullptr);}
     for(std::uint32_t x=0;x<a.size[0];++x){const auto col=std::size_t(y)*a.size[0]+x;
        for(auto r=a.offsets[col];r<a.offsets[col+1];++r){const auto&run=a.runs[r];if(!run.label)continue;
            const bool bone=run.label<slots.size()&&slots[run.label]==2;const int gx=int(x)+off[0],gy=int(y)+off[1];
            for(int az=int(run.lo);az<int(run.hi);){const int gz=az+off[2];const int zEnd=std::min(int(run.hi),az+(8-(gz&7)));
                const unsigned n=unsigned(zEnd-az);const std::uint64_t bits=((n>=64?~std::uint64_t(0):((std::uint64_t(1)<<n)-1)))<<(unsigned((gy&7)*8)+unsigned(gz&7));
                const bool inRow=(gz>>3)<TZ;const std::size_t slot=std::size_t(gx>>3)*std::size_t(TZ)+std::size_t(gz>>3);
                SeverState::TileMaterial*found=inRow?row[slot]:nullptr;
                if(!found){found=&(*map)[Cell{gx>>3,gy>>3,gz>>3}];if(inRow)row[slot]=found;}
                auto&m=*found;m.solid[gx&7]|=bits;if(bone)m.bone[gx&7]|=bits;az=zEnd;}}}}
    return map;
}
// Removed volume of a station's slab from the damage mask (cells with anatomy; air cells of the mask do not count).
inline void measureSeverStation(const Grid&g,const SeverSegment&seg,const SeverStation&st,double&removedTissue,double&removedBone){
    removedTissue=removedBone=0;const double cell=g.pitch*g.pitch*g.pitch;
    for(int tx=st.tileLo.x;tx<=st.tileHi.x;++tx)for(int ty=st.tileLo.y;ty<=st.tileHi.y;++ty)for(int tz=st.tileLo.z;tz<=st.tileHi.z;++tz){
        auto it=g.tiles.find(Cell{tx,ty,tz});if(it==g.tiles.end())continue;
        for(unsigned w=0;w<8;++w){std::uint64_t bits=it->second.removed[w];
            while(bits){const unsigned b=core::lowestBit(bits);bits&=bits-1;const unsigned local=w*64+b;
                const Cell c{tx*8+int(local>>6),ty*8+int((local>>3)&7),tz*8+int(local&7)};
                const Vec d=g.center(c)-st.center;const double ax=dot(d,seg.axis);
                if(std::abs(ax)>seg.halfWidth||dot(d,d)-ax*ax>seg.radius*seg.radius)continue;
                const auto label=g.rawMaterialBase(c);if(!label)continue;
                removedTissue+=cell;if(g.material_slots&&label<g.material_slots->size()&&(*g.material_slots)[label]==2)removedBone+=cell;}}}
}

// speedfix: the cells of (station, tile) inside the station's slab and radius - measureStationTile's per-cell test, the
// same expressions (cell centre, d = centre - station centre, ax = dot(d, axis), dot(d, d) - ax * ax) evaluated four cells
// at a time with AVX (IEEE multiplies and adds in the scalar order, no FMA), so every bit is the scalar test's.
inline std::array<std::uint64_t,8> stationSlabMask(const Grid&g,const SeverSegment&seg,const SeverStation&st,const Cell&tile){
    std::array<std::uint64_t,8> inside{};
#if defined(__AVX__)
    const double r2=seg.radius*seg.radius;
    const __m256d vaz=_mm256_set1_pd(seg.axis.z),vhw=_mm256_set1_pd(seg.halfWidth),vr2=_mm256_set1_pd(r2),half=_mm256_set1_pd(.5),
        vpitch=_mm256_set1_pd(g.pitch),voz=_mm256_set1_pd(g.origin.z),vcz=_mm256_set1_pd(st.center.z),sign=_mm256_set1_pd(-0.);
    __m256d dz[2],dzz[2];
    for(int h=0;h<2;++h){const int z0=tile.z*8+h*4;
        const __m256d cz=_mm256_add_pd(_mm256_set_pd(double(z0+3),double(z0+2),double(z0+1),double(z0)),half);
        dz[h]=_mm256_sub_pd(_mm256_add_pd(voz,_mm256_mul_pd(cz,vpitch)),vcz);dzz[h]=_mm256_mul_pd(dz[h],dz[h]);}
    for(int lx=0;lx<8;++lx){const int cx=tile.x*8+lx;const double dx=(g.origin.x+(cx+.5)*g.pitch)-st.center.x;
        for(int ly=0;ly<8;++ly){const int cy=tile.y*8+ly;const double dy=(g.origin.y+(cy+.5)*g.pitch)-st.center.y;
            const __m256d t=_mm256_set1_pd(dx*seg.axis.x+dy*seg.axis.y),dd=_mm256_set1_pd(dx*dx+dy*dy);unsigned bits=0;
            for(int h=0;h<2;++h){const __m256d ax=_mm256_add_pd(t,_mm256_mul_pd(dz[h],vaz));
                const __m256d rad=_mm256_sub_pd(_mm256_add_pd(dd,dzz[h]),_mm256_mul_pd(ax,ax));
                const __m256d out=_mm256_or_pd(_mm256_cmp_pd(_mm256_andnot_pd(sign,ax),vhw,_CMP_GT_OQ),_mm256_cmp_pd(rad,vr2,_CMP_GT_OQ));
                bits|=unsigned(~_mm256_movemask_pd(out)&15)<<(h*4);}
            inside[std::size_t(lx)]|=std::uint64_t(bits)<<(ly*8);}}
#else
    for(unsigned local=0;local<512;++local){const Cell c{tile.x*8+int(local>>6),tile.y*8+int((local>>3)&7),tile.z*8+int(local&7)};
        const Vec d=g.center(c)-st.center;const double ax=dot(d,seg.axis);
        if(!(std::abs(ax)>seg.halfWidth||dot(d,d)-ax*ax>seg.radius*seg.radius))inside[local/64]|=std::uint64_t(1)<<(local%64);}
#endif
    return inside;
}
inline void measureStationTile(const Grid&g,const SeverSegment&seg,const SeverStation&st,const Cell&tile,
    const SeverState::TileMaterial&m,double&tissue,double&bone,StationMasks*masks=nullptr,std::size_t k=0){
    tissue=bone=0;auto it=g.tiles.find(tile);if(it==g.tiles.end())return;const double cell=g.pitch*g.pitch*g.pitch;
    if(masks&&masks->cell==cell){ // heavily damaged tiles: the cached slab mask of (station k, tile)
        std::array<std::uint64_t,8> hit;unsigned n=0;for(unsigned w=0;w<8;++w){hit[w]=it->second.removed[w]&m.solid[w];n+=::painter_detail::tile_tunnel_detail::popcount(hit[w]);}
        if(n>=16){const auto key=StationMasks::key(k,tile);std::array<std::uint64_t,8> inside;bool have=masks->find(key,inside);
            if(!have&&n>=48){inside=stationSlabMask(g,seg,st,tile); // speedfix: the same bits, four cells at a time
                masks->insert(key,inside);have=true;}
            if(have){unsigned nt=0,nb=0;for(unsigned w=0;w<8;++w){const std::uint64_t in=hit[w]&inside[w];
                    nt+=::painter_detail::tile_tunnel_detail::popcount(in);nb+=::painter_detail::tile_tunnel_detail::popcount(in&m.bone[w]);}
                tissue=masks->sums[nt];bone=masks->sums[nb];return;}}}
    for(unsigned w=0;w<8;++w){std::uint64_t bits=it->second.removed[w]&m.solid[w];
        while(bits){const unsigned b=core::lowestBit(bits);bits&=bits-1;const unsigned local=w*64+b;
            const Cell c{tile.x*8+int(local>>6),tile.y*8+int((local>>3)&7),tile.z*8+int(local&7)};
            const Vec d=g.center(c)-st.center;const double ax=dot(d,seg.axis);
            if(std::abs(ax)>seg.halfWidth||dot(d,d)-ax*ax>seg.radius*seg.radius)continue;
            tissue+=cell;if((m.bone[w]>>b)&1u)bone+=cell;}}
}
inline std::uint64_t stationTileKey(std::size_t k,std::uint16_t epoch,const Cell&t){ // 12-bit station, 16-bit epoch, 3 x 12-bit tile
    return (std::uint64_t(k&0xfffu)<<52)|(std::uint64_t(epoch)<<36)|(std::uint64_t(std::uint32_t(t.x)&0xfffu)<<24)|(std::uint64_t(std::uint32_t(t.y)&0xfffu)<<12)|std::uint64_t(std::uint32_t(t.z)&0xfffu);}
struct StationRef {const SeverSegment*seg;const SeverStation*st;std::size_t k;};
// Removed tissue/bone of every listed station, kept incrementally: a station measured before re-measures only the tiles
// `dirty` changed; one never measured (or invalidated by a tear: new epoch) measures its whole tile box. The tile
// material masks come from the shared cache; stations are measured in parallel and merged in order.
inline void updateStations(const Grid&g,const std::vector<StationRef>&list,SeverState&state,const std::set<Cell>&dirty){
    if(list.empty())return;
    const bool sharded=true;
    if((sharded?state.shareCount:state.tileShare.size())>1000000){state.tileShare.clear();state.measured.assign(state.measured.size(),0);state.stationShares.assign(state.stationShares.size(),nullptr);state.shareCount=0;}
    std::vector<std::vector<Cell>> need(list.size());std::vector<Cell> all;
    std::size_t fresh=0;for(const auto&ref:list)fresh+=!state.measured[ref.k];
    const int TX=(g.size.x+7)/8,TY=(g.size.y+7)/8,TZ=(g.size.z+7)/8;std::vector<std::uint64_t> damaged;
    auto bitOf=[&](int x,int y,int z){return (std::size_t(x)*std::size_t(TY)+std::size_t(y))*std::size_t(TZ)+std::size_t(z);};
    if(fresh>8){damaged.assign((std::size_t(TX)*TY*TZ+63)/64,0);
        for(const auto&it:g.tiles){const Cell&t=it.first;if(t.x<0||t.y<0||t.z<0||t.x>=TX||t.y>=TY||t.z>=TZ)continue;const auto i=bitOf(t.x,t.y,t.z);damaged[i>>6]|=std::uint64_t(1)<<(i&63);}}
    auto present=[&](int x,int y,int z){if(damaged.empty())return g.tiles.count(Cell{x,y,z})!=0;
        if(x<0||y<0||z<0||x>=TX||y>=TY||z>=TZ)return false;const auto i=bitOf(x,y,z);return ((damaged[i>>6]>>(i&63))&1u)!=0;};
    for(std::size_t i=0;i<list.size();++i){const auto&st=*list[i].st;
        if(!state.measured[list[i].k]){for(int tx=st.tileLo.x;tx<=st.tileHi.x;++tx)for(int ty=st.tileLo.y;ty<=st.tileHi.y;++ty)for(int tz=st.tileLo.z;tz<=st.tileHi.z;++tz)
            if(present(tx,ty,tz))need[i].push_back(Cell{tx,ty,tz});}
        else if(std::size_t(st.tileHi.x-st.tileLo.x+1)*std::size_t(st.tileHi.y-st.tileLo.y+1)*std::size_t(st.tileHi.z-st.tileLo.z+1)<dirty.size()){
            for(int tx=st.tileLo.x;tx<=st.tileHi.x;++tx)for(int ty=st.tileLo.y;ty<=st.tileHi.y;++ty)for(int tz=st.tileLo.z;tz<=st.tileHi.z;++tz)if(dirty.count(Cell{tx,ty,tz}))need[i].push_back(Cell{tx,ty,tz});}
        else for(const Cell&t:dirty)if(t.x>=st.tileLo.x&&t.x<=st.tileHi.x&&t.y>=st.tileLo.y&&t.y<=st.tileHi.y&&t.z>=st.tileLo.z&&t.z<=st.tileHi.z)need[i].push_back(t);
        if(!state.table)all.insert(all.end(),need[i].begin(),need[i].end());}
    static const bool updateLog=std::getenv("GFY_TEAR_SELECT_LOG")!=nullptr;const auto measureStart=core::Clock::now();
    auto&cache=*state.material;
    {std::sort(all.begin(),all.end());all.erase(std::unique(all.begin(),all.end()),all.end());
     std::vector<Cell> missing;if(!state.table)for(const Cell&t:all)if(!cache.count(t))missing.push_back(t);
     std::vector<SeverState::TileMaterial> made(missing.size());
     core::parallelChunks(missing.size(),16,[&](std::size_t first,std::size_t last,std::size_t){
        for(std::size_t i=first;i<last;++i){auto&m=made[i];const Cell&t=missing[i];
            for(unsigned local=0;local<512;++local){const Cell c{t.x*8+int(local>>6),t.y*8+int((local>>3)&7),t.z*8+int(local&7)};
                const auto label=g.rawMaterialBase(c);if(!label)continue;m.solid[local/64]|=std::uint64_t(1)<<(local%64);
                if(g.material_slots&&label<g.material_slots->size()&&(*g.material_slots)[label]==2)m.bone[local/64]|=std::uint64_t(1)<<(local%64);}}});
     for(std::size_t i=0;i<missing.size();++i)cache.emplace(missing[i],made[i]);}
    struct Share {Cell tile;double tissue,bone;};std::vector<std::size_t> firstShare(list.size()+1,0);
    for(std::size_t i=0;i<list.size();++i)firstShare[i+1]=firstShare[i]+need[i].size();
    std::vector<Share> shares(firstShare.back());std::vector<std::uint32_t> stationOf(shares.size());
    for(std::size_t i=0;i<list.size();++i)std::fill(stationOf.begin()+std::ptrdiff_t(firstShare[i]),stationOf.begin()+std::ptrdiff_t(firstShare[i+1]),std::uint32_t(i));
    const SeverState::TileMaterial none{};
    {constexpr std::size_t block=64;const std::size_t blocks=(shares.size()+block-1)/block;std::atomic<std::size_t> next{0};
     core::parallelChunks(std::min<std::size_t>(blocks,core::parallelWidth()),1,[&](std::size_t,std::size_t,std::size_t){
      for(std::size_t n;(n=next.fetch_add(1,std::memory_order_relaxed))<blocks;)
       for(std::size_t j=n*block;j<std::min(shares.size(),(n+1)*block);++j){const std::size_t i=stationOf[j];const Cell&t=need[i][j-firstShare[i]];double a,b;
        if(state.table){const auto m=state.table->find(t);measureStationTile(g,*list[i].seg,*list[i].st,t,m==state.table->end()?none:m->second,a,b,state.masks,list[i].k);}
        else{const auto m=cache.find(t);measureStationTile(g,*list[i].seg,*list[i].st,t,m==cache.end()?none:m->second,a,b,state.masks,list[i].k);}
        shares[j]={t,a,b};}});}
    struct Shares {const Share*first,*last;const Share*begin()const{return first;}const Share*end()const{return last;}};
    auto sharesOf=[&](std::size_t i){return Shares{shares.data()+firstShare[i],shares.data()+firstShare[i+1]};};
    const auto applyStart=core::Clock::now();
    if(sharded){if(state.stationShares.size()!=state.measured.size())state.stationShares.resize(state.measured.size());
     for(std::size_t i=0;i<list.size();++i){const std::size_t k=list[i].k;
        if(!state.measured[k]){++state.epoch[k];double a=0,b=0;auto fresh=std::make_shared<SeverState::StationShareMap>();
            for(const auto&s:sharesOf(i))if(s.tissue>0||s.bone>0){fresh->map[stationTileKey(0,0,s.tile)]={s.tissue,s.bone};++state.shareCount;a+=s.tissue;b+=s.bone;}
            state.stationShares[k]=std::move(fresh);state.removedTissue[k]=a;state.removedBone[k]=b;state.measured[k]=1;}
        else{auto&own=state.stationShares[k];if(!own)own=std::make_shared<SeverState::StationShareMap>();else if(own.use_count()>1)own=std::make_shared<SeverState::StationShareMap>(*own);
            auto&m=own->map;
            for(const auto&s:sharesOf(i)){const auto kk=stationTileKey(0,0,s.tile);auto it=m.find(kk);
            const double oa=it==m.end()?0.:it->second.first,ob=it==m.end()?0.:it->second.second;
            state.removedTissue[k]+=s.tissue-oa;state.removedBone[k]+=s.bone-ob;
            if(s.tissue>0||s.bone>0){if(it==m.end()){m.emplace(kk,std::make_pair(s.tissue,s.bone));++state.shareCount;}else it->second={s.tissue,s.bone};}
            else if(it!=m.end()){m.erase(it);--state.shareCount;}}}}}
    else state.tileShare.reserve(state.tileShare.size()+shares.size());
    if(!sharded)for(std::size_t i=0;i<list.size();++i){const std::size_t k=list[i].k;
        if(!state.measured[k]){++state.epoch[k];double a=0,b=0;
            for(const auto&s:sharesOf(i))if(s.tissue>0||s.bone>0){state.tileShare[stationTileKey(k,state.epoch[k],s.tile)]={s.tissue,s.bone};a+=s.tissue;b+=s.bone;}
            state.removedTissue[k]=a;state.removedBone[k]=b;state.measured[k]=1;}
        else for(const auto&s:sharesOf(i)){const auto kk=stationTileKey(k,state.epoch[k],s.tile);auto it=state.tileShare.find(kk);
            const double oa=it==state.tileShare.end()?0.:it->second.first,ob=it==state.tileShare.end()?0.:it->second.second;
            state.removedTissue[k]+=s.tissue-oa;state.removedBone[k]+=s.bone-ob;
            if(s.tissue>0||s.bone>0){if(it==state.tileShare.end())state.tileShare.emplace(kk,std::make_pair(s.tissue,s.bone));else it->second={s.tissue,s.bone};}
            else if(it!=state.tileShare.end())state.tileShare.erase(it);}}
    if(updateLog)std::cerr<<"tear_update stations="<<list.size()<<" shares="<<shares.size()<<" measure_ms="<<core::ms(measureStart)-core::ms(applyStart)
        <<" apply_ms="<<core::ms(applyStart)<<std::endl;
}
inline void stationRemoved(const SeverState&state,std::size_t k,double&tissue,double&bone){
    tissue=std::max(0.,state.removedTissue[k]);bone=std::max(0.,state.removedBone[k]);}

// Selection and application must agree on the existing clipped tear-box budget.
struct SeverBox {int lo[3]{},hi[3]{},nx{},ny{},nz{};std::size_t cells{};};
inline bool feasibleSeverBox(const Grid&g,const SeverSegment&seg,const SeverStation&st,SeverBox&box){
    const double band=sever_detail::tearBand,Rb=seg.radius+.008;const Vec C=st.center,n=seg.axis;
    for(int k=0;k<3;++k){const double extent=std::abs(n[k])*band+std::sqrt(std::max(0.,1-n[k]*n[k]))*Rb;
        box.lo[k]=std::max(0,int(std::floor((C[k]-extent-g.origin[k])/g.pitch)));box.hi[k]=std::min((&g.size.x)[k]-1,int(std::floor((C[k]+extent-g.origin[k])/g.pitch)));
        if(box.lo[k]>box.hi[k])return false;}
    box.nx=box.hi[0]-box.lo[0]+1;box.ny=box.hi[1]-box.lo[1]+1;box.nz=box.hi[2]-box.lo[2]+1;
    box.cells=std::size_t(box.nx)*box.ny*box.nz;
    return box.cells<=64u*1024u*1024u;
}

// The tear at a failed station (see the header comment). Returns the verdict with the tiles it dirtied.
inline SeverVerdict applySever(Grid&g,const SeverSegment&seg,const SeverStation&st,std::uint64_t seed,bool dangling,double rt,double rb){
    using namespace sever_detail;const auto t0=core::Clock::now();
    SeverVerdict v;v.tear=true;v.dangling=dangling;v.bone=seg.bone;v.point=st.center;v.axis=seg.axis;v.remainingTissue=rt;v.remainingBone=rb;
    auto tS=core::Clock::now();auto stage=[&](int i){v.stageMs[i]=core::ms(tS);tS=core::Clock::now();};
    const double band=tearBand,Rb=seg.radius+.008;const Vec C=st.center,n=seg.axis;
    SeverBox box;if(!feasibleSeverBox(g,seg,st,box)){v.tear=false;return v;}
    const auto&lo=box.lo;const auto&hi=box.hi;
    const int nx=box.nx,ny=box.ny,nz=box.nz;const std::size_t N=box.cells;
    std::vector<std::uint8_t> code(N,0);std::vector<char> inSlab(N,0); // inSlab: classified (the slab); other box cells are unknown
    auto at=[&](int x,int y,int z){return (std::size_t(x-lo[0])*ny+std::size_t(y-lo[1]))*nz+std::size_t(z-lo[2]);};
    auto inBox=[&](int x,int y,int z){return x>=lo[0]&&y>=lo[1]&&z>=lo[2]&&x<=hi[0]&&y<=hi[1]&&z<=hi[2];};
    // Seeded shape.
    double fa[3],fp[3];for(int m=0;m<3;++m){fa[m]=(.35+.65*hash01(seed,m,1,901))/(m+1);fp[m]=6.283185307179586*hash01(seed,m,2,902);}
    struct BonePlane {double cu,cv,cs,sn,slope;};std::map<std::uint16_t,BonePlane> planes;
    static const bool flat=std::getenv("GFY_SEVER_FLAT")!=nullptr; // diagnostics: a plane cut instead of the ragged tear
    auto offsetOf=[&](std::uint8_t cls,std::uint16_t label,double u,double w)->double{
        if(flat)return 0.;
        auto flapAt=[&](){const double th=std::atan2(w,u);double flap=0;for(int m=0;m<3;++m)flap+=fa[m]*std::sin((m+1)*th+fp[m]);return flap;};
        switch(cls){
            // R60: centimetre relief - skin flaps to 3.2 cm, muscle bundles (1 cm) torn at different lengths to 2.6 cm,
            // bone broken on an oblique plane with splinters (2.2 cm), organs 1 cm.
            case C_SKIN:return std::clamp(.020*flapAt()+.005*valueNoise(seed,11,u,w,.006),-.032,.032);
            case C_DEEP:return std::clamp(.012*flapAt()+.004*valueNoise(seed,12,u,w,.008),-.020,.020);
            case C_MUSCLE:return std::clamp(.020*cellStep(seed,14,u,w,.010)+.004*valueNoise(seed,15,u,w,.006),-.026,.026);
            case C_BONE:{auto it=planes.find(label);double o=0;if(it!=planes.end())o=it->second.slope*((u-it->second.cu)*it->second.cs+(w-it->second.cv)*it->second.sn);
                return std::clamp(o+.004*cellNoise(seed,16,u,w,.003),-.022,.022);}
            case C_ORGAN:return .010*valueNoise(seed,17,u,w,.012);
            default:return .005*valueNoise(seed,18,u,w,.010);}};
    // Pass 1: classify the slab's cells (solid, class, mask bit); bone cells wait for their fracture planes.
    const auto&slots=*g.material_slots;const auto&an=*g.anatomy;
    std::vector<std::uint32_t> slab; // box indices of the slab's cells
    struct Pending {int x,y,z;std::uint16_t label;double u,w,s;};std::vector<Pending> boneCells;
    std::map<std::uint16_t,std::array<double,3>> centroid;
    std::vector<std::vector<std::uint32_t>> slabParts(core::parallelWidth());std::vector<std::vector<Pending>> boneParts(core::parallelWidth());std::vector<std::map<std::uint16_t,std::array<double,3>>> centroidParts(core::parallelWidth());
    std::vector<char> rem(N,1); // removed flag per box cell, filled for slab cells in pass 1 and kept current by every erase below
    auto classOf=[&](std::uint16_t label)->std::uint8_t{const unsigned slot=label<slots.size()?slots[label]:0;
        switch(slot){case 2:return C_BONE;case 5:return C_MUSCLE;case 1:return C_DEEP;case 4:case 6:return C_OTHER;default:return slot>=3?C_ORGAN:C_OTHER;}};
    auto suv=[&](int x,int y,int z,double&s,double&u,double&w){const Vec d=g.center(Cell{x,y,z})-C;s=dot(d,n);u=dot(d,seg.u);w=dot(d,seg.v);};
    core::parallelChunks(std::size_t(nx),std::max<std::size_t>(8,std::size_t(nx)/4),[&](std::size_t firstX,std::size_t lastX,std::size_t chunk){
    core::TileCache mask(g);auto& slabPart=slabParts[chunk];auto& bonePart=boneParts[chunk];auto& centroidPart=centroidParts[chunk];
    for(int x=lo[0]+int(firstX);x<lo[0]+int(lastX);++x)for(int y=lo[1];y<=hi[1];++y){
        // z range of the slab in this column: axial(z) = base + (z + .5) pitch n.z
        const double base=dot(Vec{g.origin.x+(x+.5)*g.pitch-C.x,g.origin.y+(y+.5)*g.pitch-C.y,g.origin.z-C.z},n);
        int z0=lo[2],z1=hi[2];
        if(std::abs(n.z)>1e-9){double a=(-band-base)/(g.pitch*n.z)-.5,b=(band-base)/(g.pitch*n.z)-.5;if(a>b)std::swap(a,b);z0=std::max(z0,int(std::floor(a)));z1=std::min(z1,int(std::ceil(b)));}
        else if(std::abs(base)>band)continue;
        // Anatomy labels along this column: one run cursor (the damage lattice is a refinement of the anatomy lattice).
        const int ax=int(std::floor((g.origin.x+(x+.5)*g.pitch-an.origin[0])/an.pitch)),ay=int(std::floor((g.origin.y+(y+.5)*g.pitch-an.origin[1])/an.pitch));
        const bool anyRuns=ax>=0&&ay>=0&&std::uint32_t(ax)<an.size[0]&&std::uint32_t(ay)<an.size[1];
        std::size_t rp=0,re=0;if(anyRuns){const auto col=std::size_t(ay)*an.size[0]+ax;rp=an.offsets[col];re=an.offsets[col+1];}
        auto labelAt=[&](int z)->std::uint16_t{if(!anyRuns)return 0;const int az=int(std::floor((g.origin.z+(z+.5)*g.pitch-an.origin[2])/an.pitch));
            if(az<0||std::uint32_t(az)>=an.size[2])return 0;while(rp<re&&int(an.runs[rp].hi)<=az)++rp;return (rp<re&&int(an.runs[rp].lo)<=az)?an.runs[rp].label:std::uint16_t(0);};
        for(int z=z0;z<=z1;++z){const Cell c{x,y,z};const Vec d=g.center(c)-C;const double s=dot(d,n);if(std::abs(s)>band)continue;
            const double u=dot(d,seg.u),w=dot(d,seg.v);if(u*u+w*w>Rb*Rb)continue;
            const auto label=g.ownership?g.rawMaterialBase(c):labelAt(z);auto&cc=code[at(x,y,z)];inSlab[at(x,y,z)]=1;slabPart.push_back(std::uint32_t(at(x,y,z)));
            const bool was=mask.removed(c);rem[at(x,y,z)]=was?1:0;
            if(!label){cc=was?0:AIR;continue;}
            if(was||!g.ownerCuttable(c))continue;
            const std::uint8_t cls=classOf(label);cc=std::uint8_t(SOLID|(cls<<CLS_SHIFT));
            if(cls==C_BONE){bonePart.push_back({x,y,z,label,u,w,s});if(std::abs(s)<=.003){auto&q=centroidPart[label];q[0]+=u;q[1]+=w;q[2]+=1;}}}}});
    {std::size_t total=0;for(const auto& p:slabParts)total+=p.size();slab.reserve(total);
     for(auto& p:slabParts)slab.insert(slab.end(),p.begin(),p.end());
     for(auto& p:boneParts)boneCells.insert(boneCells.end(),p.begin(),p.end());
     for(const auto& p:centroidParts)for(const auto& q:p){auto& c=centroid[q.first];c[0]+=q.second[0];c[1]+=q.second[1];c[2]+=q.second[2];}}
    // Pass 1b: connective within two cells of anatomical air is skin (flaps); then every non-bone solid cell's side.
    auto xyzOf=[&](std::size_t j,int&x,int&y,int&z){z=lo[2]+int(j%nz);y=lo[1]+int((j/nz)%ny);x=lo[0]+int(j/(std::size_t(nz)*ny));};
    // Parallel: each cell writes only its own byte and reads only its neighbours' AIR bits, which this pass never changes.
    // A cell beyond a class's largest offset needs no noise (|offset| <= bound[class]).
    static constexpr double bound[6]={.005,.032,.020,.026,.022,.010}; // OTHER, SKIN, DEEP, MUSCLE, BONE, ORGAN = offsetOf clamps
    core::parallelChunks(slab.size(),std::max<std::size_t>(4096,(slab.size()+3)/4),[&](std::size_t a0,std::size_t b0,std::size_t){
     for(std::size_t si=a0;si<b0;++si){const auto j=slab[si];auto&cc=code[j];if(!(cc&SOLID))continue;int x,y,z;xyzOf(j,x,y,z);
        std::uint8_t cls=std::uint8_t(cc>>CLS_SHIFT);if(cls==C_BONE)continue;
        if(cls==C_DEEP){bool air=false;for(int a=0;a<3&&!air;++a)for(int sgn:{-1,1})for(int dd=1;dd<=2;++dd){int q[3]={x,y,z};q[a]+=sgn*dd;
                if(!inBox(q[0],q[1],q[2])){air=true;break;}if(code[at(q[0],q[1],q[2])]&AIR){air=true;break;}}
            if(air){cls=C_SKIN;cc=std::uint8_t((cc&~(7u<<CLS_SHIFT))|(cls<<CLS_SHIFT));}}
        double s,u,w;suv(x,y,z,s,u,w);const double bd=bound[cls]+1e-9;
        if(s>bd){cc|=PIECE;continue;}if(s<-bd)continue;
        if(s>offsetOf(cls,0,u,w))cc|=PIECE;}});
    for(const auto&q:centroid){if(q.second[2]<20)continue;const double ang=6.283185307179586*hash01(seed,q.first,3,903);
        const double tilt=(35.+20.*hash01(seed,q.first,4,904))*3.141592653589793/180;
        planes[q.first]={q.second[0]/q.second[2],q.second[1]/q.second[2],std::cos(ang),std::sin(ang),std::tan(tilt)*(hash01(seed,q.first,5,905)<.5?-1:1)};}
    for(const auto&b:boneCells){if(b.s>.022+1e-9){code[at(b.x,b.y,b.z)]|=PIECE;continue;}if(b.s<-.022-1e-9)continue;if(b.s>offsetOf(C_BONE,b.label,b.u,b.w))code[at(b.x,b.y,b.z)]|=PIECE;}
    stage(0);
    // Pass 2: kerf = piece cells within two cells of a stump cell (26-neighbourhood, then one more layer).
    auto solid=[&](std::uint8_t c){return (c&SOLID)&&!(c&TEAR);};
    {std::vector<std::size_t> first;
     for(const auto jj:slab){const auto c=code[jj];if(!solid(c)||!(c&PIECE))continue;int x,y,z;xyzOf(jj,x,y,z);
        bool near=false;for(int i=-1;i<=1&&!near;++i)for(int j=-1;j<=1&&!near;++j)for(int k=-1;k<=1&&!near;++k){if(!i&&!j&&!k)continue;
            const int X=x+i,Y=y+j,Z=z+k;if(!inBox(X,Y,Z))continue;const auto q=code[at(X,Y,Z)];if(solid(q)&&!(q&PIECE))near=true;}
        if(near)first.push_back(jj);}
     for(auto i:first)code[i]|=TEAR;}
    {std::vector<std::size_t> second;
     for(const auto jj:slab){const auto c=code[jj];if(!solid(c)||!(c&PIECE))continue;int x,y,z;xyzOf(jj,x,y,z);
        bool near=false;for(int a=0;a<3&&!near;++a)for(int sgn:{-1,1}){int q[3]={x,y,z};q[a]+=sgn;if(inBox(q[0],q[1],q[2])&&(code[at(q[0],q[1],q[2])]&TEAR)){near=true;break;}}
        if(near)second.push_back(jj);}
     for(auto i:second)code[i]|=TEAR;}
    stage(1);
    // Pass 3: shreds = solid islands (6-connected) not reaching the box border, under 1 cm3.
    {std::vector<std::int32_t> comp(N,-1);std::vector<std::size_t> stack,members;const std::size_t minCells=std::size_t(1e-6/(g.pitch*g.pitch*g.pitch));
     for(const auto i:slab){if(comp[i]>=0||!solid(code[i]))continue;members.clear();stack.assign(1,i);comp[i]=0;bool border=false;
        while(!stack.empty()){const std::size_t j=stack.back();stack.pop_back();members.push_back(j);
            const int z=int(j%nz),y=int((j/nz)%ny),x=int(j/(std::size_t(nz)*ny));
            if(x==0||y==0||z==0||x==nx-1||y==ny-1||z==nz-1)border=true;
            for(int a=0;a<3;++a)for(int sgn:{-1,1}){int q[3]={x,y,z};q[a]+=sgn;if(q[0]<0||q[1]<0||q[2]<0||q[0]>=nx||q[1]>=ny||q[2]>=nz)continue;
                const std::size_t k=(std::size_t(q[0])*ny+q[1])*nz+q[2];if(!inSlab[k]){border=true;continue;}
                if(comp[k]<0&&solid(code[k])){comp[k]=0;stack.push_back(k);}}}
        if(!border&&members.size()<minCells)for(auto j:members)code[j]|=TEAR;}}
    stage(2);
    // Pass 4: the air within two cells of the kerf opens (the skin mesh crosses it), and so does a disc of air 1.5 mm
    // either side of the skin tear surface out to the slab radius: a garment around the limb is cut along the tear
    // (the tunnel inlet opens the garment gap the same way). Air-only faces never become walls.
    core::parallelChunks(slab.size(),std::max<std::size_t>(4096,(slab.size()+3)/4),[&](std::size_t a0,std::size_t b0,std::size_t){
     for(std::size_t si=a0;si<b0;++si){const auto jj=slab[si];auto&cc=code[jj];if(cc!=AIR)continue;int x,y,z;xyzOf(jj,x,y,z);
        double s,u,w;suv(x,y,z,s,u,w);if(std::abs(s)>.032+1.5*g.pitch)continue;if(std::abs(s-offsetOf(C_SKIN,0,u,w))<=1.5*g.pitch)cc=std::uint8_t(AIR|OPEN);}});
    {std::vector<std::size_t> cur;for(const auto i:slab)if(code[i]&TEAR)cur.push_back(i);
     for(int layer=0;layer<2&&!cur.empty();++layer){std::vector<std::size_t> next;
        for(auto j:cur){const int z=int(j%nz),y=int((j/nz)%ny),x=int(j/(std::size_t(nz)*ny));
            for(int i=-1;i<=1;++i)for(int jj=-1;jj<=1;++jj)for(int k=-1;k<=1;++k){const int X=x+i,Y=y+jj,Z=z+k;if(X<0||Y<0||Z<0||X>=nx||Y>=ny||Z>=nz)continue;
                auto&q=code[(std::size_t(X)*ny+Y)*nz+Z];if(q==AIR){q=std::uint8_t(AIR|OPEN);next.push_back((std::size_t(X)*ny+Y)*nz+Z);}}}
        cur.swap(next);}}
    stage(3);
    // Commit: erase the kerf (journaled like every cut). Then slivers: a kept cell of the box with removed cells on
    // both sides along an axis, or with five removed face neighbours, is a one-cell sheet or pin between the tear
    // and a bullet channel (two walls back to back: a non-manifold edge for the exact join): removed, twice over.
    // Then the checkerboard repair every cut gets, around everything this tear removed.
    std::vector<Cell> fresh;std::set<Cell> touched;std::size_t solidCells=0;
    {Cell lastKey{INT_MIN,INT_MIN,INT_MIN};
     for(const auto jj:slab){const auto c=code[jj];if(!(c&(TEAR|OPEN)))continue;int x,y,z;xyzOf(jj,x,y,z);
        const Cell cell{x,y,z};if(!g.erase(cell))continue;rem[jj]=1;fresh.push_back(cell);const Cell tk=Grid::tileKey(cell);if(!(tk==lastKey)){touched.insert(tk);lastKey=tk;}if(c&TEAR)++solidCells;}}
    // Checkerboard repair witness: any cell of the slab (tissue or air); the repair removes the kept cell nearest
    // the station centre. Then, on the mask itself, slivers (a kept cell of the slab, tissue or air, with removed
    // cells on both sides along an axis or on five faces) and islands (kept cells, tissue or air, enclosed by
    // removed cells within the slab, under 1 cm3: a closed box of walls with no skin, which the exact join
    // rejects). Repair, slivers and islands alternate until nothing changes (each can make the other).
    auto intersects=[&](Cell c){return inBox(c.x,c.y,c.z)&&inSlab[at(c.x,c.y,c.z)]!=0;};
    std::size_t extraSolid=0;const std::size_t minCells=std::size_t(1e-6/(g.pitch*g.pitch*g.pitch));
    // Cleanup rounds on flat arrays. Round 0 scans the whole slab; a later round only looks where something changed:
    // a kept cell's sliver verdict, or the islands, can only change next to a cell that became removed (erased here or
    // by the repair). Components reached from a candidate stop as soon as they prove big (border, size, or a cell of a
    // component already proven big this round).
    std::vector<std::uint32_t> seen(N,0),bigStamp(N,0);std::vector<std::size_t> stack,members,flipped,cand;
    std::vector<std::uint8_t> edge(N,0);for(const auto jj:slab){int x,y,z;xyzOf(jj,x,y,z);edge[jj]=(x<=lo[0]||y<=lo[1]||z<=lo[2]||x>=hi[0]||y>=hi[1]||z>=hi[2])?1:0;}
    const std::size_t sx=std::size_t(ny)*nz,sy=std::size_t(nz),sz=1; // index strides
    for(const Cell&c:fresh)flipped.push_back(at(c.x,c.y,c.z));
    auto isSliver=[&](std::size_t jj){if(edge[jj]||rem[jj]||!(code[jj]&(SOLID|AIR)))return false;
        const std::size_t nb[6]={jj-sx,jj+sx,jj-sy,jj+sy,jj-sz,jj+sz};bool known=true;int count=0;
        for(int k=0;k<6;++k){known=known&&inSlab[nb[k]];count+=rem[nb[k]];}
        return known&&(count>=5||(rem[nb[0]]&&rem[nb[1]])||(rem[nb[2]]&&rem[nb[3]])||(rem[nb[4]]&&rem[nb[5]]));};
    auto island=[&](std::size_t i,std::uint32_t stamp)->bool{members.clear();stack.assign(1,i);seen[i]=stamp;bool big=false;
        while(!stack.empty()){const std::size_t j=stack.back();stack.pop_back();members.push_back(j);
            if(edge[j]||bigStamp[j]==stamp||members.size()>=minCells){big=true;break;}
            const std::size_t nb[6]={j-sx,j+sx,j-sy,j+sy,j-sz,j+sz};
            for(int k=0;k<6;++k){const std::size_t q=nb[k];if(!inSlab[q]){big=true;continue;}
                if(bigStamp[q]==stamp){big=true;continue;} // joins a component already proven big this round
                if(seen[q]==stamp||!(code[q]&(SOLID|AIR))||rem[q])continue;seen[q]=stamp;stack.push_back(q);}
            if(big)break;}
        if(big){for(auto j:members)bigStamp[j]=stamp;for(auto j:stack)bigStamp[j]=stamp;return false;}return true;};
    stage(4);
    for(int round=0;round<8;++round){
        std::size_t added=0;
        try{added=resolveEdgeContacts(g,fresh,C,intersects,touched,&extraSolid);}catch(const RecoverableImpactError&){v.repairIncomplete=true;added=1;}
        if(added){core::TileCache now(g); // repairs sit within one cell of a fresh cell
            for(const Cell&f:fresh)for(int i=-1;i<=1;++i)for(int j=-1;j<=1;++j)for(int k=-1;k<=1;++k){const Cell q{f.x+i,f.y+j,f.z+k};
                if(!inBox(q.x,q.y,q.z))continue;const auto jj=at(q.x,q.y,q.z);if(!inSlab[jj]||rem[jj])continue;if(now.removed(q)){rem[jj]=1;flipped.push_back(jj);}}}
        fresh.clear();const std::uint32_t stamp=std::uint32_t(round+1);
        auto push=[&](std::size_t jj){int x,y,z;xyzOf(jj,x,y,z);fresh.push_back(Cell{x,y,z});};
        static const bool fullScan=std::getenv("GFY_SEVER_FULLSCAN")!=nullptr;std::size_t sliverHits=0,islandHits=0;
        if(round==0||fullScan){
            for(const auto jj:slab)if(isSliver(jj)){push(jj);++sliverHits;}
            for(const auto i:slab){if(seen[i]==stamp||!(code[i]&(SOLID|AIR))||rem[i])continue;if(island(i,stamp)){++islandHits;for(auto j:members)push(j);}}}
        else{cand.clear();
            for(const auto f:flipped){const std::size_t nb[6]={f-sx,f+sx,f-sy,f+sy,f-sz,f+sz};for(int k=0;k<6;++k){const auto q=nb[k];if(q<N&&inSlab[q]&&!rem[q]&&(code[q]&(SOLID|AIR)))cand.push_back(q);}}
            std::sort(cand.begin(),cand.end());cand.erase(std::unique(cand.begin(),cand.end()),cand.end());
            for(const auto jj:cand)if(isSliver(jj)){push(jj);++sliverHits;}
            for(const auto i:cand){if(seen[i]==stamp||rem[i])continue;if(island(i,stamp)){++islandHits;for(auto j:members)push(j);}}}
#ifdef GFY_COLLIDER_DIAGNOSTICS
        std::cerr<<"tear_round "<<round<<" repair_added="<<added<<" flipped="<<flipped.size()<<" cand="<<cand.size()<<" slivers="<<sliverHits<<" islands="<<islandHits<<" fresh="<<fresh.size()<<std::endl;
#endif
        flipped.clear();if(fresh.empty())break;
        std::vector<Cell> erased;for(const Cell&c:fresh)if(g.erase(c)){erased.push_back(c);touched.insert(Grid::tileKey(c));++v.slivers;const auto jj=at(c.x,c.y,c.z);rem[jj]=1;flipped.push_back(jj);if(code[jj]&SOLID)++solidCells;}
        fresh.swap(erased);v.rounds=round+1;if(fresh.empty())break;}
    stage(5);
    v.cells=solidCells+extraSolid;v.dirty=core::expandDirtyTiles(touched);
    v.ms=core::ms(t0);return v;
}

// After an accepted cut: the stations its dirty tiles touch are re-measured; the weakest failing one tears.
inline SeverVerdict severCheck(Grid&g,const std::vector<SeverSegment>&segments,SeverState&state,const std::set<Cell>&dirty,double severScale,std::uint64_t seed){
    SeverVerdict none;if(segments.empty()||dirty.empty())return none;
    std::size_t total=0;for(const auto&seg:segments)total+=seg.stations.size();
    if(state.failed.size()!=total)state.failed.assign(total,0);
    Cell dlo=*dirty.begin(),dhi=dlo;for(const Cell&t:dirty){dlo={std::min(dlo.x,t.x),std::min(dlo.y,t.y),std::min(dlo.z,t.z)};dhi={std::max(dhi.x,t.x),std::max(dhi.y,t.y),std::max(dhi.z,t.z)};}
    const double T=std::clamp(.45-.1*(severScale-1),.25,.6);
    const SeverSegment*bestSeg=nullptr;const SeverStation*bestSt=nullptr;std::size_t bestIndex=0;bool bestDangling=false;double bestRt=1,bestRb=1,worst=1e30;
    std::size_t index=0;
    for(const auto&seg:segments)for(const auto&st:seg.stations){const std::size_t k=index++;if(state.failed[k])continue;
        if(st.tileLo.x>dhi.x||st.tileHi.x<dlo.x||st.tileLo.y>dhi.y||st.tileHi.y<dlo.y||st.tileLo.z>dhi.z||st.tileHi.z<dlo.z)continue;
        bool touched=false;for(const Cell&t:dirty)if(t.x>=st.tileLo.x&&t.x<=st.tileHi.x&&t.y>=st.tileLo.y&&t.y<=st.tileHi.y&&t.z>=st.tileLo.z&&t.z<=st.tileHi.z){touched=true;break;}
        if(!touched)continue;
        double removedTissue,removedBone;measureSeverStation(g,seg,st,removedTissue,removedBone);
        const double rt=std::clamp(1-removedTissue/st.intactTissue,0.,1.);
        const double rb=st.intactBone>1e-7?std::clamp(1-removedBone/st.intactBone,0.,1.):rt;
        const bool tear=rt<=T&&rb<=.4,dangling=!tear&&rb<=.25&&rt<=T+.15;
        if(!(tear||dangling))continue;
        SeverBox box;if(!feasibleSeverBox(g,seg,st,box))continue;
        const double score=rt+rb;if(score<worst){worst=score;bestSeg=&seg;bestSt=&st;bestIndex=k;bestDangling=dangling;bestRt=rt;bestRb=rb;}}
    if(!bestSeg)return none;
    auto verdict=applySever(g,*bestSeg,*bestSt,seed,bestDangling,bestRt,bestRb);
    // A clipped/budget-rejected tear leaves these stations eligible for a later cut.
    if(verdict.tear){std::size_t k=0;for(const auto&seg:segments)for(const auto&st:seg.stations){if(&seg==bestSeg&&std::abs(st.s-bestSt->s)<=bestSeg->radius)state.failed[k]=1;++k;}}
    return verdict;
}
