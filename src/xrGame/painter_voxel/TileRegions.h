#pragma once
// Included inside FastWound_impl.h (namespace gfy_live::fastwound).
//
// Tile-local wound regions. A region = a cluster of removed tiles (tiles within
// 2 of each other) plus its halo E (all six face neighbours of every removed
// tile, and edge/corner neighbours a removed cell touches). The region joins
// ONLY the lattice pieces (FaceLattice.h) of its joined faces J_R (faces within
// one cell of a tile of E) that lie in E. A joined face's remainder -- its
// parts minus the pieces joined by the regions containing it -- is emitted
// STATICALLY as one merged, ear-clipped region per part (PolyTriangulate.h),
// using every canonical crossing next to a joined tile and no other vertex. A
// new hit re-joins only the region it touches.
#include "PolyTriangulate.h"
#include <map>
namespace fastwound {

struct LatticeRow {gfy_packed::Vertex row;std::uint64_t hash{};double discarded{};};
struct LatticeData {FaceLattice lattice;unsigned mask{};};
using LatticeMemo=std::unordered_map<std::uint64_t,std::shared_ptr<const LatticeData>>;
inline std::uint64_t latticeKey(std::uint32_t face,unsigned mask){return (std::uint64_t(face)<<3)|mask;}

// Canonical skin of a corner / original-edge lattice vertex, shared bit for bit
// by every face using it -- only where every face corner at the position(s)
// carries identical skin; an authored skin discontinuity keeps each face's own
// interpolation (as the join does).
inline bool canonicalSkin(const core::Source& s,const AssetData& data,const LatticeVertex& v){
    if(v.weightRule==0)return data.skinConsistent(core::quantExact(s.v[v.lo].p));
    if(v.weightRule==1)return data.skinConsistent(core::quantExact(s.v[v.lo].p))&&data.skinConsistent(core::quantExact(s.v[v.hi].p));
    return false;
}
inline std::array<double,64> latticeWeights(const core::Source& s,const AssetData& data,std::uint32_t face,const LatticeVertex& v){
    if(!canonicalSkin(s,data,v))return core::weights(s,face,v.v);
    if(v.weightRule==0)return data.positionWeights(core::quantExact(s.v[v.lo].p));
    const auto a=data.positionWeights(core::quantExact(s.v[v.lo].p)),b=data.positionWeights(core::quantExact(s.v[v.hi].p));
    std::array<double,64> w{};for(unsigned k=0;k<64;++k)w[k]=a[k]*(1-v.t)+b[k]*v.t;return w;
}
inline Vec floatRounded(Vec p){return Vec{double(float(p.x)),double(float(p.y)),double(float(p.z))};}

// Static remainder of one joined face: rows per vertex, triangles.
struct StaticFace {std::vector<LatticeRow> rows;std::vector<std::array<std::uint32_t,3>> tris;
    std::string packingError,triangulationError;std::size_t dropped{};};
struct TileRegionState {
    const std::vector<char>* bad{}; // AssetData::edgeBad (always-protected edges)
    LatticeSpec spec;
    LatticeMemo lattices;
    std::unordered_map<std::uint32_t,std::uint8_t> mask; // current global protect mask of every joined face
    // Static remainder per joined face for the CURRENT regions, and the memo of
    // the previous hit keyed by (face, mask, hashes of the regions joining it).
    std::unordered_map<std::uint32_t,std::shared_ptr<const StaticFace>> current;
    struct StaticKey {std::uint32_t face;unsigned mask;std::vector<std::uint64_t> regions;bool operator==(const StaticKey& o)const{return face==o.face&&mask==o.mask&&regions==o.regions;}};
    struct StaticKeyHash {std::size_t operator()(const StaticKey& k)const{std::uint64_t h=core::quantMix((std::uint64_t(k.face)<<3)|k.mask);for(auto r:k.regions)h=core::quantMix(h^r);return std::size_t(h);}};
    std::unordered_map<std::uint32_t,StaticKey> staticKeys; // key of every face in `current`
    std::unordered_map<std::uint32_t,std::pair<StaticKey,std::shared_ptr<const StaticFace>>> staticMemo; // a face's previous key and static
    RegionList staticsResults; // the results prepareStatics last saw (kept alive: pointer identity tells new results apart)
    // Tile-local region results of the last candidate that used them: still
    // reusable (pure functions of their identity) after a fallback candidate.
    RegionList lastTileLocal;
    std::unordered_map<std::uint64_t,std::pair<std::vector<Cell>,std::vector<std::uint32_t>>> joinedFaces; // E -> J_R
    // Per-region fallback: tile-local region hashes whose join failed (they go
    // straight to the former path) and the former-path results last used.
    std::unordered_set<std::uint64_t> failedTileLocal;RegionList lastFallback;
    // Chunked regions: tiles of clusters whose chunk join failed; a cluster holding any of them is joined whole.
    std::unordered_set<Cell,core::CellHash> noSplitTiles;
    // Chunks whose own join failed: joined together with every chunk of their 3x3x3 neighbourhood (one box).
    std::unordered_set<Cell,core::CellHash> mergeChunks;
    // Display of a chunked cluster: its reduced chunk meshes welded and reduced once more without seam locks
    // (keyed by cluster, valid while the member results are the same objects).
    struct MergedDisplay {std::vector<const RegionResult*> members;std::shared_ptr<const RegionResult> result;};
    std::unordered_map<std::uint64_t,MergedDisplay> mergedDisplay;
    std::unordered_map<Cell,std::vector<std::uint32_t>,core::CellHash> tileJ; // faces within one cell of a tile
    // findChunkRegions: previous Region per chunk unit id, with its content hash.
    std::unordered_map<std::uint64_t,std::pair<std::uint64_t,std::shared_ptr<const Region>>> chunkRegions;
    std::unordered_map<std::uint64_t,std::pair<std::uint64_t,std::shared_ptr<const Region>>> boxedRegions; // boxed units, rebuilt per call
    // Persistent chunk partition of the grid (findChunkRegions): per tile its mask hash and 27-bit halo, per halo
    // tile the count of tiles reaching it, per chunk its tiles and E. Synced to each grid by the mask hashes, so a
    // hit only updates the chunks around its changed tiles; a clean chunk keeps its content hash and Region.
    struct KeyHash {std::size_t operator()(std::uint64_t k)const{return std::size_t(k);}};
    struct TileRec {std::uint64_t hash{};std::uint32_t halo{},gen{};bool present{};};
    struct ChunkRec {std::vector<Cell> tiles,E;bool dirty{true};};
    struct Partition {core::FlatMap<std::uint64_t,TileRec,KeyHash> tiles;core::FlatMap<std::uint64_t,std::uint32_t,KeyHash> haloCount;
        std::map<std::uint64_t,ChunkRec> chunks;std::uint32_t generation{};std::size_t present{};int chunkTiles{-1};};
    Partition part;
    std::size_t fallbackRegions{}; // this candidate
    // Per-hit statistics (profile builds print them).
    std::size_t staticBuilt{},staticTriangles{},staticDropped{};
};

inline void cellBounds(const std::vector<Cell>& cells,Cell& lo,Cell& hi){lo={1<<30,1<<30,1<<30};hi={-(1<<30),-(1<<30),-(1<<30)};
    for(const auto& c:cells)for(int k=0;k<3;++k){(&lo.x)[k]=std::min((&lo.x)[k],(&c.x)[k]);(&hi.x)[k]=std::max((&hi.x)[k],(&c.x)[k]);}}
// Must the neighbour tile in direction (dx,dy,dz) be joined with this damaged
// tile? The slab subtract splits a piece along the planes of each removed cell
// run ACROSS the whole piece, so cut vertices land on the tile's faces anywhere:
// all six face neighbours, always. Edge / corner neighbours only when a removed
// cell touches them (closed boxes); a piece edge ON a tile edge line (a face
// plane containing it) is the remaining case, and any non-canonical vertex on a
// region seam is rejected by the gate (tileLocalOpenings). Bit ((x*8+y)*8+z):
// word x, bit y*8+z.
inline bool haloTouched(const std::array<std::uint64_t,8>& bits,int dx,int dy,int dz){
    // core::subtract cuts a piece first along the run's x planes (across the
    // whole piece: cut points on the tile's y and z faces), then along y and z
    // planes only inside the run's x (and y) slab: those reach the x (y) faces
    // only where removed cells touch them. So the y/z face neighbours always,
    // the x face neighbours by the touch rule below.
    if(!dx&&std::abs(dy)+std::abs(dz)<=1)return true;
#ifdef GFY_CS_FULL_HALO
    return true;
#endif
    const std::uint64_t zb=dz>0?0x80ull:(dz<0?0x01ull:0xFFull);std::uint64_t m=0;
    for(int y=0;y<8;++y)if(dy==0||(dy>0&&y==7)||(dy<0&&y==0))m|=zb<<(8*y);
    for(int x=0;x<8;++x)if((dx==0||(dx>0&&x==7)||(dx<0&&x==0))&&(bits[std::size_t(x)]&m))return true;
    return false;
}

// Regions, their E and J_R; the global protect masks; lattices of joined faces.
// Chunk edge in tiles (4 tiles = 32 mm at 1 mm cells; 0 disables) and the smallest cluster that is split.
inline int chunkTiles(){static const int v=[]{const char* e=std::getenv("GFY_CHUNK_TILES");return e?std::max(0,std::atoi(e)):4;}();return v;}
inline int chunkMinTiles(){static const int v=[]{const char* e=std::getenv("GFY_CHUNK_MIN_TILES");return e?std::max(1,std::atoi(e)):48;}();return v;}
inline std::vector<Region> findTileRegions(const Grid& grid,const core::BodyBVH& tree,const AssetData& data,TileFaceMemo& memo,
                                           TileRegionState& state,const LatticeSpec& spec){
    const auto& clean=data.clean;state.bad=&data.edgeBad;state.spec=spec;core::StageProfile stages("tiles");
    std::vector<Cell> keys;keys.reserve(grid.tiles.size());for(const auto& t:grid.tiles)keys.push_back(t.first);
    std::sort(keys.begin(),keys.end());
    std::vector<int> parent(keys.size());for(int i=0;i<int(parent.size());++i)parent[i]=i;
    auto root=[&](int i){while(parent[i]!=i){parent[i]=parent[parent[i]];i=parent[i];}return i;};
    auto unite=[&](int a,int b){a=root(a);b=root(b);if(a!=b)parent[std::max(a,b)]=std::min(a,b);};
    // Halos (tile +-1) of two regions may not meet: unite tiles within 2 tiles.
    // Dense index over the tiles' bounds (+-2), else a hash map.
    {Cell lo,hi;cellBounds(keys,lo,hi);const std::int64_t sx=hi.x-lo.x+5,sy=hi.y-lo.y+5,sz=hi.z-lo.z+5;
     if(!keys.empty()&&sx*sy*sz<=(std::int64_t(1)<<24)){std::vector<int> dense(std::size_t(sx*sy*sz),-1);
        auto at=[&](int x,int y,int z){return std::size_t(((std::int64_t(x-lo.x+2)*sy)+(y-lo.y+2))*sz+(z-lo.z+2));};
        for(int i=0;i<int(keys.size());++i)dense[at(keys[i].x,keys[i].y,keys[i].z)]=i;
        for(int i=0;i<int(keys.size());++i)for(int dx=-2;dx<=2;++dx)for(int dy=-2;dy<=2;++dy)for(int dz=-2;dz<=2;++dz){
            if(dx<0||(dx==0&&(dy<0||(dy==0&&dz<=0))))continue;const int j=dense[at(keys[i].x+dx,keys[i].y+dy,keys[i].z+dz)];if(j>=0)unite(i,j);}}
     else{core::FlatMap<Cell,int,core::CellHash> index;index.reserve(keys.size());for(int i=0;i<int(keys.size());++i)index.emplace(keys[i],i);
        for(int i=0;i<int(keys.size());++i)for(int dx=-2;dx<=2;++dx)for(int dy=-2;dy<=2;++dy)for(int dz=-2;dz<=2;++dz){
            if(dx<0||(dx==0&&(dy<0||(dy==0&&dz<=0))))continue;auto f=index.find(Cell{keys[i].x+dx,keys[i].y+dy,keys[i].z+dz});if(f!=index.end())unite(i,f->second);}}}
    std::map<int,std::vector<int>> members;for(int i=0;i<int(keys.size());++i)members[root(i)].push_back(i);
    stages.mark("unite");
    // Chunked regions (bounded per-hit work): a cluster reaching into more than one chunk of chunkTiles^3
    // tiles is joined as one region per chunk, its halo E partitioned by chunk. A hit then re-joins only
    // the chunks whose tiles changed; the others are reused from the region cache. Chunk seams: see
    // mergeBoundaryRects (unit seam edges) and computeRegion (seam openings, fixed points, seam skin).
    struct Unit {std::vector<int> ids;std::vector<Cell> E;bool chunked{},boxed{};Cell chunk{},chunkHi{};std::uint64_t cluster{};};
    std::vector<Unit> units;units.reserve(members.size());
    const int CT=chunkTiles();
    auto chunkOf=[CT](Cell c){auto f=[CT](int x){return x>=0?x/CT:-((-x+CT-1)/CT);};return Cell{f(c.x),f(c.y),f(c.z)};};
    for(auto& m:members){core::FlatMap<Cell,char,core::CellHash> expanded;
        for(int i:m.second){const Cell c=keys[i];const auto& bits=grid.tiles.at(c).removed;
            for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)for(int dz=-1;dz<=1;++dz)if(haloTouched(bits,dx,dy,dz))expanded.try_emplace(Cell{c.x+dx,c.y+dy,c.z+dz},0);}
        std::vector<Cell> list;list.reserve(expanded.size());for(const auto& e:expanded)list.push_back(e.first);std::sort(list.begin(),list.end());
        bool split=CT>0&&int(m.second.size())>=chunkMinTiles();
        if(split){bool blocked=false;for(int i:m.second)if(state.noSplitTiles.count(keys[i])){blocked=true;break;}split=!blocked;}
        if(split){const Cell first=chunkOf(list.front());bool multi=false;for(const Cell c:list){const Cell k=chunkOf(c);if(!(k==first)){multi=true;break;}}split=multi;}
        if(!split){Unit u;u.ids=m.second;u.E=std::move(list);units.push_back(std::move(u));continue;}
        std::map<Cell,Unit> byChunk;
        const std::uint64_t clusterId=core::quantMix(0xC1u^(std::uint64_t(std::uint32_t(keys[m.second.front()].x))|(std::uint64_t(std::uint32_t(keys[m.second.front()].y))<<21)|(std::uint64_t(std::uint32_t(keys[m.second.front()].z))<<42)));
        for(int i:m.second)byChunk[chunkOf(keys[i])].ids.push_back(i);
        for(const Cell c:list)byChunk[chunkOf(c)].E.push_back(c);
        // Failed chunks join as boxes of chunks: every chunk of the cluster within the box [f-1,f+1]^3 of a
        // failed chunk f, boxes that touch merged into their bounding box (repeated until stable).
        std::vector<std::pair<Cell,Cell>> boxes;
        for(const auto& item:byChunk)if(state.mergeChunks.count(item.first))boxes.push_back({Cell{item.first.x-1,item.first.y-1,item.first.z-1},Cell{item.first.x+1,item.first.y+1,item.first.z+1}});
        for(bool changed=true;changed;){changed=false;
            for(std::size_t a=0;a<boxes.size()&&!changed;++a)for(std::size_t b=a+1;b<boxes.size()&&!changed;++b){auto& A=boxes[a];const auto& B=boxes[b];
                if(A.first.x<=B.second.x+1&&B.first.x<=A.second.x+1&&A.first.y<=B.second.y+1&&B.first.y<=A.second.y+1&&A.first.z<=B.second.z+1&&B.first.z<=A.second.z+1){
                    A.first={std::min(A.first.x,B.first.x),std::min(A.first.y,B.first.y),std::min(A.first.z,B.first.z)};
                    A.second={std::max(A.second.x,B.second.x),std::max(A.second.y,B.second.y),std::max(A.second.z,B.second.z)};
                    boxes.erase(boxes.begin()+std::ptrdiff_t(b));changed=true;}}}
        auto boxOf=[&](Cell k)->int{for(std::size_t b=0;b<boxes.size();++b){const auto& B=boxes[b];if(k.x>=B.first.x&&k.x<=B.second.x&&k.y>=B.first.y&&k.y<=B.second.y&&k.z>=B.first.z&&k.z<=B.second.z)return int(b);}return -1;};
        std::vector<Unit> merged(boxes.size());
        for(std::size_t b=0;b<boxes.size();++b){merged[b].chunked=true;merged[b].boxed=true;merged[b].chunk=boxes[b].first;merged[b].chunkHi=boxes[b].second;merged[b].cluster=clusterId;}
        for(auto& item:byChunk){const int b=boxOf(item.first);
            if(b<0){item.second.chunked=true;item.second.chunk=item.first;item.second.chunkHi=item.first;item.second.cluster=clusterId;units.push_back(std::move(item.second));continue;}
            auto& u=merged[std::size_t(b)];u.ids.insert(u.ids.end(),item.second.ids.begin(),item.second.ids.end());u.E.insert(u.E.end(),item.second.E.begin(),item.second.E.end());}
        for(auto& u:merged)if(!u.E.empty()){std::sort(u.E.begin(),u.E.end());units.push_back(std::move(u));}}
    std::vector<std::vector<Cell>> expandedTiles;expandedTiles.reserve(units.size());
    for(auto& u:units)expandedTiles.push_back(u.E);
    // Joined faces J_R: every face within one cell of a tile of E. Then any
    // source edge within one cell of E has all its faces in J_R, so no strip
    // protected because its neighbour is outside the global J (width <= pitch/4)
    // can meet E: a region's join never depends on the protect masks, which
    // other regions change (verified in tileLocalFragments).
    std::vector<std::vector<std::uint32_t>> joined;joined.reserve(members.size());
    std::vector<char> inJ(tree.source.f.size(),0);
    // J_R: faces whose triangle overlaps (tile box +- 1 cell) of some tile of E,
    // found face-first (one BVH query per region, first overlapping tile wins),
    // memoized by the region's E.
    // Cache lookups in order, the misses computed concurrently (independent regions), then filled in order.
    std::vector<std::uint64_t> jkeys(expandedTiles.size());std::vector<const std::vector<std::uint32_t>*> hitList(expandedTiles.size(),nullptr);
    std::vector<std::size_t> misses;
    for(std::size_t u=0;u<expandedTiles.size();++u){const auto& tiles=expandedTiles[u];
        std::uint64_t key=0x7A11E5ull;for(const Cell c:tiles)key=core::quantMix(key^(std::uint64_t(std::uint32_t(c.x))|(std::uint64_t(std::uint32_t(c.y))<<21)|(std::uint64_t(std::uint32_t(c.z))<<42)));
        jkeys[u]=key;auto cached=state.joinedFaces.find(key);
        if(cached!=state.joinedFaces.end()&&cached->second.first==tiles)hitList[u]=&cached->second.second;else misses.push_back(u);}
    std::vector<std::vector<std::uint32_t>> computed(misses.size());
    // Faces within one cell of each tile, memoized per tile (a pure function of the tile and the asset).
    std::mutex tileMemoMutex;
    auto tileFacesJ=[&](Cell t)->std::vector<std::uint32_t>{
        {std::lock_guard<std::mutex> lock(tileMemoMutex);auto it=state.tileJ.find(t);if(it!=state.tileJ.end())return it->second;}
        core::Bounds box;box.lo=grid.origin+Vec{double(t.x*8-1),double(t.y*8-1),double(t.z*8-1)}*grid.pitch;box.hi=grid.origin+Vec{double(t.x*8+9),double(t.y*8+9),double(t.z*8+9)}*grid.pitch;
        std::vector<std::uint32_t> candidates,list;tree.query(0,box,candidates);std::sort(candidates.begin(),candidates.end());
        for(auto f:candidates){const auto& ids=tree.source.f[f];if(triangleBoxOverlap(tree.source.v[ids[0]].p,tree.source.v[ids[1]].p,tree.source.v[ids[2]].p,box.lo,box.hi))list.push_back(f);}
        std::lock_guard<std::mutex> lock(tileMemoMutex);if(state.tileJ.size()>=65536)state.tileJ.clear();return state.tileJ.try_emplace(t,std::move(list)).first->second;};
    auto facesOf=[&](const std::vector<Cell>& tiles,std::vector<std::uint32_t>& list){
        for(const Cell c:tiles){const auto part=tileFacesJ(c);list.insert(list.end(),part.begin(),part.end());}
        std::sort(list.begin(),list.end());list.erase(std::unique(list.begin(),list.end()),list.end());return;
        core::FlatMap<Cell,char,core::CellHash> inE;inE.reserve(tiles.size());for(const Cell c:tiles)inE.try_emplace(c,0);
        Cell lo,hi;cellBounds(tiles,lo,hi);
        core::Bounds box;box.lo=grid.origin+Vec{double(lo.x*8-1),double(lo.y*8-1),double(lo.z*8-1)}*grid.pitch;box.hi=grid.origin+Vec{double(hi.x*8+9),double(hi.y*8+9),double(hi.z*8+9)}*grid.pitch;
        std::vector<std::uint32_t> candidates;tree.query(0,box,candidates);std::sort(candidates.begin(),candidates.end());
        for(std::size_t n=0;n<candidates.size();++n){const auto& ids=tree.source.f[candidates[n]];const Vec A=tree.source.v[ids[0]].p,B=tree.source.v[ids[1]].p,C=tree.source.v[ids[2]].p;
            Vec mn=A,mx=A;for(const Vec& q:{B,C})for(int k=0;k<3;++k){mn[k]=std::min(mn[k],q[k]);mx[k]=std::max(mx[k],q[k]);}
            const Cell t0=tileOfPoint(spec,mn-Vec{grid.pitch,grid.pitch,grid.pitch}),t1=tileOfPoint(spec,mx+Vec{grid.pitch,grid.pitch,grid.pitch});bool hit=false;
            for(int x=t0.x;x<=t1.x&&!hit;++x)for(int y=t0.y;y<=t1.y&&!hit;++y)for(int z=t0.z;z<=t1.z&&!hit;++z){if(!inE.count(Cell{x,y,z}))continue;
                const Vec blo=grid.origin+Vec{double(x*8-1),double(y*8-1),double(z*8-1)}*grid.pitch,bhi=grid.origin+Vec{double(x*8+9),double(y*8+9),double(z*8+9)}*grid.pitch;
                if(triangleBoxOverlap(A,B,C,blo,bhi))hit=true;}
            if(hit)list.push_back(candidates[n]);}};
    {std::atomic<std::size_t> cursor{0};
     core::parallelChunks(misses.size()>=4?core::parallelWidth():1,1,[&](std::size_t,std::size_t,std::size_t){for(std::size_t k;(k=cursor++)<misses.size();)facesOf(expandedTiles[misses[k]],computed[k]);});}

    {std::size_t k=0;
     for(std::size_t u=0;u<expandedTiles.size();++u){
        if(hitList[u]){for(auto f:*hitList[u])inJ[f]=1;joined.push_back(*hitList[u]);continue;}
        auto& list=computed[k++];for(auto f:list)inJ[f]=1;state.joinedFaces[jkeys[u]]={expandedTiles[u],list};joined.push_back(std::move(list));}}
    if(state.joinedFaces.size()>=16384){std::unordered_set<std::uint64_t> used(jkeys.begin(),jkeys.end());
        for(auto it=state.joinedFaces.begin();it!=state.joinedFaces.end();)it=used.count(it->first)?std::next(it):state.joinedFaces.erase(it);}
    stages.mark("joined_faces");
    // Lattice mask of a joined face. A face without a bad edge is ONE part (the
    // whole triangle) for every mask: an edge shared with a face outside J is
    // more than one cell from every joined tile, so no kept crossing (see
    // keepCrossing) and no joined piece ever touches it -- it stays whole
    // without a strip. Only faces with an always-protected (bad) edge use the
    // strip decomposition, whose masks change pieces outside E only.
    auto protectMask=[&](std::uint32_t f){unsigned m=0;bool anyBad=false;for(int e=0;e<3;++e)anyBad=anyBad||data.edgeBad[3*f+e];if(!anyBad)return 0u;
        for(int e=0;e<3;++e){const auto id=3*f+e;bool split=!data.edgeBad[id];
            for(auto n=data.edgeNeighbourFirst[id];n<data.edgeNeighbourFirst[id+1]&&split;++n)split=inJ[data.edgeNeighbours[n]]!=0;
            if(!split)m|=1u<<e;}return m;};
    std::vector<Region> out;std::size_t r=0;
    for(auto& unit:units){auto ids=unit.ids;std::sort(ids.begin(),ids.end());Region region;region.tileLocal=true;
        std::uint64_t h=0x51ED2701ull;auto mix=[&](std::uint64_t v){h=core::quantMix(h^v);};
        if(unit.chunked){region.chunked=true;region.boxed=unit.boxed;region.cluster=unit.cluster;const int t[3]={unit.chunk.x,unit.chunk.y,unit.chunk.z},th[3]={unit.chunkHi.x,unit.chunkHi.y,unit.chunkHi.z};
            for(int a=0;a<3;++a){region.seamLo[a]=t[a]*CT*8;region.seamHi[a]=(th[a]+1)*CT*8;}
            mix(unit.boxed?0xB0C5ull:0xC4C4ull);mix(std::uint64_t(std::uint32_t(unit.chunk.x))|(std::uint64_t(std::uint32_t(unit.chunk.y))<<32));mix(std::uint64_t(std::uint32_t(unit.chunk.z)));
            mix(std::uint64_t(std::uint32_t(unit.chunkHi.x))|(std::uint64_t(std::uint32_t(unit.chunkHi.y))<<32));mix(std::uint64_t(std::uint32_t(unit.chunkHi.z)));
            for(const Cell c:unit.E)mix(std::uint64_t(std::uint32_t(c.x))^(std::uint64_t(std::uint32_t(c.y))<<21)^(std::uint64_t(std::uint32_t(c.z))<<42));}
        for(int i:ids){const Cell c=keys[i];region.keys.push_back(c);const core::Tile& tile=grid.tiles.at(c);region.masks.push_back(tile);
            mix(std::uint64_t(std::uint32_t(c.x))|(std::uint64_t(std::uint32_t(c.y))<<32));mix(std::uint64_t(std::uint32_t(c.z)));for(auto word:tile.removed)mix(word);}
        region.expanded=std::move(expandedTiles[r]);
        region.faces=std::move(joined[r++]);region.selected=region.faces;
        // Identity: tiles, masks and joined faces (E follows from tiles+masks).
        // Protect bits are NOT part of it (see above).
        for(auto f:region.faces){const unsigned pm=protectMask(f);region.protect.push_back(std::uint8_t(pm));region.relevant.push_back(0);mix(0x9E3779B97F4A7C15ull^(std::uint64_t(f)<<3));}
        region.hash=h;out.push_back(std::move(region));}
    stages.mark("regions");
    std::size_t built=0;state.mask.clear();
    for(std::uint32_t f=0;f<inJ.size();++f)if(inJ[f]){const unsigned pm=protectMask(f);state.mask[f]=std::uint8_t(pm);const auto k=latticeKey(f,pm);
        if(!state.lattices.count(k)){auto d=std::make_shared<LatticeData>();d->mask=pm;d->lattice=buildFaceLattice(tree.source,f,spec,pm);state.lattices.emplace(k,std::move(d));++built;}}
    stages.mark("lattices");
    return out;
}

// Chunk regions without clustering (direct display): removed tiles and their halos are partitioned by chunk
// (halos of different regions are disjoint by construction). A chunk whose tiles, masks and halo are unchanged
// reuses its previous Region; only the protect bits are refreshed from the global joined-face set. Work per hit
// is proportional to the damage, plus a cheap pass over all tiles.
inline std::vector<Region> findChunkRegions(const Grid& grid,const core::BodyBVH& tree,const AssetData& data,TileRegionState& state,const LatticeSpec& spec){
    state.bad=&data.edgeBad;state.spec=spec;core::StageProfile stages("tiles");
    const int CT=chunkTiles();
    auto floorDiv=[](int x,int d){return x>=0?x/d:-((-x+d-1)/d);};
    auto chunkOf=[&](Cell c){return Cell{floorDiv(c.x,CT),floorDiv(c.y,CT),floorDiv(c.z,CT)};};
    // 64-bit keys (21 bits per axis, biased) sort in the same order as Cell's (x,y,z) comparison.
    auto enc=[](Cell c){auto u=[](int v){return std::uint64_t(std::uint32_t(v+(1<<20)))&0x1FFFFFull;};return (u(c.x)<<42)|(u(c.y)<<21)|u(c.z);};
    auto dec=[](std::uint64_t k){auto s=[](std::uint64_t v){return int(v&0x1FFFFFull)-(1<<20);};return Cell{s(k>>42),s(k>>21),s(k)};};
    auto mixc=[](std::uint64_t& h,Cell c){h=core::quantMix(h^(std::uint64_t(std::uint32_t(c.x))|(std::uint64_t(std::uint32_t(c.y))<<21)|(std::uint64_t(std::uint32_t(c.z))<<42)));};
    auto unitId=[&](bool boxedUnit,Cell lo,Cell hi){std::uint64_t id=core::quantMix(boxedUnit?0xB0C5ull:0xC4C4ull);mixc(id,lo);mixc(id,hi);return id;};
    // Sync the persistent partition (TileRegionState::Partition) to this grid: a tile whose mask hash changed (or a
    // new / vanished tile) updates the halo counts, its chunk's lists and marks its chunk dirty, plus the chunks of
    // its six face neighbours (their walls read this tile's facing cell layer).
    auto& P=state.part;if(P.chunkTiles!=CT){P=TileRegionState::Partition();P.chunkTiles=CT;state.chunkRegions.clear();}
    const std::uint32_t gen=++P.generation;
    auto maskHash=[](const core::Tile& t){std::uint64_t h=0x7E11ull;for(auto w:t.removed)h=core::quantMix(h^w);return h;};
    auto haloBits=[](const core::Tile& t){
#ifdef GFY_CS_FULL_HALO
        return (1u<<27)-1;
#else
        const auto& r=t.removed;std::uint64_t any=0;for(auto w:r)any|=w;
        const std::uint64_t planes[3]={r[0],any,r[7]};std::uint32_t b=0;unsigned k=0;
        for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy){
            const std::uint64_t rows=planes[dx+1]&(dy<0?0xFFull:dy>0?0xFF00000000000000ull:~0ull);
            for(int dz=-1;dz<=1;++dz,++k)if((!dx&&std::abs(dy)+std::abs(dz)<=1)||
                (rows&(dz<0?0x0101010101010101ull:dz>0?0x8080808080808080ull:~0ull)))b|=1u<<k;
        }return b;
#endif
    };
    auto markChunk=[&](Cell ch){auto it=P.chunks.find(enc(ch));if(it!=P.chunks.end())it->second.dirty=true;};
    auto sortedInsert=[](std::vector<Cell>& v,Cell c){auto it=std::lower_bound(v.begin(),v.end(),c);if(it==v.end()||!(*it==c))v.insert(it,c);};
    auto sortedErase=[](std::vector<Cell>& v,Cell c){auto it=std::lower_bound(v.begin(),v.end(),c);if(it!=v.end()&&*it==c)v.erase(it);};
    auto haloAdd=[&](Cell nb){if(P.haloCount[enc(nb)]++==0){auto& r=P.chunks[enc(chunkOf(nb))];sortedInsert(r.E,nb);r.dirty=true;}};
    auto haloRemove=[&](Cell nb){auto it=P.haloCount.find(enc(nb));if(it==P.haloCount.end()||it->second==0)return;
        if(--it->second==0){auto cit=P.chunks.find(enc(chunkOf(nb)));if(cit!=P.chunks.end()){sortedErase(cit->second.E,nb);cit->second.dirty=true;}}};
    auto rehalo=[&](Cell c,std::uint32_t was,std::uint32_t now){if(was==now)return;int k=0;
        for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)for(int dz=-1;dz<=1;++dz,++k){const bool o=((was>>k)&1u)!=0,n=((now>>k)&1u)!=0;if(o==n)continue;
            const Cell nb{c.x+dx,c.y+dy,c.z+dz};if(n)haloAdd(nb);else haloRemove(nb);}};
    auto touch=[&](Cell c){const Cell ch=chunkOf(c);markChunk(ch);static const int dd[6][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
        for(int d=0;d<6;++d){const Cell n2=chunkOf(Cell{c.x+dd[d][0],c.y+dd[d][1],c.z+dd[d][2]});if(!(n2==ch))markChunk(n2);}};
    auto add=[&](Cell c,TileRegionState::TileRec& rec,const core::Tile& t,std::uint64_t h){rec.hash=h;rec.halo=haloBits(t);rec.gen=gen;rec.present=true;++P.present;
        auto& r=P.chunks[enc(chunkOf(c))];sortedInsert(r.tiles,c);r.dirty=true;rehalo(c,0,rec.halo);touch(c);};
    auto drop=[&](Cell c,TileRegionState::TileRec& rec){rehalo(c,rec.halo,0);rec.halo=0;rec.hash=0;rec.present=false;--P.present;
        auto cit=P.chunks.find(enc(chunkOf(c)));if(cit!=P.chunks.end()){sortedErase(cit->second.tiles,c);cit->second.dirty=true;}touch(c);};
    P.tiles.reserve(grid.tiles.size());
    for(const auto& t:grid.tiles){const Cell c=t.first;const std::uint64_t h=maskHash(t.second);auto& rec=P.tiles[enc(c)];
        if(!rec.present){add(c,rec,t.second,h);continue;}
        rec.gen=gen;if(rec.hash==h)continue;
        const std::uint32_t hb=haloBits(t.second);rehalo(c,rec.halo,hb);rec.hash=h;rec.halo=hb;touch(c);}
    // Tiles gone since the last call (a candidate off an older grid): their records go.
    if(P.present!=grid.tiles.size())for(auto& kv:P.tiles)if(kv.second.present&&kv.second.gen!=gen)drop(dec(kv.first),kv.second);
    for(auto it=P.chunks.begin();it!=P.chunks.end();)if(it->second.tiles.empty()&&it->second.E.empty()){state.chunkRegions.erase(unitId(false,dec(it->first),dec(it->first)));it=P.chunks.erase(it);}else ++it;
    stages.mark("partition");
    // Failure boxes (state.mergeChunks): every chunk within [f-1,f+1]^3 of a failed chunk, touching boxes merged.
    std::vector<std::pair<Cell,Cell>> boxes;
    for(const auto& kv:P.chunks){const Cell ch=dec(kv.first);if(state.mergeChunks.count(ch))boxes.push_back({Cell{ch.x-1,ch.y-1,ch.z-1},Cell{ch.x+1,ch.y+1,ch.z+1}});}
    for(bool changed=true;changed;){changed=false;
        for(std::size_t a=0;a<boxes.size()&&!changed;++a)for(std::size_t b=a+1;b<boxes.size()&&!changed;++b){auto& A=boxes[a];const auto& B=boxes[b];
            if(A.first.x<=B.second.x+1&&B.first.x<=A.second.x+1&&A.first.y<=B.second.y+1&&B.first.y<=A.second.y+1&&A.first.z<=B.second.z+1&&B.first.z<=A.second.z+1){
                A.first={std::min(A.first.x,B.first.x),std::min(A.first.y,B.first.y),std::min(A.first.z,B.first.z)};
                A.second={std::max(A.second.x,B.second.x),std::max(A.second.y,B.second.y),std::max(A.second.z,B.second.z)};
                boxes.erase(boxes.begin()+std::ptrdiff_t(b));changed=true;}}}
    auto boxOf=[&](Cell k)->int{for(std::size_t b=0;b<boxes.size();++b){const auto& B=boxes[b];if(k.x>=B.first.x&&k.x<=B.second.x&&k.y>=B.first.y&&k.y<=B.second.y&&k.z>=B.first.z&&k.z<=B.second.z)return int(b);}return -1;};
    struct Unit {std::vector<Cell> tiles,E;Cell lo,hi;bool boxed{};};
    std::vector<Unit> boxed(boxes.size());
    for(std::size_t b=0;b<boxes.size();++b){boxed[b].boxed=true;boxed[b].lo=boxes[b].first;boxed[b].hi=boxes[b].second;}
    std::vector<Region> out;out.reserve(P.chunks.size()+boxes.size());std::vector<char> inJ(tree.source.f.size(),0);
    auto contentOf=[&](const Unit& u,std::uint64_t id){
        std::uint64_t content=id;for(const Cell c:u.E)mixc(content,c);
        for(const Cell c:u.tiles){mixc(content,c);for(auto word:grid.tiles.at(c).removed)content=core::quantMix(content^word);}
        // A tile's wall faces depend on its six face neighbours (tileBoundary); a neighbour in another unit that changes
        // changes this unit's walls on the seam, so out-of-unit face neighbours are content too.
        for(const Cell c:u.tiles){static const int dd[6][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
            for(int d=0;d<6;++d){const Cell nb{c.x+dd[d][0],c.y+dd[d][1],c.z+dd[d][2]};const Cell ch=chunkOf(nb);
                if(ch.x>=u.lo.x&&ch.x<=u.hi.x&&ch.y>=u.lo.y&&ch.y<=u.hi.y&&ch.z>=u.lo.z&&ch.z<=u.hi.z)continue;
                auto it=grid.tiles.find(nb);if(it==grid.tiles.end()){content=core::quantMix(content^0xA11u);continue;}
                // Only the neighbour's cell layer facing this tile can change this tile's faces (bit = (y*8+z), word = x).
                const auto& r=it->second.removed;std::uint64_t layer=0;
                switch(d){case 0:layer=r[0];break;case 1:layer=r[7];break;
                    case 2:for(int x=0;x<8;++x)layer|=(r[x]&0xFFull)<<(8*x);break;
                    case 3:for(int x=0;x<8;++x)layer|=((r[x]>>56)&0xFFull)<<(8*x);break;
                    case 4:for(int x=0;x<8;++x)layer=core::quantMix(layer^(r[x]&0x0101010101010101ull));break;
                    default:for(int x=0;x<8;++x)layer=core::quantMix(layer^(r[x]&0x8080808080808080ull));break;}
                content=core::quantMix(content^layer^(0x51ull<<d));}}
        return content;};
    auto buildRegion=[&](const Unit& u,std::uint64_t content){
        Region region;region.tileLocal=true;region.chunked=true;region.boxed=u.boxed;region.cluster=0;
        for(int a=0;a<3;++a){region.seamLo[a]=(&u.lo.x)[a]*CT*8;region.seamHi[a]=((&u.hi.x)[a]+1)*CT*8;}
        region.keys=u.tiles;region.masks.reserve(u.tiles.size());for(const Cell c:u.tiles)region.masks.push_back(grid.tiles.at(c));
        region.expanded=u.E;
        for(const Cell t:u.E){auto it=state.tileJ.find(t);
            if(it==state.tileJ.end()){core::Bounds box;box.lo=grid.origin+Vec{double(t.x*8-1),double(t.y*8-1),double(t.z*8-1)}*grid.pitch;box.hi=grid.origin+Vec{double(t.x*8+9),double(t.y*8+9),double(t.z*8+9)}*grid.pitch;
                std::vector<std::uint32_t> candidates,list;tree.query(0,box,candidates);std::sort(candidates.begin(),candidates.end());
                for(auto f:candidates){const auto& ids=tree.source.f[f];if(triangleBoxOverlap(tree.source.v[ids[0]].p,tree.source.v[ids[1]].p,tree.source.v[ids[2]].p,box.lo,box.hi))list.push_back(f);}
                if(state.tileJ.size()>=65536)state.tileJ.clear();it=state.tileJ.emplace(t,std::move(list)).first;}
            region.faces.insert(region.faces.end(),it->second.begin(),it->second.end());}
        std::sort(region.faces.begin(),region.faces.end());region.faces.erase(std::unique(region.faces.begin(),region.faces.end()),region.faces.end());
        region.selected=region.faces;region.protect.assign(region.faces.size(),0);region.relevant.assign(region.faces.size(),0);
        std::uint64_t h=content;for(auto f:region.faces)h=core::quantMix(h^(0x9E3779B97F4A7C15ull^(std::uint64_t(f)<<3)));region.hash=h;
        return region;};
    auto reuse=[&](const std::shared_ptr<const Region>& r){for(auto f:r->faces)inJ[f]=1;out.push_back(*r);};
    // Faces within one cell of the tiles the dirty units join, where the memo lacks them: queried concurrently (a
    // pure function of the tile and the asset), then memoized in list order.
    {std::vector<Cell> want;core::FlatMap<Cell,char,core::CellHash> seen;
     for(const auto& kv:P.chunks){if(!kv.second.dirty)continue;for(const Cell t:kv.second.E)if(!state.tileJ.count(t)&&seen.try_emplace(t,0).second)want.push_back(t);}
     if(want.size()>=8){std::vector<std::vector<std::uint32_t>> lists(want.size());std::atomic<std::size_t> cursor{0};
      core::parallelChunks(std::min<std::size_t>(want.size(),core::parallelWidth()),1,[&](std::size_t,std::size_t,std::size_t){for(std::size_t k;(k=cursor++)<want.size();){const Cell t=want[k];
        core::Bounds box;box.lo=grid.origin+Vec{double(t.x*8-1),double(t.y*8-1),double(t.z*8-1)}*grid.pitch;box.hi=grid.origin+Vec{double(t.x*8+9),double(t.y*8+9),double(t.z*8+9)}*grid.pitch;
        std::vector<std::uint32_t> candidates;tree.query(0,box,candidates);std::sort(candidates.begin(),candidates.end());auto& list=lists[k];
        for(auto f:candidates){const auto& ids=tree.source.f[f];if(triangleBoxOverlap(tree.source.v[ids[0]].p,tree.source.v[ids[1]].p,tree.source.v[ids[2]].p,box.lo,box.hi))list.push_back(f);}}});
      for(std::size_t k=0;k<want.size();++k){if(state.tileJ.size()>=65536)state.tileJ.clear();state.tileJ.emplace(want[k],std::move(lists[k]));}}}
    // Standalone chunks in chunk order (a clean chunk keeps its Region; a dirty one re-hashes and keeps it when the
    // content is unchanged), then the boxed units, rebuilt from their chunks every call.
    for(auto& kv:P.chunks){const Cell ch=dec(kv.first);auto& rec=kv.second;const int b=boxOf(ch);
        if(b>=0){auto& u=boxed[std::size_t(b)];u.tiles.insert(u.tiles.end(),rec.tiles.begin(),rec.tiles.end());u.E.insert(u.E.end(),rec.E.begin(),rec.E.end());continue;}
        const std::uint64_t id=unitId(false,ch,ch);auto cached=state.chunkRegions.find(id);
        if(!rec.dirty&&cached!=state.chunkRegions.end()){reuse(cached->second.second);continue;}
        Unit u;u.tiles=rec.tiles;u.E=rec.E;u.lo=u.hi=ch;const std::uint64_t content=contentOf(u,id);rec.dirty=false;
        if(cached!=state.chunkRegions.end()&&cached->second.first==content){reuse(cached->second.second);continue;}
        auto shared=std::make_shared<const Region>(buildRegion(u,content));state.chunkRegions[id]={content,shared};reuse(shared);}
    std::unordered_map<std::uint64_t,std::pair<std::uint64_t,std::shared_ptr<const Region>>> nextBoxed;
    for(auto& u:boxed){if(u.E.empty())continue;std::sort(u.tiles.begin(),u.tiles.end());std::sort(u.E.begin(),u.E.end());
        const std::uint64_t id=unitId(true,u.lo,u.hi),content=contentOf(u,id);auto cached=state.boxedRegions.find(id);
        if(cached!=state.boxedRegions.end()&&cached->second.first==content){reuse(cached->second.second);nextBoxed[id]=cached->second;continue;}
        auto shared=std::make_shared<const Region>(buildRegion(u,content));nextBoxed[id]={content,shared};reuse(shared);}
    state.boxedRegions=std::move(nextBoxed);
    stages.mark("regions");
    // Protect bits from the global joined-face set (faces with an always-protected edge only).
    auto protectMask=[&](std::uint32_t f){unsigned m=0;bool anyBad=false;for(int e=0;e<3;++e)anyBad=anyBad||data.edgeBad[3*f+e];if(!anyBad)return 0u;
        for(int e=0;e<3;++e){const auto id=3*f+e;bool split=!data.edgeBad[id];
            for(auto n=data.edgeNeighbourFirst[id];n<data.edgeNeighbourFirst[id+1]&&split;++n)split=inJ[data.edgeNeighbours[n]]!=0;
            if(!split)m|=1u<<e;}return m;};
    for(auto& region:out)for(std::size_t k=0;k<region.faces.size();++k)region.protect[k]=std::uint8_t(protectMask(region.faces[k]));
    std::size_t built=0;state.mask.clear();
    for(std::uint32_t f=0;f<inJ.size();++f)if(inJ[f]){const unsigned pm=protectMask(f);state.mask[f]=std::uint8_t(pm);const auto k=latticeKey(f,pm);
        if(!state.lattices.count(k)){auto d=std::make_shared<LatticeData>();d->mask=pm;d->lattice=buildFaceLattice(tree.source,f,spec,pm);state.lattices.emplace(k,std::move(d));++built;}}
    stages.mark("lattices");
    return out;
}
// Does a (convex) strip polygon meet any tile of E (closed boxes)?
template<class Set>inline bool stripMeets(const LatticeSpec& spec,const LatticePiece& piece,const Set& inE){
    const auto& poly=piece.poly;Vec lo{1e30,1e30,1e30},hi{-1e30,-1e30,-1e30};
    for(const auto& v:poly)for(int a=0;a<3;++a){lo[a]=std::min(lo[a],v.v.p[a]);hi[a]=std::max(hi[a],v.v.p[a]);}
    const double eps=1e-9;
    const Cell t0=tileOfPoint(spec,lo-Vec{eps,eps,eps}),t1=tileOfPoint(spec,hi+Vec{eps,eps,eps});
    for(int x=t0.x;x<=t1.x;++x)for(int y=t0.y;y<=t1.y;++y)for(int z=t0.z;z<=t1.z;++z){if(!inE.count(Cell{x,y,z}))continue;
        const Vec blo=spec.origin+Vec{double(x*8),double(y*8),double(z*8)}*spec.pitch-Vec{eps,eps,eps},bhi=spec.origin+Vec{double(x*8+8),double(y*8+8),double(z*8+8)}*spec.pitch+Vec{eps,eps,eps};
        for(std::size_t k=1;k+1<poly.size();++k)if(triangleBoxOverlap(poly[0].v.p,poly[k].v.p,poly[k+1].v.p,blo,bhi))return true;}
    return false;
}
// Tile range of a part's bounding box, clamped to [lo,hi]; false if empty.
inline bool partRange(const LatticeSpec& spec,const LatticePiece& part,Cell lo,Cell hi,Cell& a,Cell& b){
    Vec mn{1e30,1e30,1e30},mx{-1e30,-1e30,-1e30};for(const auto& v:part.poly)for(int k=0;k<3;++k){mn[k]=std::min(mn[k],v.v.p[k]);mx[k]=std::max(mx[k],v.v.p[k]);}
    a=tileOfPoint(spec,mn);b=tileOfPoint(spec,mx);
    for(int k=0;k<3;++k){(&a.x)[k]=std::max((&a.x)[k],(&lo.x)[k]);(&b.x)[k]=std::min((&b.x)[k],(&hi.x)[k]);if((&a.x)[k]>(&b.x)[k])return false;}
    return true;
}
// Lattice pieces of face f's non-strip parts in the tiles of `tiles` (a set
// with count(Cell)), within the tile box [lo,hi].
template<class Set>inline void piecesIn(const LatticeData& lat,const LatticeSpec& spec,const Set& tiles,Cell lo,Cell hi,std::vector<std::pair<std::size_t,LatticePiece>>& out){
    std::vector<LatticePiece> pieces;
    for(std::size_t p=0;p<lat.lattice.parts.size();++p){const auto& part=lat.lattice.parts[p];if(part.strip)continue;Cell a,b;if(!partRange(spec,part,lo,hi,a,b))continue;
        pieces.clear();latticeSplitRange(*lat.lattice.frame,part.poly,a,b,pieces);
        for(auto& piece:pieces)if(tiles.count(piece.tile))out.push_back({p,std::move(piece)});}
}

// Exterior fragments of a tile-local region: lattice pieces of its joined faces
// in E, clipped by the removed cells of their own tile only.
inline core::ClipResult tileLocalFragments(const Grid& grid,const Region& region,const TileRegionState& state,const core::Source& patch,
                                           std::vector<Vec>& fixedPositions,std::unordered_map<core::Quant,std::array<double,64>,core::QuantHash>& fixedSkin,const AssetData& data){
    core::StageProfile stages("tile_exterior");
    core::ClipResult result;core::FlatMap<Cell,char,core::CellHash> inE,damaged;
    for(auto c:region.expanded)inE.try_emplace(c,0);for(auto c:region.keys)damaged.try_emplace(c,0);
    const double pitch=grid.pitch;Cell elo,ehi;cellBounds(region.expanded,elo,ehi);
    // Faces are independent until conform: clip them concurrently, then append
    // in face order (deterministic).
    struct FaceOut {std::vector<LatticePiece> pieces;std::vector<Vec> positions;std::vector<std::pair<core::Quant,std::array<double,64>>> skin;std::string error;};
    std::vector<FaceOut> outs(region.faces.size());
    // Phase 1 (per face): lattice pieces in E, canonical positions and skin.
    std::atomic<std::size_t> cursor{0};const std::size_t faceCount=region.faces.size();
    core::parallelChunks(faceCount>=8?core::parallelWidth():1,1,[&](std::size_t,std::size_t,std::size_t){
    std::vector<std::pair<std::size_t,LatticePiece>> pieces;
    for(std::size_t slot;(slot=cursor++)<faceCount;){const std::uint32_t pi=std::uint32_t(slot);const std::uint32_t face=region.faces[pi];const auto& lat=*state.lattices.at(latticeKey(face,region.protect[pi]));auto& fo=outs[pi];
        for(const auto& part:lat.lattice.parts){
            // Only an always-protected (bad) edge's strip may meet E; a strip
            // protected because its neighbour is outside J never can (J_R
            // covers E +- 1 cell). Verified, not assumed.
            if(part.strip&&!state.bad->at(3*std::size_t(face)+std::size_t(part.edge))&&stripMeets(state.spec,part,inE))
                fo.error="tile_local_strip_in_region";}
        pieces.clear();piecesIn(lat,state.spec,inE,elo,ehi,pieces);
        for(auto& item:pieces){const auto& piece=item.second;
            for(const auto& lv:piece.poly){fo.positions.push_back(lv.v.p);
                if(canonicalSkin(patch,data,lv))fo.skin.push_back({core::quantExact(floatRounded(lv.v.p)),latticeWeights(patch,data,face,lv)});}
            fo.pieces.push_back(std::move(item.second));}}});
    stages.mark("pieces");
    // Phase 2 (per piece; pieces vary most in cost): clip by the removed cells
    // of the piece's own tile.
    std::vector<std::pair<std::uint32_t,std::uint32_t>> tasks;for(std::uint32_t pi=0;pi<faceCount;++pi)for(std::uint32_t k=0;k<outs[pi].pieces.size();++k)tasks.push_back({pi,k});
    std::vector<std::vector<core::Poly>> clipped(tasks.size());std::atomic<std::size_t> taskCursor{0};
    core::parallelChunks(tasks.size()>=16?core::parallelWidth():1,1,[&](std::size_t,std::size_t,std::size_t){
    for(std::size_t slot;(slot=taskCursor++)<tasks.size();){const auto& piece=outs[tasks[slot].first].pieces[tasks[slot].second];
            std::vector<std::pair<Vec,Vec>> moved;
            core::Poly p;for(const auto& lv:piece.poly){core::V v=lv.v;v.material=1;v.p=exactPlanes(state.spec,lv);
                if(std::memcmp(&v.p,&lv.v.p,sizeof(Vec)))moved.push_back({v.p,lv.v.p});
                p.push_back(v);}
            std::vector<core::Poly> parts{p};
            auto tileIt=grid.tiles.find(piece.tile);
            if(damaged.count(piece.tile)&&tileIt!=grid.tiles.end()){const auto& bits=tileIt->second.removed;
                Cell lo,hi;for(int a=0;a<3;++a){double mn=1e30,mx=-1e30;for(auto&v:p){mn=std::min(mn,v.p[a]);mx=std::max(mx,v.p[a]);}
                    const int t0=(&piece.tile.x)[a]*8;(&lo.x)[a]=std::max(t0,int(std::floor((mn-grid.origin[a])/pitch)));(&hi.x)[a]=std::min(t0+7,int(std::floor((mx-grid.origin[a])/pitch)));}
                auto removed=[&](int x,int y,int z){const unsigned bit=unsigned(((x&7)*8+(y&7))*8+(z&7));return (bits[bit/64]>>(bit%64))&1u;};
                // Every cell under the piece's bounds removed: the piece lies
                // inside the removed volume and the subtract loop below would
                // leave nothing of it (area < 1e-16 slivers are dropped there too).
                bool all=true;for(int x=lo.x;x<=hi.x&&all;++x)for(int y=lo.y;y<=hi.y&&all;++y)for(int z=lo.z;z<=hi.z&&all;++z)all=removed(x,y,z)!=0;
                if(all)parts.clear();
                for(int x=lo.x;x<=hi.x&&!parts.empty();++x)for(int y=lo.y;y<=hi.y&&!parts.empty();++y)for(int z=lo.z;z<=hi.z&&!parts.empty();++z){
                    if(!removed(x,y,z))continue;const int first=z;while(z<hi.z&&removed(x,y,z+1))++z;
                    const Vec a=grid.origin+Vec{double(x),double(y),double(first)}*pitch,b=a+Vec{pitch,pitch,(z-first+1)*pitch};
                    std::vector<core::Poly> next;next.reserve(parts.size()+4);
                    // Reuse the exact clipping chain already used by the union path.
                    // A false return appends nothing: retain the untouched input.
                    for(auto& q:parts){if(!core::overlaps(q,a,b)||!core::subtractParts(q,a,b,next))next.push_back(std::move(q));}
                    parts=std::move(next);}}
            // Vertices moved onto a tile plane for clipping get their canonical
            // position back (bit-exact match: clipping copies kept vertices).
            if(!moved.empty())for(auto& q:parts)for(auto& v:q)for(const auto& m:moved)if(!std::memcmp(&v.p,&m.first,sizeof(Vec))){v.p=m.second;break;}
            clipped[slot]=std::move(parts);}});
    stages.mark("clip");
    for(auto& fo:outs)if(!fo.error.empty())throw RecoverableImpactError(fo.error);
    for(std::size_t t=0;t<tasks.size();++t)for(auto& q:clipped[t])result.polygons.push_back({tasks[t].first,std::move(q)});
    for(auto& fo:outs){fixedPositions.insert(fixedPositions.end(),fo.positions.begin(),fo.positions.end());for(auto& s:fo.skin)fixedSkin.emplace(s.first,s.second);}
    stages.mark("gather");
    core::conform(patch,result);
    // A fragment indistinguishable from a line at the 1e-8 snap (width <= 3e-8 m,
    // the rule the join applies to cavity polygons) would only fan into
    // zero-area triangles of arbitrary float orientation: drop it.
    result.polygons.erase(std::remove_if(result.polygons.begin(),result.polygons.end(),[](const core::Fragment& f){const auto& poly=f.polygon;
        Vec a=poly.front().p,b=a;double longest=0;for(const auto& v:poly){const double d=length(v.p-a);if(d>longest){longest=d;b=v.p;}}
        if(longest<=0)return true;double width=0;for(const auto& v:poly)width=std::max(width,length(cross(v.p-a,b-a))/longest);return width<=3e-8;}),result.polygons.end());
    // Canonical skin applies at exterior vertices only: a lattice point clipped
    // away (inside the removed volume) may coincide with a cavity vertex whose
    // skin the cavity solve owns.
    {core::FlatMap<core::Quant,char,core::QuantHash> present;for(const auto& f:result.polygons)for(const auto& v:f.polygon)present.try_emplace(core::quantExact(floatRounded(v.p)),0);
     for(auto it=fixedSkin.begin();it!=fixedSkin.end();)it=present.count(it->first)?std::next(it):fixedSkin.erase(it);}
    stages.mark("conform");
    return result;
}

// Seams of a tile-local region with statically emitted pieces: edges between
// two canonical lattice vertices on a tile plane separating a tile of E from a
// tile outside E, or on an inner/side line of an always-protected strip that
// meets E. A cut or conform vertex on a seam (a T-junction with the static
// side) is not canonical, so such an edge is still rejected by the gate.
inline std::function<bool(Vec,Vec)> tileLocalOpenings(const Grid& grid,const Region& region,const TileRegionState& state,const LatticeSpec& spec,const std::vector<Vec>& fixedPositions){
    (void)grid;
    auto inE=std::make_shared<core::FlatMap<Cell,char,core::CellHash>>();for(auto c:region.expanded)inE->try_emplace(c,0);
    auto canonical=std::make_shared<core::FlatMap<core::Quant,char,core::QuantHash>>();canonical->reserve(fixedPositions.size());
    for(const auto& p:fixedPositions)canonical->try_emplace(core::quantExact(floatRounded(p)),0);
    auto strips=std::make_shared<std::vector<std::pair<Vec,Vec>>>();
    for(std::uint32_t pi=0;pi<region.faces.size();++pi){if(!region.protect[pi])continue;const auto& lat=*state.lattices.at(latticeKey(region.faces[pi],region.protect[pi]));
        for(const auto& part:lat.lattice.parts){if(!part.strip||!state.bad->at(3*std::size_t(region.faces[pi])+std::size_t(part.edge))||!stripMeets(spec,part,*inE))continue;const auto& poly=part.poly;
            for(std::size_t k=0;k<poly.size();++k){if(poly[k].line.kind==LineKind::edge)continue;strips->push_back({poly[k].v.p,poly[(k+1)%poly.size()].v.p});}}}
    return [inE,strips,spec,canonical](Vec a,Vec b)->bool{
        if(!canonical->count(core::quantExact(a))||!canonical->count(core::quantExact(b)))return false;
        constexpr double tol=3e-7;
        for(int axis=0;axis<3;++axis){const double ia=(a[axis]-spec.origin[axis])/(8*spec.pitch);const int i=int(std::llround(ia));const double X=tilePlane(spec,axis,i);
            if(std::abs(a[axis]-X)>tol||std::abs(b[axis]-X)>tol)continue;
            Vec m=(a+b)*.5;Vec lo=m,hi=m;lo[axis]=X-4*spec.pitch;hi[axis]=X+4*spec.pitch;
            const bool l=inE->count(tileOfPoint(spec,lo))!=0,h=inE->count(tileOfPoint(spec,hi))!=0;if(l!=h)return true;}
        for(const auto& s:*strips){const Vec d=s.second-s.first;const double dd=dot(d,d);if(dd<1e-20)continue;
            auto on=[&](Vec v){const double t=dot(v-s.first,d)/dd;return length(v-(s.first+d*std::clamp(t,0.,1.)))<=tol;};
            if(on(a)&&on(b))return true;}
        return false;};
}

// ---- Static remainder of a joined face ------------------------------------
// keep(p): a crossing is a vertex of the static outline iff a tile whose closed
// box contains it is joined for this face (T_f). Any tile of any E whose box
// contains a point of f has f -- and every face sharing the point's edge -- in
// its J, so both sides of every shared line apply the same rule.
template<class Set>inline bool keepCrossing(const LatticeSpec& spec,const Set& tiles,Vec p){
    int lo[3],hi[3];constexpr double tol=2e-8;
    for(int a=0;a<3;++a){const int t=tileIndex(spec,a,p[a]);lo[a]=hi[a]=t;if(std::abs(p[a]-tilePlane(spec,a,t))<=tol)lo[a]=t-1;if(std::abs(p[a]-tilePlane(spec,a,t+1))<=tol)hi[a]=t+1;}
    for(int x=lo[0];x<=hi[0];++x)for(int y=lo[1];y<=hi[1];++y)for(int z=lo[2];z<=hi[2];++z)if(tiles.count(Cell{x,y,z}))return true;
    return false;
}
// Union of the E sets of the regions joining a face (usually one).
struct TileSets {std::vector<const core::FlatMap<Cell,char,core::CellHash>*> sets;Cell lo{1<<30,1<<30,1<<30},hi{-(1<<30),-(1<<30),-(1<<30)};
    bool count(Cell c)const{for(auto* s:sets)if(s->count(c))return true;return false;}bool empty()const{return sets.empty();}};
inline std::shared_ptr<const StaticFace> buildStaticFace(const core::Source& s,const AssetData& data,std::uint32_t face,const LatticeData& lat,
                                                         const LatticeSpec& spec,const TileSets& T){
    auto out=std::make_shared<StaticFace>();auto& d=*out;
    const Cell tlo=T.lo,thi=T.hi;
    const auto& fr=*lat.lattice.frame;
    // 2D frame: drop the dominant normal axis; CCW for the face's own winding.
    const Vec n=cross(fr.c[1]-fr.c[0],fr.c[2]-fr.c[0]);int k=0;for(int a=1;a<3;++a)if(std::abs(n[a])>std::abs(n[k]))k=a;
    int u=(k+1)%3,w=(k+2)%3;if(n[k]<0)std::swap(u,w);
    core::FlatMap<core::Quant,std::uint32_t,core::QuantHash> ids;std::vector<LatticeVertex> verts;std::vector<polytri::P2> pts;
    auto vid=[&](const LatticeVertex& v){const auto q=core::quantExact(v.v.p);auto it=ids.find(q);if(it!=ids.end())return it->second;
        const std::uint32_t id=std::uint32_t(verts.size());ids.emplace(q,id);verts.push_back(v);pts.push_back({v.v.p[u]-fr.c[0][u],v.v.p[w]-fr.c[0][w]});return id;};
    std::vector<std::pair<std::size_t,LatticePiece>> pieces;
    if(!T.empty())piecesIn(lat,spec,T,tlo,thi,pieces);
    std::vector<polytri::Tri> tris;std::string error;
    for(std::size_t p=0;p<lat.lattice.parts.size()&&d.triangulationError.empty();++p){const auto& part=lat.lattice.parts[p];
        // A non-strip part whose every overlapped tile is joined has no static remainder.
        if(!part.strip&&!T.empty()){Vec mn{1e30,1e30,1e30},mx{-1e30,-1e30,-1e30};for(const auto& v:part.poly)for(int a=0;a<3;++a){mn[a]=std::min(mn[a],v.v.p[a]);mx[a]=std::max(mx[a],v.v.p[a]);}
            const Cell a0=tileOfPoint(spec,mn),a1=tileOfPoint(spec,mx);std::int64_t n=std::int64_t(a1.x-a0.x+1)*(a1.y-a0.y+1)*(a1.z-a0.z+1);bool all=n<=4096;
            for(int x=a0.x;x<=a1.x&&all;++x)for(int y=a0.y;y<=a1.y&&all;++y)for(int z=a0.z;z<=a1.z&&all;++z)all=T.count(Cell{x,y,z});
            if(all)continue;}
        // Outline: the part's vertices plus the kept crossings of its lines.
        std::vector<std::uint32_t> outline;const auto& poly=part.poly;
        for(std::size_t i=0;i<poly.size();++i){const auto& a=poly[i];const auto& b=poly[(i+1)%poly.size()];
            if(part.strip){if(!a.crossing||keepCrossing(spec,T,a.v.p))outline.push_back(vid(a));continue;}
            outline.push_back(vid(a));
            for(auto c:lineCrossings(fr,a.line,a.v,b.v))if(keepCrossing(spec,T,c.v.p)){c.line=a.line;outline.push_back(vid(c));}}
        // Directed boundary: outline minus the joined pieces (opposite edges cancel).
        std::map<std::pair<std::uint32_t,std::uint32_t>,int> edges;
        auto add=[&](std::uint32_t x,std::uint32_t y){if(x==y)return;auto r=edges.find({y,x});if(r!=edges.end()){if(--r->second==0)edges.erase(r);return;}++edges[{x,y}];};
        for(std::size_t i=0;i<outline.size();++i)add(outline[i],outline[(i+1)%outline.size()]);
        for(const auto& item:pieces){if(item.first!=p)continue;const auto& q=item.second.poly;for(std::size_t i=0;i<q.size();++i)add(vid(q[(i+1)%q.size()]),vid(q[i]));}
        std::vector<polytri::DEdge> list;for(const auto& e:edges){if(e.second!=1){d.triangulationError="edge_multiplicity";break;}list.push_back(e.first);}
        if(!d.triangulationError.empty()||list.empty())continue;
        if(!polytri::triangulate(pts,list,3e-8,tris,error)){d.triangulationError=error;break;}
        // Rows are made below per used vertex; keep triangles by vertex id.
        for(const auto& t:tris)d.tris.push_back(t);}
    if(!d.triangulationError.empty()){d.tris.clear();return out;}
    // Rows per used vertex (the join's interpolation, canonical skin on corner
    // and original-edge vertices), float positions; the join's drop rules.
    std::vector<std::int64_t> row(verts.size(),-1);std::vector<std::array<std::uint32_t,3>> kept;kept.reserve(d.tris.size());
    try{
        for(const auto& t:d.tris){Vec p[3];std::array<core::Quant,3> q;for(int j=0;j<3;++j){p[j]=floatRounded(verts[t[j]].v.p);q[j]=core::quantExact(p[j]);}
            if(q[0]==q[1]||q[1]==q[2]||q[2]==q[0]||length(cross(p[1]-p[0],p[2]-p[0]))<1e-16){++d.dropped;continue;}
            std::array<std::uint32_t,3> r;
            for(int j=0;j<3;++j){auto& id=row[t[j]];if(id<0){const auto& lv=verts[t[j]];auto jv=core::interpolateVertex(s,face,lv.v);if(!jv.hardware_corner&&canonicalSkin(s,data,lv))jv.w=latticeWeights(s,data,face,lv);jv.p=p[j];
                    LatticeRow lr;lr.row=core::packing::vertex(jv,&lr.discarded);lr.hash=core::packing::vertexHash(lr.row);id=std::int64_t(d.rows.size());d.rows.push_back(lr);}
                r[j]=std::uint32_t(id);}
            kept.push_back(r);}
    }catch(const core::packing::PackingInfluenceError& e){d.packingError=e.what();kept.clear();}
    d.tris=std::move(kept);
    return out;
}

// Static remainders of every joined face for the current regions (memoized
// by face, mask and the regions joining it; built in parallel).
inline void prepareStatics(const core::Source& s,const AssetData& data,const RegionList& regions,TileRegionState& state){
    core::StageProfile stages("statics");
    // A face's static is a pure function of its key (face, protect mask, hashes of the regions joining it). Only faces
    // of region results new since the last call or gone since, and faces whose mask changed, can have a new key;
    // every other face keeps its current static.
    std::vector<char> affected(s.f.size(),0);
    {std::unordered_set<const RegionResult*> prev;prev.reserve(state.staticsResults.size());for(const auto& r:state.staticsResults)prev.insert(r.get());
     std::unordered_set<const RegionResult*> cur;cur.reserve(regions.size());for(const auto& r:regions)cur.insert(r.get());
     for(const auto& r:regions)if(!prev.count(r.get()))for(auto f:r->faces)affected[f]=1;
     for(const auto& r:state.staticsResults)if(!cur.count(r.get()))for(auto f:r->faces)affected[f]=1;
     for(const auto& k:state.staticKeys){if(affected[k.first])continue;auto m=state.mask.find(k.first);if(m==state.mask.end()||m->second!=k.second.mask)affected[k.first]=1;}}
    std::unordered_map<std::uint32_t,std::vector<const RegionResult*>> byFace;
    for(const auto& r:regions)if(r->tileLocal)for(auto f:r->faces)if(affected[f])byFace[f].push_back(r.get());
    struct Job {TileRegionState::StaticKey key;TileSets tiles;std::shared_ptr<const StaticFace> built;};std::vector<Job> jobs;
    struct RegionSet {core::FlatMap<Cell,char,core::CellHash> set;Cell lo,hi;bool built=false;};std::unordered_map<const RegionResult*,RegionSet> sets;
    // Affected faces no tile-local region joins any more leave (the memo keeps their key and static).
    for(auto it=state.staticKeys.begin();it!=state.staticKeys.end();){
        if(affected[it->first]&&!byFace.count(it->first)){state.staticMemo[it->first]={std::move(it->second),state.current.at(it->first)};state.current.erase(it->first);it=state.staticKeys.erase(it);}
        else ++it;}
    auto adopt=[&](std::uint32_t face,TileRegionState::StaticKey key,std::shared_ptr<const StaticFace> built){auto have=state.staticKeys.find(face);
        if(have!=state.staticKeys.end()){state.staticMemo[face]={std::move(have->second),state.current.at(face)};have->second=std::move(key);}
        else state.staticKeys.emplace(face,std::move(key));
        state.current[face]=std::move(built);};
    for(auto& item:byFace){TileRegionState::StaticKey key;key.face=item.first;key.mask=state.mask.at(item.first);
        std::sort(item.second.begin(),item.second.end(),[](auto a,auto b){return a->hash<b->hash;});for(auto* r:item.second)key.regions.push_back(r->hash);
        auto have=state.staticKeys.find(item.first);
        if(have!=state.staticKeys.end()&&have->second==key)continue; // same key: the current static stands
        auto memo=state.staticMemo.find(item.first);
        if(memo!=state.staticMemo.end()&&memo->second.first==key){auto kept=memo->second.second;state.staticMemo.erase(memo);adopt(item.first,std::move(key),std::move(kept));continue;}
        Job job;job.key=key;
        for(auto* r:item.second){auto& rs=sets[r];if(!rs.built){rs.built=true;rs.set.reserve(r->expanded.size());for(auto c:r->expanded)rs.set.try_emplace(c,0);cellBounds(r->expanded,rs.lo,rs.hi);}
            job.tiles.sets.push_back(&rs.set);for(int k=0;k<3;++k){(&job.tiles.lo.x)[k]=std::min((&job.tiles.lo.x)[k],(&rs.lo.x)[k]);(&job.tiles.hi.x)[k]=std::max((&job.tiles.hi.x)[k],(&rs.hi.x)[k]);}}
        jobs.push_back(std::move(job));}
    stages.mark("keys");
    std::atomic<std::size_t> jobCursor{0};
    core::parallelChunks(jobs.size()>=8?core::parallelWidth():1,1,[&](std::size_t,std::size_t,std::size_t){for(std::size_t n;(n=jobCursor++)<jobs.size();){auto& job=jobs[n];
        job.built=buildStaticFace(s,data,job.key.face,*state.lattices.at(latticeKey(job.key.face,job.key.mask)),state.spec,job.tiles);}});
    stages.mark("build");
    state.staticBuilt=jobs.size();state.staticTriangles=0;state.staticDropped=0;
    for(auto& job:jobs)adopt(job.key.face,std::move(job.key),std::move(job.built));
    for(const auto& c:state.current){state.staticTriangles+=c.second->tris.size();state.staticDropped+=c.second->dropped;}
    state.staticsResults=regions;
}
// A static triangulation failure rejects the candidate (recoverable); a static
// row that failed packing fails it like the join's rows.
inline void checkStatics(const TileRegionState& state){
    for(const auto& c:state.current)if(!c.second->triangulationError.empty())throw RecoverableImpactError("tile_local_static_triangulation "+c.second->triangulationError);
    for(const auto& c:state.current)if(!c.second->packingError.empty())throw core::packing::PackingInfluenceError(c.second->packingError);
}
inline void appendStatic(const TileRegionState& state,std::uint32_t face,const std::vector<std::uint32_t>& faceIndices,std::vector<TriRef>& list,std::size_t& count){
    const auto& d=*state.current.at(face);
    for(const auto& t:d.tris){TriRef r;for(int k=0;k<3;++k){const auto& row=d.rows[t[k]];r.row[k]=&row.row;r.hash[k]=row.hash;r.discarded[k]=row.discarded;}r.original_face=faceIndices[face];list.push_back(r);++count;}
}
}
