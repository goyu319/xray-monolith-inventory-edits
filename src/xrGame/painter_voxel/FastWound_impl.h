#pragma once
// Included by LiveDamageWorker.cpp inside its anonymous namespace, after
// joinFullBody. Incremental full-body wound assembly:
//
//  1. Exact per-tile cavity boundary quads from the 512-bit tile masks
//     (bit-parallel equivalent of core::dirtyBoundary for owner-free grids).
//  2. The removed tiles are partitioned into spatially independent REGIONS.
//     Two tiles share a region when they are 26-neighbours, or when the
//     bounding boxes of everything a region can emit (tile boxes grown by the
//     2-cell source selection halo, plus the bounds of every selected source
//     face) come within one damage pitch. Every join tolerance is <= 1e-6 m, so
//     no weld, conform, edge, skin, spur or gate decision can couple two
//     regions; their source patches are disjoint.
//  3. A region's joined, gated and packed triangles are a pure function of its
//     exact tile masks (plus the immutable asset). Unchanged regions are reused
//     from the committed session state; only regions containing new damage run
//     boundary merge, exterior clip and join again.
//  4. Untouched source faces never become JoinedTriangles: their packed rows and
//     whole untouched exterior children are precomputed per asset.
//
// Semantics: a single-region result is byte-identical to the former union
// path. With several regions, cavity-skin mode (conformed retry, harmonic vs
// seed-copy) is decided per region instead of by the union of all historical
// wounds, so an unrelated later wound no longer re-skins an old one.
#include "FaceLattice.h"
namespace fastwound {
using QuadList=std::shared_ptr<const std::vector<core::Quad>>;
using BoundaryCache=std::map<Cell,QuadList>;

// Exact equivalent of core::dirtyBoundary(g,{t}) when !g.ownership: same quads,
// same (cell x,y,z; direction) order, same corner arithmetic and material.
inline void tileBoundary(const Grid& g,Cell t,std::vector<core::Quad>& out){
    if(t.x<0||t.y<0||t.z<0)return;auto it=g.tiles.find(t);if(it==g.tiles.end())return;
    const auto& w=it->second.removed;
    auto neighbour=[&](int dx,int dy,int dz)->const std::array<std::uint64_t,8>*{auto f=g.tiles.find(Cell{t.x+dx,t.y+dy,t.z+dz});return f==g.tiles.end()?nullptr:&f->second.removed;};
    const auto* mx=neighbour(-1,0,0);const auto* px=neighbour(1,0,0);const auto* my=neighbour(0,-1,0);
    const auto* py=neighbour(0,1,0);const auto* mz=neighbour(0,0,-1);const auto* pz=neighbour(0,0,1);
    constexpr std::uint64_t Z0=0x0101010101010101ull,Z7=Z0<<7;
    static const int corners[6][4][3]={{{0,0,0},{0,0,1},{0,1,1},{0,1,0}},{{1,0,0},{1,1,0},{1,1,1},{1,0,1}},{{0,0,0},{1,0,0},{1,0,1},{0,0,1}},{{0,1,0},{0,1,1},{1,1,1},{1,1,0}},{{0,0,0},{0,1,0},{1,1,0},{1,0,0}},{{0,0,1},{1,0,1},{1,1,1},{0,1,1}}};
    static const int delta[6][3]={{-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1}};
    for(int x=0;x<8;++x){const std::uint64_t c=w[x];if(!c)continue;
        std::uint64_t n[6];
        n[0]=x>0?w[x-1]:(mx?(*mx)[7]:0);
        n[1]=x<7?w[x+1]:(px?(*px)[0]:0);
        n[2]=(c<<8)|(my?((*my)[x]>>56):0);
        n[3]=(c>>8)|(py?(((*py)[x]&0xFFull)<<56):0);
        n[4]=((c<<1)&~Z0)|(mz?(((*mz)[x]&Z7)>>7):0);
        n[5]=((c>>1)&~Z7)|(pz?(((*pz)[x]&Z0)<<7):0);
        std::uint64_t open[6],any=0;for(int a=0;a<6;++a){open[a]=c&~n[a];any|=open[a];}
        while(any){unsigned bit=0;while(!((any>>bit)&1u))++bit;any&=any-1;
            const Cell cell{t.x*8+x,t.y*8+int(bit>>3),t.z*8+int(bit&7)};
            for(int a=0;a<6;++a){if(!((open[a]>>bit)&1u))continue;
                const Cell nb{cell.x+delta[a][0],cell.y+delta[a][1],cell.z+delta[a][2]};
                core::Quad q;q.removedCell=cell;q.material=g.renderMaterial(nb);
                for(int k=0;k<4;++k)q.p[k]=g.origin+Vec{double(cell.x+corners[a][k][0]),double(cell.y+corners[a][k][1]),double(cell.z+corners[a][k][2])}*g.pitch;
                out.push_back(q);}
        }
    }
}

// ---- Compact direct-path records ------------------------------------------------------------------------------
// A unit voxel face of the direct path as 16 bytes: the removed cell, the face (tileBoundary's index 0..5) and the
// render material of the cell across it. Its four corner doubles are regenerated on use by faceCorners() with the
// very expression tileBoundary uses, so every downstream double (keys, normals, clips) is bit-identical to the
// former 112-byte core::Quad path. The corner and neighbour tables are tileBoundary's, in the same order.
struct FaceRec {int x,y,z;std::uint16_t material;std::uint8_t face;std::uint8_t pad{};};
inline constexpr int kFaceCorners[6][4][3]={{{0,0,0},{0,0,1},{0,1,1},{0,1,0}},{{1,0,0},{1,1,0},{1,1,1},{1,0,1}},{{0,0,0},{1,0,0},{1,0,1},{0,0,1}},{{0,1,0},{0,1,1},{1,1,1},{1,1,0}},{{0,0,0},{0,1,0},{1,1,0},{1,0,0}},{{0,0,1},{1,0,1},{1,1,1},{0,1,1}}};
inline constexpr int kFaceDelta[6][3]={{-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1}};
inline void faceCorners(const Grid& g,const FaceRec& f,Vec* out){
    for(int k=0;k<4;++k)out[k]=g.origin+Vec{double(f.x+kFaceCorners[f.face][k][0]),double(f.y+kFaceCorners[f.face][k][1]),double(f.z+kFaceCorners[f.face][k][2])}*g.pitch;}
// tileBoundary's faces, in tileBoundary's order, as FaceRec.
inline void tileBoundaryRecs(const Grid& g,Cell t,std::vector<FaceRec>& out){
    if(t.x<0||t.y<0||t.z<0)return;auto it=g.tiles.find(t);if(it==g.tiles.end())return;
    const auto& w=it->second.removed;
    auto neighbour=[&](int dx,int dy,int dz)->const std::array<std::uint64_t,8>*{auto f=g.tiles.find(Cell{t.x+dx,t.y+dy,t.z+dz});return f==g.tiles.end()?nullptr:&f->second.removed;};
    const auto* mx=neighbour(-1,0,0);const auto* px=neighbour(1,0,0);const auto* my=neighbour(0,-1,0);
    const auto* py=neighbour(0,1,0);const auto* mz=neighbour(0,0,-1);const auto* pz=neighbour(0,0,1);
    constexpr std::uint64_t Z0=0x0101010101010101ull,Z7=Z0<<7;
    for(int x=0;x<8;++x){const std::uint64_t c=w[x];if(!c)continue;
        std::uint64_t n[6];
        n[0]=x>0?w[x-1]:(mx?(*mx)[7]:0);
        n[1]=x<7?w[x+1]:(px?(*px)[0]:0);
        n[2]=(c<<8)|(my?((*my)[x]>>56):0);
        n[3]=(c>>8)|(py?(((*py)[x]&0xFFull)<<56):0);
        n[4]=((c<<1)&~Z0)|(mz?(((*mz)[x]&Z7)>>7):0);
        n[5]=((c>>1)&~Z7)|(pz?(((*pz)[x]&Z0)<<7):0);
        std::uint64_t open[6],any=0;for(int a=0;a<6;++a){open[a]=c&~n[a];any|=open[a];}
        while(any){unsigned bit=0;while(!((any>>bit)&1u))++bit;any&=any-1;
            const Cell cell{t.x*8+x,t.y*8+int(bit>>3),t.z*8+int(bit&7)};
            for(int a=0;a<6;++a){if(!((open[a]>>bit)&1u))continue;
                const Cell nb{cell.x+kFaceDelta[a][0],cell.y+kFaceDelta[a][1],cell.z+kFaceDelta[a][2]};
                FaceRec f;f.x=cell.x;f.y=cell.y;f.z=cell.z;f.material=std::uint16_t(g.renderMaterial(nb));f.face=std::uint8_t(a);
                out.push_back(f);}
        }
    }
}
inline bool compactWalls(){static const bool v=[]{const char* e=std::getenv("GFY_COMPACT_WALLS");return !e||std::atoi(e)!=0;}();return v;}
struct PointerHash {std::size_t operator()(const void* p)const{return std::size_t(reinterpret_cast<std::uintptr_t>(p)>>4);}};
struct PackedTri {std::uint32_t row[3];std::uint32_t face;std::uint32_t rect{~0u};}; // face: original source face, ~0 for cavity; rect: merged boundary rect of a cavity triangle
// A merged boundary rectangle of core::mergeBoundaryQuads, with the data that
// fixes its position in the former union output: plane key, rows and runs.
using PlaneKey=std::array<int,4>; // normal axis, integer plane, normal sign, material
struct RectMeta {PlaneKey plane;int u0,u1,v0,v1;};
// Exact copy of core::mergeBoundaryQuads (same quads, same order) that also
// reports each emitted rectangle and the row set of every plane.
// seamLo/seamHi (chunked regions): cell-plane coordinates of the chunk's six faces. A rectangle never
// spans a seam edge: its edges lying on a seam plane are unit cells, and a rectangle ON a seam plane is
// emitted as unit quads, so the two chunks of a seam produce identical seam vertices (no T-junction).
inline std::vector<core::Quad> mergeBoundaryRects(const Grid& g,const std::vector<core::Quad>& input,std::vector<RectMeta>& meta,std::map<PlaneKey,std::vector<int>>& planeRows,
                                                  const int* seamLo=nullptr,const int* seamHi=nullptr,bool tileSplit=false){
    // Flat (plane, row, u) records sorted lexicographically visit planes, rows
    // and sorted unique runs in exactly the nested std::map order.
    struct Entry {PlaneKey plane;int row,u;bool operator<(const Entry& o)const{return std::tie(plane,row,u)<std::tie(o.plane,o.row,o.u);}bool operator==(const Entry& o)const{return plane==o.plane&&row==o.row&&u==o.u;}};
    std::vector<Entry> byAxis[3];for(auto& list:byAxis)list.reserve(input.size()/2);
    for(auto& q:input){Vec n=cross(q.p[1]-q.p[0],q.p[2]-q.p[0]);int axis=0;
        for(int a=1;a<3;++a)if(std::abs(n[a])>std::abs(n[axis]))axis=a;
        int u=(axis+1)%3,v=(axis+2)%3;Vec lo=q.p[0];for(auto p:q.p)for(int a=0;a<3;++a)lo[a]=std::min(lo[a],p[a]);
        byAxis[axis].push_back({{axis,int(std::llround((lo[axis]-g.origin[axis])/g.pitch)),n[axis]>0?1:-1,int(q.material)},
            int(std::llround((lo[v]-g.origin[v])/g.pitch)),int(std::llround((lo[u]-g.origin[u])/g.pitch))});
    }
    // The axis is the leading key: sorting each axis separately (concurrently)
    // and concatenating in axis order is the same total order.
    core::parallelChunks(3,1,[&](std::size_t first,std::size_t last,std::size_t){for(std::size_t a=first;a<last;++a)std::sort(byAxis[a].begin(),byAxis[a].end());});
    std::vector<Entry> entries;entries.reserve(byAxis[0].size()+byAxis[1].size()+byAxis[2].size());
    for(auto& list:byAxis)entries.insert(entries.end(),list.begin(),list.end());
    entries.erase(std::unique(entries.begin(),entries.end()),entries.end());
    std::vector<core::Quad> out;out.reserve(entries.size()/2+16);
    for(std::size_t first=0;first<entries.size();){
        const PlaneKey key=entries[first].plane;std::size_t last=first;while(last<entries.size()&&entries[last].plane==key)++last;
        const int axis=key[0],u=(axis+1)%3,v=(axis+2)%3;auto& rows=planeRows[key];
        struct Rect{int u0,u1,v0,v1;};std::map<std::pair<int,int>,Rect> active;
        auto emitOne=[&](Rect r){core::Quad q;q.material=unsigned(key[3]);int uv[4][2]={{r.u0,r.v0},{r.u1,r.v0},{r.u1,r.v1},{r.u0,r.v1}};
            for(int k=0;k<4;++k){Vec p;p[axis]=key[1];p[u]=uv[k][0];p[v]=uv[k][1];q.p[k]=g.origin+p*g.pitch;}
            if(key[2]<0)std::reverse(q.p.begin(),q.p.end());
            (&q.removedCell.x)[axis]=key[1]-(key[2]>0?1:0);
            (&q.removedCell.x)[u]=r.u0;(&q.removedCell.x)[v]=r.v0;out.push_back(q);meta.push_back({key,r.u0,r.u1,r.v0,r.v1});
        };
        auto onSeam=[&](int a,int x){return seamLo&&(x==seamLo[a]||x==seamHi[a]);};
        auto emitSeam=[&](Rect r){
            if(!seamLo){emitOne(r);return;}
            if(onSeam(axis,key[1])){for(int uu=r.u0;uu<r.u1;++uu)for(int vv=r.v0;vv<r.v1;++vv)emitOne(Rect{uu,uu+1,vv,vv+1});return;}
            const bool su0=onSeam(u,r.u0),su1=onSeam(u,r.u1),sv0=onSeam(v,r.v0),sv1=onSeam(v,r.v1);
            if(!(su0||su1||sv0||sv1)){emitOne(r);return;}
            const Rect c{r.u0+(su0?1:0),r.u1-(su1?1:0),r.v0+(sv0?1:0),r.v1-(sv1?1:0)};const bool hasCore=c.u0<c.u1&&c.v0<c.v1;
            if(hasCore)emitOne(c);
            for(int uu=r.u0;uu<r.u1;++uu)for(int vv=r.v0;vv<r.v1;++vv){if(hasCore&&uu>=c.u0&&uu<c.u1&&vv>=c.v0&&vv<c.v1)continue;emitOne(Rect{uu,uu+1,vv,vv+1});}
        };
        // Tile-local regions: a rectangle never crosses a tile plane. The skin is split into lattice pieces at
        // tile planes, so its rim has a vertex wherever a tile plane crosses it; a wall rectangle spanning that
        // plane would meet it with a T-junction (open-edge gate failure and a slow former-path fallback).
        auto emit=[&](Rect r){if(!tileSplit){emitSeam(r);return;}
            auto next8=[](int x){return (x>=0?x/8:-((-x+7)/8))*8+8;};
            for(int a=r.u0;a<r.u1;){const int ua=std::min(r.u1,next8(a));for(int b=r.v0;b<r.v1;){const int vb=std::min(r.v1,next8(b));emitSeam(Rect{a,ua,b,vb});b=vb;}a=ua;}};
        for(std::size_t at=first;at<last;){const int row=entries[at].row;std::size_t end=at;while(end<last&&entries[end].row==row)++end;rows.push_back(row);
            std::map<std::pair<int,int>,Rect> next;
            for(std::size_t k=at;k<end;){int lo=entries[k].u,hi=lo+1;++k;while(k<end&&entries[k].u==hi){++hi;++k;}
                std::pair<int,int> runKey{lo,hi};auto it=active.find(runKey);
                if(it!=active.end()&&it->second.v1==row){Rect r=it->second;r.v1=row+1;next.emplace(runKey,r);active.erase(it);}
                else next.emplace(runKey,Rect{lo,hi,row,row+1});
            }
            for(auto& item:active)emit(item.second);active=std::move(next);at=end;
        }
        for(auto& item:active)emit(item.second);first=last;
    }
    return out;
}
using GroupKey=std::pair<unsigned,std::uint16_t>;
struct RegionResult {
    std::vector<Cell> keys;std::vector<core::Tile> masks;std::uint64_t hash{};
    std::vector<std::uint32_t> faces; // sorted original source faces of the patch (active)
    std::vector<std::uint32_t> selected; // sorted faces the former union patch held for this region (active + passive)
    std::vector<gfy_packed::Vertex> rows;std::vector<std::uint64_t> hashes;std::vector<double> discarded;
    std::vector<std::pair<GroupKey,std::vector<PackedTri>>> groups; // ascending key, join order inside
    std::size_t nonmanifoldCavityDoubled{};
    std::size_t triangles{},originalOpen{},newOpen{},nonmanifold{},numericalOpenEdges{},numericalSeams{},numericalSpurs{},interiorSheetJunctions{},cavityTriangles{};
    double maxConflict{},numericalOpenLength{};
    std::string packingError; // deferred until after the topology gate, like the union path
    std::vector<RectMeta> rects;std::map<PlaneKey,std::vector<int>> planeRows; // cavity order provenance
    bool same(const std::vector<Cell>& k,const std::vector<core::Tile>& m)const{
        if(k.size()!=keys.size())return false;
        for(std::size_t i=0;i<k.size();++i)if(!(k[i]==keys[i])||m[i].removed!=masks[i].removed)return false;
        return true;}
    bool same(const std::vector<Cell>& k,const std::vector<core::Tile>& m,const std::vector<std::uint32_t>& f)const{return f==faces&&same(k,m);}
    // Same region content: same tiles, joined faces and relevant protect bits.
    bool same(const std::vector<Cell>& k,const std::vector<core::Tile>& m,const std::vector<std::uint32_t>& f,const std::vector<std::uint8_t>& p,const std::vector<std::uint8_t>& rel)const{
        if(rel!=relevant||p.size()!=protect.size()||!same(k,m,f))return false;for(std::size_t i=0;i<p.size();++i)if((p[i]&rel[i])!=(protect[i]&relevant[i]))return false;return true;}
    bool tileLocal{};std::vector<Cell> expanded;std::vector<std::uint8_t> protect,relevant;
    // Low-poly display (LowPolyDisplay.h): exact joined triangle count before the reduction, error, timing.
    std::size_t lowpolyInput{},lowpolyMixed{},lowpolyFlipped{},lowpolyRetries{},lowpolyTarget{},lowpolyFixedExt{},lowpolyFreeExt{};
    std::size_t lowpolyDeepIn{},lowpolyDeepOut{},lowpolyDeepFinal{};double lowpolyDeepMs{}; // wall triangles below the depth band: before/after their coarser first pass, in the output
    std::size_t lowpolyLocked{},lowpolySeamDropped{},lowpolyWedges{};
    // Chunked region: the chunk's six seam planes in metres (vertices on them are locked by the reduction).
    bool chunked{};double seamLoM[3]{},seamHiM[3]{};
    // Direct display (buildDirect): wall feature radius (2 x removed tissue volume / wall area) for the reduction's
    // error limit, and the canonical lattice skin points (sorted keys) that stay locked; other skin rim points are free.
    bool directDisplay{};double featureRadius{};std::vector<core::Quant> lockedSkinKeys;
    // Merged display group: its member chunks (sorted). A vertex on a chunk plane stays locked unless every chunk
    // whose closed box holds it is a member (then both sides of the plane are reduced here together).
    std::vector<Cell> lockMembers;double chunkOrigin[3]{},chunkSize{};float lowpolyError{},lowpolyLimit{},lowpolyFlipArea{};double lowpolyMs{};
    // regionDefect() of the EXACT joined triangles, taken before the low-poly reduction.
    bool defectChecked{};std::string exactDefect;
};
using RegionList=std::vector<std::shared_ptr<const RegionResult>>;
// Per tile (pure function of the tile key, asset and lattice):
//  selected: faces the former joinFullBody selected (bounds within 2 cells);
//  active:   selected faces whose triangle comes within one cell of the tile
//            (can be clipped or meet a cut/cavity point), plus unclean ones.
// findRegions adds every SELECTED face sharing a vertex position with an
// active face (slab splits end anywhere on a clipped face's edges). Every
// other selected (passive) face cannot be clipped, receive a point, weld or
// re-skin: core::join returns it exactly as its untouched rows, so it is packed
// from those rows in its former patch order.
struct TileFaces {std::vector<std::uint32_t> selected,active,touching;};
using TileFaceMemo=std::unordered_map<Cell,TileFaces,core::CellHash>;

struct Region {std::vector<Cell> keys;std::vector<core::Tile> masks;std::vector<std::uint32_t> faces,selected;std::uint64_t hash{};
    // Chunked (TileRegions.h): one chunk of a large cluster (boxed: a box of chunks around a failed chunk);
    // seam planes in cell coordinates.
    bool chunked{},boxed{};int seamLo[3]{},seamHi[3]{};std::uint64_t cluster{};
    // Tile-local regions (TileRegions.h): joined tiles (cluster + halo) and the
    // protected-edge mask of every joined face.
    bool tileLocal{};std::vector<Cell> expanded;std::vector<std::uint8_t> protect,relevant;};

inline bool triangleBoxOverlap(Vec a,Vec b,Vec c,Vec lo,Vec hi);
inline const TileFaces& tileFaces(const Grid& grid,const core::BodyBVH& tree,const std::vector<char>& clean,TileFaceMemo& memo,Cell c){
    auto found=memo.find(c);if(found!=memo.end())return found->second;
    core::Bounds box;
    box.lo=grid.origin+Vec{double(c.x*8-2),double(c.y*8-2),double(c.z*8-2)}*grid.pitch;
    box.hi=grid.origin+Vec{double(c.x*8+10),double(c.y*8+10),double(c.z*8+10)}*grid.pitch;
    TileFaces faces;tree.query(0,box,faces.selected);std::sort(faces.selected.begin(),faces.selected.end());
    const Vec lo=grid.origin+Vec{double(c.x*8-1),double(c.y*8-1),double(c.z*8-1)}*grid.pitch,hi=grid.origin+Vec{double(c.x*8+9),double(c.y*8+9),double(c.z*8+9)}*grid.pitch;
    for(auto f:faces.selected){const auto&ids=tree.source.f[f];
        const bool touch=triangleBoxOverlap(tree.source.v[ids[0]].p,tree.source.v[ids[1]].p,tree.source.v[ids[2]].p,lo,hi);
        if(touch)faces.touching.push_back(f);
        if(f>=clean.size()||!clean[f]||touch)faces.active.push_back(f);}
    return memo.emplace(c,std::move(faces)).first->second;
}

// Conservative triangle / axis-aligned box overlap (separating axis test).
inline bool triangleBoxOverlap(Vec a,Vec b,Vec c,Vec lo,Vec hi){
    const Vec center=(lo+hi)*.5,half=(hi-lo)*.5;const Vec v0=a-center,v1=b-center,v2=c-center;
    const Vec e[3]={v1-v0,v2-v1,v0-v2};
    for(int k=0;k<3;++k){const double mn=std::min({v0[k],v1[k],v2[k]}),mx=std::max({v0[k],v1[k],v2[k]});if(mn>half[k]||mx<-half[k])return false;}
    const Vec n=cross(e[0],e[1]);const double r=half.x*std::abs(n.x)+half.y*std::abs(n.y)+half.z*std::abs(n.z);const double s=dot(n,v0);if(s>r||s<-r)return false;
    for(int i=0;i<3;++i)for(int k=0;k<3;++k){Vec axis;axis[k]=1;const Vec ax=cross(axis,e[i]);if(dot(ax,ax)<1e-30)continue;
        const double p0=dot(ax,v0),p1=dot(ax,v1),p2=dot(ax,v2);const double rr=half.x*std::abs(ax.x)+half.y*std::abs(ax.y)+half.z*std::abs(ax.z);
        if(std::min({p0,p1,p2})>rr||std::max({p0,p1,p2})<-rr)return false;}
    return true;
}
inline std::vector<Region> findRegions(const Grid& grid,const core::BodyBVH& tree,const AssetData& data,TileFaceMemo& memo){const auto& clean=data.clean;
    std::vector<Cell> keys;keys.reserve(grid.tiles.size());for(const auto& t:grid.tiles)keys.push_back(t.first);
    std::sort(keys.begin(),keys.end());
    core::FlatMap<Cell,int,core::CellHash> index;index.reserve(keys.size());
    for(int i=0;i<int(keys.size());++i)index.emplace(keys[i],i);
    std::vector<int> parent(keys.size());for(int i=0;i<int(parent.size());++i)parent[i]=i;
    auto root=[&](int i){while(parent[i]!=i){parent[i]=parent[parent[i]];i=parent[i];}return i;};
    auto unite=[&](int a,int b){a=root(a);b=root(b);if(a!=b)parent[std::max(a,b)]=std::min(a,b);};
    // Adjacency is symmetric: visiting the 13 lexicographically greater
    // neighbours of every tile unites exactly the same components.
    for(int i=0;i<int(keys.size());++i)for(int dx=0;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)for(int dz=-1;dz<=1;++dz){
        if(dx==0&&(dy<0||(dy==0&&dz<=0)))continue;auto f=index.find(Cell{keys[i].x+dx,keys[i].y+dy,keys[i].z+dz});if(f!=index.end())unite(i,f->second);}
    // A face ACTIVE for several tiles may be clipped or conformed by all of
    // them: those tiles are one region. Active faces are disjoint between
    // regions; passive faces belong to no region.
    std::vector<char> unionSelected(tree.source.f.size(),0);
    for(int i=0;i<int(keys.size());++i)for(auto f:tileFaces(grid,tree,clean,memo,keys[i]).selected)unionSelected[f]=1;
    // Tile i's active faces: its own active faces and every union-selected
    // face sharing a vertex position with them.
    std::vector<std::vector<std::uint32_t>> tileActive(keys.size());
    for(int i=0;i<int(keys.size());++i){auto& list=tileActive[i];const auto& own=tileFaces(grid,tree,clean,memo,keys[i]).active;list=own;
        for(auto f:own)for(auto r=data.ringFirst[f];r<data.ringFirst[f+1];++r)if(unionSelected[data.ringFaces[r]])list.push_back(data.ringFaces[r]);
        std::sort(list.begin(),list.end());list.erase(std::unique(list.begin(),list.end()),list.end());}
    struct FaceHash{std::size_t operator()(std::uint32_t f)const{return f;}};core::FlatMap<std::uint32_t,int,FaceHash> faceOwner;faceOwner.reserve(keys.size()*8);
#ifdef GFY_CS_TRACE
    {std::vector<int> adj(keys.size());for(int i=0;i<int(keys.size());++i)adj[i]=root(i); // 26-adjacency clusters only (before face unions)
     std::map<std::uint32_t,std::set<int>> clustersOf;std::map<std::uint32_t,int> kind; // 1 own-active, 2 ring
     for(int i=0;i<int(keys.size());++i){const auto&own=tileFaces(grid,tree,clean,memo,keys[i]).active;
        for(auto f:tileActive[i]){clustersOf[f].insert(adj[i]);kind[f]|=std::binary_search(own.begin(),own.end(),f)?1:2;}}
     std::set<int> cl;for(int i=0;i<int(keys.size());++i)cl.insert(adj[i]);
     std::size_t coupling=0,own=0,ringOnly=0,unclean=0;double maxExt=0;std::vector<std::pair<double,std::uint32_t>> big;
     for(auto&e:clustersOf)if(e.second.size()>1){++coupling;if(kind[e.first]&1)++own;else ++ringOnly;if(!clean[e.first])++unclean;
        auto fb=tree.faceBounds(e.first);double ext=std::max({fb.hi.x-fb.lo.x,fb.hi.y-fb.lo.y,fb.hi.z-fb.lo.z});maxExt=std::max(maxExt,ext);big.push_back({ext,e.first});}
     std::sort(big.rbegin(),big.rend());
     std::cout<<"couple_trace clusters="<<cl.size()<<" coupling_faces="<<coupling<<" own_active="<<own<<" ring_only="<<ringOnly<<" unclean="<<unclean<<" max_extent="<<maxExt;
     for(std::size_t k=0;k<big.size()&&k<6;++k)std::cout<<" ["<<big[k].second<<":"<<big[k].first<<" k"<<kind[big[k].second]<<" c"<<int(clean[big[k].second])<<" n"<<clustersOf[big[k].second].size()<<"]";
     std::cout<<'\n';}
#endif
    for(int i=0;i<int(keys.size());++i)for(auto f:tileActive[i]){auto [it,fresh]=faceOwner.try_emplace(f,i);if(!fresh)unite(i,it->second);}
    std::map<int,std::vector<int>> members;for(int i=0;i<int(keys.size());++i)members[root(i)].push_back(i);
    std::vector<int> roots;for(auto& m:members)roots.push_back(m.first);
    std::vector<Region> out;
    for(int r:roots){auto ids=members[r];std::sort(ids.begin(),ids.end());Region region;
        std::uint64_t h=1469598103934665603ull;auto mix=[&](std::uint64_t v){h=core::quantMix(h^v);};
        for(int i:ids){const Cell c=keys[i];region.keys.push_back(c);const core::Tile& tile=grid.tiles.at(c);region.masks.push_back(tile);
            mix(std::uint64_t(std::uint32_t(c.x))|(std::uint64_t(std::uint32_t(c.y))<<32));mix(std::uint64_t(std::uint32_t(c.z)));
            for(auto word:tile.removed)mix(word);
            const auto& f=tileFaces(grid,tree,clean,memo,c);region.faces.insert(region.faces.end(),tileActive[i].begin(),tileActive[i].end());region.selected.insert(region.selected.end(),f.selected.begin(),f.selected.end());}
        for(auto* list:{&region.faces,&region.selected}){std::sort(list->begin(),list->end());list->erase(std::unique(list->begin(),list->end()),list->end());}
        // A region's output also depends on which ring faces are selected by
        // OTHER regions; its identity includes its active face list.
        for(auto f:region.faces)mix(0x9E3779B97F4A7C15ull^f);
        region.hash=h;out.push_back(std::move(region));}
    return out; // ascending first tile key
}

// Former joinFullBody + gate statistics + packing rows, restricted to one region.
}
#include "TileRegions.h"
namespace fastwound {
#include "LowPolyDisplay.h"
inline std::string regionDefect(const Asset& asset,const RegionResult& r);

inline bool nearestSkinWeights(const core::BodyBVH& tree,Vec p,std::array<double,64>& w,std::uint32_t* seed=nullptr,
                               std::size_t* triangleTests=nullptr,std::size_t* treeNodes=nullptr,std::size_t* seedTests=nullptr,
                               const std::uint8_t* faceChain=nullptr,unsigned allowedChains=~0u,std::uint32_t* faceOut=nullptr,Vec* pointOut=nullptr,double* dist2Out=nullptr){
    if(tree.nodes.empty())return false;
    double best=1e300;std::uint32_t bestFace=~0u;Vec bestPoint;
    auto face=[&](std::uint32_t f){if(faceChain&&!((allowedChains>>faceChain[f])&1u))return;const auto& ids=tree.source.f[f];
        const Vec q=core::closestTriangle(p,tree.source.v[ids[0]].p,tree.source.v[ids[1]].p,tree.source.v[ids[2]].p);const double d=dot(q-p,q-p);
        if(d<best||(d==best&&f<bestFace)){best=d;bestFace=f;bestPoint=q;}};
    if(seed&&*seed<tree.source.f.size()){face(*seed);if(seedTests)++*seedTests;}
    auto boxDist=[&](const core::Bounds& b){double d=0;for(int a=0;a<3;++a){const double v=std::max({b.lo[a]-p[a],0.,p[a]-b.hi[a]});d+=v*v;}return d;};
    std::vector<int> stack;stack.reserve(64);stack.push_back(0);
    while(!stack.empty()){const int n=stack.back();stack.pop_back();if(treeNodes)++*treeNodes;const auto& node=tree.nodes[std::size_t(n)];if(boxDist(node.bounds)>best)continue;
        if(node.left<0){for(std::size_t i=node.first;i<node.first+node.count;++i){face(tree.order[i]);if(triangleTests)++*triangleTests;}}
        else{const double dl=boxDist(tree.nodes[std::size_t(node.left)].bounds),dr=boxDist(tree.nodes[std::size_t(node.right)].bounds);
            if(dl<dr){stack.push_back(node.right);stack.push_back(node.left);}else{stack.push_back(node.left);stack.push_back(node.right);}}}
    if(bestFace==~0u)return false;if(seed)*seed=bestFace;if(faceOut)*faceOut=bestFace;if(pointOut)*pointOut=bestPoint;if(dist2Out)*dist2Out=best;
    core::V v;v.p=bestPoint;v.b=core::faceBary(tree.source,bestFace,bestPoint);
    for(auto& b:v.b)b=std::clamp(b,0.,1.);{double sum=v.b[0]+v.b[1]+v.b[2];if(!(sum>0))return false;for(auto& b:v.b)b/=sum;}
    w=core::weights(tree.source,bestFace,v);double total=0;for(double x:w)total+=x;if(!(total>0))return false;for(double& x:w)x/=total;
    return true;
}
inline bool skin2Legacy(){static const bool v=[]{const char* e=std::getenv("GFY_SKIN2_LEGACY");return e&&std::atoi(e)!=0;}();return v;}
inline bool sideTransfer(){static const bool v=[]{const char* e=std::getenv("GFY_WALL_TRANSFER");return !e||std::atoi(e)!=0;}();return v;}
// a skin or lattice node mixes a pair of limbs when the weaker one holds at least this share (GFY_WALL_MIX_SHARE, .25): the
// crotch apex, where the mesh skin itself mixes the legs about 50/50; below it the inner-thigh skins are single-legged
// W4: limb weight in trunk tissue fades from GFY_WALL_TRUNK_FREE_MM (45) to that plus GFY_WALL_TRUNK_REACH_MM (default 0 = off; replaced by W5) outside the limb physics shape
inline double trunkFree(){static const double v=[]{const char* e=std::getenv("GFY_WALL_TRUNK_FREE_MM");return (e?std::atof(e):45.)*.001;}();return v;}
inline double trunkReach(){static const double v=[]{const char* e=std::getenv("GFY_WALL_TRUNK_REACH_MM");return (e?std::atof(e):0.)*.001;}();return v;}
// W5: a trunk node whose closest skin is an arm takes the closest trunk skin within GFY_WALL_ARM_TRUNK_MM (40; 0 = off)
inline double armTrunkReach(){static const double v=[]{const char* e=std::getenv("GFY_WALL_ARM_TRUNK_MM");return (e?std::atof(e):40.)*.001;}();return v;}
inline double wallMixShare(){static const double v=[]{const char* e=std::getenv("GFY_WALL_MIX_SHARE");return e?std::atof(e):.25;}();return v;}
struct LimbRegions {
    std::array<std::uint8_t,64> boneChain{};  // chain of each source bone's physical element (0 trunk / unknown)
    std::vector<std::uint8_t> faceChain;      // chain of each source face's dominant bone
    // element owning the tissue at p (bone id, 255 none), its chain, the limb chains (bits) whose tissue lies within
    // limbChainMargin of the nearest limb's, and how much farther (score, m) the nearest limb is than the owning element
    std::function<void(const Vec&,std::uint8_t&,std::uint8_t&,unsigned&,double&)> classify;
    // skin2: lowest element score per chain at p (index 0: trunk elements; signed distance, m), the owning element and its
    // chain; the rest anchor of every bone and its physical parent (255: none), for the wall refinement's error bound; the
    // sibling of each chain (the other leg / arm: same parent element, same length; 0 none) and each limb bone's match in it
    std::function<void(const Vec&,std::array<double,32>&,std::uint8_t&,std::uint8_t&)> chainScores;
    std::array<Vec,64> pivot{};std::array<std::uint8_t,64> parent{},mirrorBone{};std::array<std::uint8_t,32> sibling{},fromTop{};
    unsigned chains{};std::size_t faces{};
};
// skin2: the limb side of a point: the limb chain whose tissue it is in, else (trunk) the nearest limb chain (0: none)
inline std::uint8_t limbSide(const LimbRegions& L,const Vec& p){std::array<double,32> sc;std::uint8_t e=255,c=0;L.chainScores(p,sc,e,c);if(c)return c;
    double m=std::numeric_limits<double>::infinity();std::uint8_t best=0;for(unsigned k=1;k<=L.chains&&k<32;++k)if(sc[k]<m){m=sc[k];best=std::uint8_t(k);}return best;}
// skin2: keep the side's limb and the trunk; the side's sibling limb moves its weight to the matching bones of the side
// (GFY_WALL_TRANSFER=0: dropped, as round 1); other limbs are dropped. Renormalized; false when nothing is left.
inline bool gateToSide(const LimbRegions& L,std::uint8_t side,std::array<double,64>& w){
    for(unsigned b=0;b<64;++b){const auto c=L.boneChain[b];if(!c||c==side||w[b]==0)continue;
        if(sideTransfer()&&L.sibling[c]==side&&L.mirrorBone[b]<64)w[L.mirrorBone[b]]+=w[b];w[b]=0;}
    double kept=0;for(double x:w)kept+=x;if(!(kept>0))return false;for(double& x:w)x/=kept;return true;}
inline constexpr double limbSkinReach=.02,limbForeignMax=.01,limbAdoptGap=.03;
// trunk points keep the limbs whose tissue score is within this of the nearest limb's (GFY_LIMB_MARGIN_MM, default 0:
// only the nearest limb; 10/40 mm left mixed both-leg walls at the crotch that stretch across when the legs part)
inline double limbChainMargin(){static const double v=[]{const char* e=std::getenv("GFY_LIMB_MARGIN_MM");return (e?std::atof(e):0.)*.001;}();return v;}
inline bool wallSkinLegacy(){static const bool v=[]{const char* e=std::getenv("GFY_WALL_SKIN_LEGACY");return e&&std::atoi(e)!=0;}();return v;}
// p on the inner side of face f's plane (outward normal cross(b-a,c-a), turned to agree with the face's shading normals)
template<class S> inline bool behindFace(const S& s,std::uint32_t f,const Vec& p,const Vec& q){
    const auto& ids=s.f[f];const Vec a=s.v[ids[0]].p,b=s.v[ids[1]].p,c=s.v[ids[2]].p;Vec n=cross(b-a,c-a);
    if(dot(n,s.v[ids[0]].normal+s.v[ids[1]].normal+s.v[ids[2]].normal)<0)n=n*-1.;
    return dot(p-q,n)<=0;}
// A WallWeights node: the closest skin unless the node lies inside the body (anatomy or render closure) and that skin is not
// the skin over it (farther than limbSkinReach, or the node stands in front of the face); then the influences of limbs other
// than the node's own are dropped (a node in a limb element: its limb + trunk; a trunk node: the nearest limb, limbs within
// limbChainMargin of it, trunk). More than limbForeignMax dropped -> the closest skin among faces of the allowed chains,
// filtered (a trunk node within limbAdoptGap of its nearest limb: among that limb's faces first - the crotch apex follows
// its own side's thigh on both sides of the midline, not the pelvis on one and the thigh on the other); none -> rigid on
// the node's element. Signed (negative) skins keep the closest skin.
inline bool wallNodeSkin(const Asset& asset,const LimbRegions* limbs,const Vec& p,std::array<double,64>& w,std::uint32_t* seed=nullptr,
                         std::size_t* triangleTests=nullptr,std::size_t* treeNodes=nullptr,std::size_t* seedTests=nullptr){
    std::uint32_t face=~0u;Vec q;double d2=0;
    if(!nearestSkinWeights(asset.source_tree,p,w,seed,triangleTests,treeNodes,seedTests,nullptr,~0u,&face,&q,&d2))return false;
    if(!limbs||wallSkinLegacy()||gfy_signed_hw::negative(w))return true;
    const bool inside=(asset.base.anatomy&&asset.base.anatomy->point(p.x,p.y,p.z))||(asset.closure_field&&asset.closure_field->sample(p.x,p.y,p.z)!=0u);
    // skin2 W5: a node in trunk tissue whose closest skin belongs to an arm (the sleeve over the armpit; arms = limb pairs not
    // hanging from the top element) takes the closest trunk skin within armTrunkReach instead: walls in the chest under the
    // armpit followed the sleeve (10-25 % upper arm) and stood out of the vest when the arm went up (S7 armsup). Legs keep
    // their skin: the crotch apex skin itself mixes the legs, and walls there must follow it (S7 crouch, stride).
    if(inside&&!skin2Legacy()&&armTrunkReach()>0&&face<limbs->faceChain.size()){const auto fc=limbs->faceChain[face];
        if(fc&&fc<32&&!limbs->fromTop[fc]){std::uint8_t own=255,ownChain=0;unsigned near=0;double gap=0;limbs->classify(p,own,ownChain,near,gap);
            if(!ownChain){std::array<double,64> c{};std::uint32_t f2=~0u;Vec q2;double e2=0;
                if(nearestSkinWeights(asset.source_tree,p,c,nullptr,triangleTests,treeNodes,nullptr,limbs->faceChain.data(),1u,&f2,&q2,&e2)&&!gfy_signed_hw::negative(c)
                   &&e2<=armTrunkReach()*armTrunkReach()){w=c;return true;}}}}
    if(!inside||(d2<=limbSkinReach*limbSkinReach&&behindFace(asset.source_tree.source,face,p,q)))return true;
    std::uint8_t own=255,ownChain=0;unsigned near=0;double gap=1e300;limbs->classify(p,own,ownChain,near,gap);
    const unsigned allowed=1u|(ownChain?(1u<<ownChain):near);
    double foreign=0;for(unsigned b=0;b<64;++b)if(w[b]>0&&!((allowed>>limbs->boneChain[b])&1u))foreign+=w[b];
    if(foreign<=limbForeignMax)return true;
    const bool adopt=!ownChain&&near&&!(near&(near-1u))&&gap<=limbAdoptGap;
    for(int pass=adopt?0:1;pass<2;++pass){std::array<double,64> c{};
        if(nearestSkinWeights(asset.source_tree,p,c,nullptr,triangleTests,treeNodes,nullptr,limbs->faceChain.data(),pass==0?near:allowed)&&!gfy_signed_hw::negative(c)){
            double kept=0;for(unsigned b=0;b<64;++b){if(!((allowed>>limbs->boneChain[b])&1u))c[b]=0;kept+=c[b];}
            if(kept>0){for(double& x:c)x/=kept;w=c;return true;}}}
    if(own<64){w.fill(0.);w[own]=1.;}
    return true;
}
inline constexpr double limbTearShare=.05;
// skin2: the limb a skin leans to: more than limbTearShare of it in one limb chain while that chain's sibling (the other
// leg / arm) holds no more than limbTearShare; a skin mixing both (the crotch apex: the mesh skin itself mixes the legs
// there) or no limb -> 0. The tear and the exact tear line work between single-sided corners only.
inline std::uint8_t leanSide(const LimbRegions& L,const std::array<double,64>& w){if(gfy_signed_hw::negative(w))return 0;double share[32]{};
    for(unsigned b=0;b<64;++b)if(w[b]>0)share[L.boneChain[b]&31u]+=w[b];unsigned best=0;double bs=0;for(unsigned c=1;c<32;++c)if(share[c]>bs){bs=share[c];best=c;}
    if(!(bs>limbTearShare))return 0;const auto sib=L.sibling[best];if(sib&&share[sib]>=wallMixShare())return 0;return std::uint8_t(best);}
inline void tearLimbContacts(const LimbRegions& limbs,const std::vector<core::Vec>& p,const std::vector<std::array<std::uint32_t,3>>& tris,
    const std::vector<std::array<double,64>>& w,std::unordered_map<std::size_t,std::array<std::array<double,64>,3>>& torn){
    if(wallSkinLegacy())return;
    std::vector<std::uint8_t> dom(p.size(),0);
    for(std::size_t i=0;i<p.size();++i){if(gfy_signed_hw::negative(w[i])){dom[i]=255;continue;}double share[32]{};
        for(unsigned b=0;b<64;++b)if(w[i][b]>0)share[limbs.boneChain[b]&31u]+=w[i][b];
        unsigned best=0;double bs=0;for(unsigned c=1;c<32;++c)if(share[c]>bs){bs=share[c];best=c;}dom[i]=bs>limbTearShare?std::uint8_t(best):0;}
    for(std::size_t t=0;t<tris.size();++t){const auto& tr=tris[t];const std::uint8_t d[3]={dom[tr[0]],dom[tr[1]],dom[tr[2]]};
        if(d[0]==255||d[1]==255||d[2]==255)continue;
        std::uint8_t a=0,b=0;for(auto x:d)if(x){if(!a)a=x;else if(x!=a)b=x;}
        if(!b)continue;
        std::uint8_t own=255,ownChain=0;unsigned near=0;double gap=0;limbs.classify((p[tr[0]]+p[tr[1]]+p[tr[2]])*(1./3),own,ownChain,near,gap);
        std::uint8_t keep=0;for(auto x:d)if(x&&x==ownChain)keep=x;
        if(!keep){double sa=0,sb=0;for(int k=0;k<3;++k)for(unsigned bb=0;bb<64;++bb){const auto ch=limbs.boneChain[bb];const double x=w[tr[k]][bb];if(x>0){if(ch==a)sa+=x;else if(ch==b)sb+=x;}}keep=sa>=sb?a:b;}
        std::array<std::array<double,64>,3> g{};std::array<bool,3> ownSide{};double rich=-1;int richest=0;
        for(int k=0;k<3;++k){double s=0,inKeep=0;for(unsigned bb=0;bb<64;++bb){const auto ch=limbs.boneChain[bb];const double x=w[tr[k]][bb];if(ch&&ch!=keep)continue;g[k][bb]=x;s+=x;if(ch==keep)inKeep+=x;}
            ownSide[std::size_t(k)]=s>=.5;if(s>0)for(double& x:g[k])x/=s;else g[k].fill(0.);if(inKeep>rich){rich=inKeep;richest=k;}}
        for(int k=0;k<3;++k)if(!ownSide[std::size_t(k)])g[k]=g[std::size_t(richest)]; // a corner mostly in the other limb
        torn.emplace(t,g);}
}
// skin2 (2026-10-29): walls torn where two limbs' sides meet (thighs touching below the crotch apex, the midline under it).
// A triangle whose corners lean to different limbs (leanSide: single-sided corners only) takes the side of its centroid
// (limbSide: the limb whose tissue it is in, else the nearest limb; one of the corners' limbs, else the one with more weight
// over the corners); each corner leaning to another limb is re-skinned for that side from its own lattice nodes (W1 blend
// forced to that side). A corner whose nodes hold nothing of the side takes the skin of the triangle's corner richest in the
// side (round 1's rule, which round 1 used for every such corner). After the exact cut (ChannelHistoryAdapter refineWalls)
// the re-skinned corners lie on the side boundary, so the torn edge is the contact; packWithField's row dedupe gives each
// side its own copy of the shared corners.
template<class Regate> inline void tearLimbSides(const LimbRegions& limbs,const std::vector<core::Vec>& p,const std::vector<std::array<std::uint32_t,3>>& tris,
    const std::vector<std::array<double,64>>& w,std::unordered_map<std::size_t,std::array<std::array<double,64>,3>>& torn,Regate&& regate){
    if(wallSkinLegacy()||!limbs.chainScores)return;
    std::vector<std::uint8_t> dom(p.size(),0);for(std::size_t i=0;i<p.size();++i)dom[i]=leanSide(limbs,w[i]);
    for(std::size_t t=0;t<tris.size();++t){const auto& tr=tris[t];const std::uint8_t d[3]={dom[tr[0]],dom[tr[1]],dom[tr[2]]};
        std::uint8_t a=0,b=0;for(auto x:d)if(x){if(!a)a=x;else if(x!=a)b=x;}
        if(!b)continue;
        std::uint8_t side=limbSide(limbs,(p[tr[0]]+p[tr[1]]+p[tr[2]])*(1./3));
        if(side!=a&&side!=b){double sa=0,sb=0;for(int k=0;k<3;++k)for(unsigned bb=0;bb<64;++bb){const auto ch=limbs.boneChain[bb];const double x=w[tr[k]][bb];if(x>0){if(ch==a)sa+=x;else if(ch==b)sb+=x;}}side=sa>=sb?a:b;}
        std::array<std::array<double,64>,3> g{};std::array<bool,3> failed{};double rich=-1;int richest=-1;
        for(int k=0;k<3;++k){g[std::size_t(k)]=w[tr[k]];
            if(d[k]&&d[k]!=side){if(!regate(p[tr[k]],side,g[std::size_t(k)]))failed[std::size_t(k)]=true;}
            else{double s=0;for(unsigned bb=0;bb<64;++bb)if(limbs.boneChain[bb]==side)s+=w[tr[k]][bb];if(s>rich){rich=s;richest=k;}}}
        for(int k=0;k<3;++k)if(failed[std::size_t(k)]){if(richest>=0){g[std::size_t(k)]=w[tr[std::size_t(richest)]];(void)gateToSide(limbs,side,g[std::size_t(k)]);}
            else{g[std::size_t(k)]=w[tr[k]];if(!gateToSide(limbs,side,g[std::size_t(k)]))g[std::size_t(k)]=w[tr[k]];}}
        torn.emplace(t,g);}
}
struct WallWeights {
    const Asset& asset;static constexpr double h=.02;
    std::unordered_map<std::uint64_t,std::array<std::pair<std::uint16_t,float>,6>> local;
    std::shared_ptr<const LimbRegions> limbs;
    std::uint32_t nodeSeed=~0u; // skin2 S1: the last face node() found (exact initial bound of the next search)
    explicit WallWeights(const Asset& a):asset(a),limbs(a.limbRegions()){}
    static std::uint64_t key(int x,int y,int z){auto u=[](int v){return std::uint64_t(std::uint32_t(v+(1<<20)))&0x1FFFFFull;};return (u(x)<<42)|(u(y)<<21)|u(z);}
    const std::array<std::pair<std::uint16_t,float>,6>& node(int x,int y,int z){
        const auto k=key(x,y,z);auto it=local.find(k);if(it!=local.end())return it->second;
        {std::lock_guard<std::mutex> lock(asset.skin_nodes.mutex);auto g=asset.skin_nodes.map.find(k);if(g!=asset.skin_nodes.map.end())return local.emplace(k,g->second).first->second;}
        std::array<std::pair<std::uint16_t,float>,6> top{};std::array<double,64> full{};
        if(wallNodeSkin(asset,limbs.get(),Vec{x*h,y*h,z*h},full,&nodeSeed)){ // skin2 S1: coherent seed
            std::array<unsigned,64> order;for(unsigned b=0;b<64;++b)order[b]=b;const bool signedSkin=gfy_signed_hw::negative(full);std::stable_sort(order.begin(),order.end(),[&](unsigned a,unsigned b){return signedSkin?std::abs(full[a])>std::abs(full[b]):full[a]>full[b];});
            double sum=0;for(int j=0;j<6;++j){top[std::size_t(j)]={std::uint16_t(order[std::size_t(j)]),float(full[order[std::size_t(j)]])};sum+=top[std::size_t(j)].second;}
            if(sum>0)for(auto& e:top)e.second=float(e.second/sum);}
        {std::lock_guard<std::mutex> lock(asset.skin_nodes.mutex);if(asset.skin_nodes.map.size()>=200000)asset.skin_nodes.map.clear();top=asset.skin_nodes.map.try_emplace(k,top).first->second;}
        return local.emplace(k,top).first->second;}
    bool sample(const Vec& p,std::array<double,64>& w){
        if(sided())return sampleNodes(p,w,[&](int x,int y,int z)->const Top*{return &node(x,y,z);},-1); // skin2 W1
        w.fill(0.);const double fx=p.x/h,fy=p.y/h,fz=p.z/h;const int x=int(std::floor(fx)),y=int(std::floor(fy)),z=int(std::floor(fz));
        const double tx=fx-x,ty=fy-y,tz=fz-z;double total=0;
        for(int c=0;c<8;++c){const int dx=c&1,dy=(c>>1)&1,dz=(c>>2)&1;const double k=(dx?tx:1-tx)*(dy?ty:1-ty)*(dz?tz:1-tz);if(k<=0)continue;
            for(const auto& e:node(x+dx,y+dy,z+dz))if(e.second!=0.f){w[e.first]+=k*e.second;total+=k*e.second;}}
        if(!(total>0))return false;for(double& v:w)v/=total;gateMixedLimbs(p,w);return true;}
    void gateMixedLimbs(const Vec& p,std::array<double,64>& w)const{if(!limbs||wallSkinLegacy()||gfy_signed_hw::negative(w))return;
        unsigned present=0;for(unsigned b=0;b<64;++b)if(w[b]>.01&&limbs->boneChain[b])present|=1u<<(limbs->boneChain[b]&31u);if(!(present&(present-1)))return;
        std::uint8_t own=255,ownChain=0;unsigned near=0;double gap=0;limbs->classify(p,own,ownChain,near,gap);const unsigned allowed=1u|(ownChain?(1u<<ownChain):near);
        double kept=0;for(unsigned b=0;b<64;++b){if(!((allowed>>limbs->boneChain[b])&1u))w[b]=0;kept+=w[b];}
        if(kept>0){for(double& v:w)v/=kept;}else if(own<64){w.fill(0.);w[own]=1.;}}
    // skin2 W1 (2026-10-29): the trilinear blend of the 8 lattice nodes around p, with one change: a node whose limb weight lies
    // in ONE limb of a pair (left/right leg or arm) that is not the side of p (the limb whose tissue p is in, else the nearest
    // limb: limbSide) gives that weight to the matching bones of p's side. Where 2 cm cells straddle touching thighs or the
    // crotch midline, nodes hold the other leg's skin (the trouser legs' inner walls interleave there); round 1 dropped the
    // other limb per sample (E2d), which left corners 4 mm apart with 0 % and 30 % thigh just right of the midline (slivers when
    // the hips flex). Nodes that mix both legs (the crotch apex, where the skin itself mixes them) are blended as they are.
    // forced >= 0: blend for that side (the tear re-skins a corner for its triangle's side). Without limb data, with signed
    // skins or with GFY_SKIN2_LEGACY=1: the round-1 sample (blend, then gateMixedLimbs).
    using Top=std::array<std::pair<std::uint16_t,float>,6>;
    bool sided()const{return limbs&&limbs->chainScores&&!skin2Legacy()&&!wallSkinLegacy();}
    template<class Get> bool sampleNodes(const Vec& p,std::array<double,64>& w,Get&& get,int forced)const{
        w.fill(0.);const double fx=p.x/h,fy=p.y/h,fz=p.z/h;const int x=int(std::floor(fx)),y=int(std::floor(fy)),z=int(std::floor(fz));
        const double tx=fx-x,ty=fy-y,tz=fz-z;std::array<const Top*,8> nd{};std::array<double,8> kk{};
        for(int c=0;c<8;++c){const int dx=c&1,dy=(c>>1)&1,dz=(c>>2)&1;const double k=(dx?tx:1-tx)*(dy?ty:1-ty)*(dz?tz:1-tz);if(k<=0)continue;
            nd[std::size_t(c)]=get(x+dx,y+dy,z+dz);if(!nd[std::size_t(c)])return false;kk[std::size_t(c)]=k;}
        return blendSided(p,nd,kk,w,forced);}
    bool blendSided(const Vec& p,const std::array<const Top*,8>& nd,const std::array<double,8>& kk,std::array<double,64>& w,int forced)const{
        w.fill(0.);bool signedSkin=false,paired=false,anyLimb=false;std::array<unsigned,8> single{};
        for(std::size_t i=0;i<8;++i){if(!nd[i])continue;double share[32]{};
            for(const auto& e:*nd[i]){if(e.second<0.f)signedSkin=true;if(e.second>0.f&&e.first<64)share[limbs->boneChain[e.first]&31u]+=e.second;}
            unsigned c=0;double cs=0;for(unsigned q=1;q<32;++q)if(share[q]>cs){cs=share[q];c=q;}
            if(cs>.01&&limbs->sibling[c]&&share[limbs->sibling[c]]<wallMixShare())single[i]=c;
            if(cs>.01)anyLimb=true;}
        // classify (the costly part of a sample) only when single-sided nodes of both limbs of a pair meet in this cell
        {unsigned seen=0;for(std::size_t i=0;i<8;++i)if(single[i])seen|=1u<<single[i];
         for(unsigned c=1;c<32&&!paired;++c)if(((seen>>c)&1u)&&limbs->sibling[c]&&((seen>>limbs->sibling[c])&1u))paired=true;}
        // the side, and (W4) in trunk tissue how far each limb's physics shape is
        int side=forced;std::array<double,32> sc;std::uint8_t own=255,ownChain=0;bool scored=false;
        if(side<0&&!signedSkin&&(paired||(anyLimb&&trunkReach()>0))){limbs->chainScores(p,sc,own,ownChain);scored=true;
            if(ownChain)side=ownChain;else{double m=std::numeric_limits<double>::infinity();side=0;for(unsigned q=1;q<=limbs->chains&&q<32;++q)if(sc[q]<m){m=sc[q];side=int(q);}}}
        if(side<0||signedSkin)side=0;
        double total=0;
        for(std::size_t i=0;i<8;++i){if(!nd[i])continue;const unsigned c=single[i];
            const bool move=side>0&&c&&int(c)!=side&&limbs->sibling[c]==unsigned(side)&&sideTransfer();
            for(const auto& e:*nd[i])if(e.second!=0.f){unsigned b=e.first;if(move&&b<64&&limbs->boneChain[b]==c&&limbs->mirrorBone[b]<64)b=limbs->mirrorBone[b];
                w[b]+=kk[i]*e.second;total+=kk[i]*e.second;}}
        if(!(total>0))return false;for(double& v:w)v/=total;
        // W4 trunk reach: in trunk tissue a limb's weight fades out with the distance from that limb's physics shape (free up to
        // GFY_WALL_TRUNK_FREE_MM, gone GFY_WALL_TRUNK_REACH_MM further): walls in the chest under a raised arm followed the
        // sleeve's skin (10-25 % upper arm 15 cm below the shoulder) and stood out of the vest when the arm went up
        if(scored&&!ownChain&&trunkReach()>0){double kept=0;std::array<double,32> f;for(unsigned q=0;q<32;++q){const double d=q?sc[q]-trunkFree():0.;f[q]=q&&!limbs->fromTop[q]?std::clamp(1.-std::max(0.,d)/trunkReach(),0.,1.):1.;}
            std::array<double,64> g=w;for(unsigned b=0;b<64;++b){g[b]*=f[limbs->boneChain[b]&31u];kept+=g[b];}if(kept>0){for(double& v:g)v/=kept;w=g;}}
        return true;}
    // skin2: sampleCached blended for a given side (cached nodes only; false when a node is missing)
    bool sampleCachedSide(const Vec& p,std::array<double,64>& w,bool& cached,int side)const{cached=true;
        const bool ok=sampleNodes(p,w,[&](int x,int y,int z)->const Top*{const auto it=local.find(key(x,y,z));if(it==local.end()){cached=false;return nullptr;}return &it->second;},side);
        return ok&&cached;}
    // R64: sample() over the local cache only (read-only, safe beside other readers); cached=false when a node is missing
    bool sampleCached(const Vec& p,std::array<double,64>& w,bool& cached)const{
        if(sided()){cached=true;return sampleNodes(p,w,[&](int x,int y,int z)->const Top*{const auto it=local.find(key(x,y,z));if(it==local.end()){cached=false;return nullptr;}return &it->second;},-1);} // skin2 W1
        cached=true;w.fill(0.);const double fx=p.x/h,fy=p.y/h,fz=p.z/h;const int x=int(std::floor(fx)),y=int(std::floor(fy)),z=int(std::floor(fz));
        const double tx=fx-x,ty=fy-y,tz=fz-z;double total=0;
        for(int c=0;c<8;++c){const int dx=c&1,dy=(c>>1)&1,dz=(c>>2)&1;const double k=(dx?tx:1-tx)*(dy?ty:1-ty)*(dz?tz:1-tz);if(k<=0)continue;
            const auto it=local.find(key(x+dx,y+dy,z+dz));if(it==local.end()){cached=false;return false;}
            for(const auto& e:it->second)if(e.second!=0.f){w[e.first]+=k*e.second;total+=k*e.second;}}
        if(!(total>0))return false;for(double& v:w)v/=total;gateMixedLimbs(p,w);return true;}
};
inline bool directDisplay(){static const bool v=[]{const char* e=std::getenv("GFY_DIRECT");return !e||std::atoi(e)!=0;}();return v;}
// Skin: the tile-local exact cut, fan-triangulated per fragment, exact interpolated skin (canonical lattice skin
// where fixed). Walls: every unit voxel face between a removed cell and a tissue cell, facing into the cavity,
// position-only skin. No envelope clip, no weld, no skin solve, no gate: the wall reaches up to the last tissue
// voxel (a lip of at most one cell around the skin opening, never a gap into the body).
inline core::IndexedJoin buildDirect(const Asset& asset,const Grid& grid,const core::Source& patch,const core::ClipResult& outside,
                                     const std::vector<core::Quad>& quads,const std::unordered_map<core::Quant,std::array<double,64>,core::QuantHash>& fixedSkin,
                                     double& tissueCells,std::size_t& wallQuads){
    core::IndexedJoin out;auto& store=out.extra;core::StageProfile dstages("direct");
    for(const auto& fragment:outside.polygons){const auto& poly=fragment.polygon;if(poly.size()<3)continue;
        std::vector<core::JoinedVertex*> vs;vs.reserve(poly.size());
        for(const auto& v:poly){auto j=core::interpolateVertex(patch,fragment.sourceFace,v);j.p=v.p;
            auto fixed=fixedSkin.find(core::quantExact(floatRounded(v.p)));if(fixed!=fixedSkin.end()&&!j.hardware_corner)j.w=fixed->second;
            store.push_back(j);vs.push_back(&store.back());}
        // Orient by the source face (fragments come in either order); drop slivers whose float corners flip.
        const auto& ids=patch.f[fragment.sourceFace];const Vec fn=cross(patch.v[ids[1]].p-patch.v[ids[0]].p,patch.v[ids[2]].p-patch.v[ids[0]].p);
        Vec pn;for(std::size_t k=0;k<vs.size();++k){const Vec a=vs[k]->p,b=vs[(k+1)%vs.size()]->p;pn=pn+Vec{(a.y-b.y)*(a.z+b.z),(a.z-b.z)*(a.x+b.x),(a.x-b.x)*(a.y+b.y)};}
        if(dot(pn,fn)<0)std::reverse(vs.begin(),vs.end());
        for(std::size_t k=1;k+1<vs.size();++k){const Vec a=floatRounded(vs[0]->p),b=floatRounded(vs[k]->p),c=floatRounded(vs[k+1]->p);const Vec m=cross(b-a,c-a);
            if(length(m)<1e-14||dot(m,fn)<=0)continue;
            core::IndexedTriangle t;t.material=0;t.v={vs[0],vs[k],vs[k+1]};out.triangles.push_back(t);}}
    dstages.mark("skin");WallWeights weights(asset);
    struct VK{core::Quant q;unsigned m;bool operator==(const VK& o)const{return q==o.q&&m==o.m;}};
    struct VKH{std::size_t operator()(const VK& k)const{return std::size_t(core::quantMix(std::uint64_t(core::QuantHash{}(k.q))^(std::uint64_t(k.m)<<56)));}};
    core::FlatMap<VK,core::JoinedVertex*,VKH> wallVertex;wallVertex.reserve(quads.size());
    out.triangles.reserve(quads.size()*2);
    std::vector<core::JoinedVertex*> c;c.reserve(8);core::Poly quadPoly;quadPoly.reserve(4);
    // A wall polygon (cavity-facing winding) fanned into triangles, corners welded by position and material.
    auto emit=[&](const core::Poly& poly,unsigned material){if(poly.size()<3)return;
        Vec pn;for(std::size_t k=0;k<poly.size();++k){const Vec a=poly[k].p,b=poly[(k+1)%poly.size()].p;pn=pn+Vec{(a.y-b.y)*(a.z+b.z),(a.z-b.z)*(a.x+b.x),(a.x-b.x)*(a.y+b.y)};}
        const double len=length(pn);if(len<1e-14)return;
        const Vec nn=pn*(1/len);const Vec T=std::abs(nn.x)>.9?Vec{0,0,1}:Vec{1,0,0};const Vec B=cross(nn,T);
        c.resize(poly.size());
        for(std::size_t k=0;k<poly.size();++k){const VK key{core::quantExact(poly[k].p),material};auto it=wallVertex.find(key);
            if(it==wallVertex.end()){core::JoinedVertex j;j.p=poly[k].p;j.normal=nn;j.tangent=T;j.binormal=B;j.sourceFace=~std::uint32_t(0);
                j.uv={dot(j.p,T)/.08,dot(j.p,B)/.08}; /* wall texture tile 8 cm */if(!weights.sample(j.p,j.w))throw core::packing::PackingInfluenceError("wall_skin_unavailable");
                store.push_back(j);it=wallVertex.try_emplace(key,&store.back()).first;}
            c[k]=it->second;}
        for(std::size_t k=1;k+1<c.size();++k){if(length(cross(c[k]->p-c[0]->p,c[k+1]->p-c[0]->p))<1e-14)continue;
            core::IndexedTriangle t;t.material=material;t.v={c[0],c[k],c[k+1]};out.triangles.push_back(t);++out.stats.cavityTriangles;}};
    // Every voxel face between a removed cell and a kept cell (tissue OR air, e.g. the gap between a garment and
    // the body) that lies inside the closed envelope. A face the envelope touches is clipped by its planes exactly
    // as the join does (the wall ends on the skin surface: no lip, no gap to the cut). The other faces form
    // edge-connected patches that no envelope face crosses, each wholly inside or outside: one inside() test each.
    const auto& es=asset.envelope_tree.source;
    std::unordered_map<std::uint64_t,std::shared_ptr<const std::vector<std::uint32_t>>> tileCache;
    auto floorDiv=[](int x,int d){return x>=0?x/d:-((-x+d-1)/d);};
    auto tileFaces=[&](Cell t)->const std::vector<std::uint32_t>&{
        const std::uint64_t key=WallWeights::key(t.x,t.y,t.z);auto it=tileCache.find(key);if(it!=tileCache.end())return *it->second;
        std::shared_ptr<const std::vector<std::uint32_t>> list;
        {std::lock_guard<std::mutex> lock(asset.envelope_tiles.mutex);auto g=asset.envelope_tiles.map.find(key);if(g!=asset.envelope_tiles.map.end())list=g->second;}
        if(!list){core::Bounds box;box.lo=grid.origin+Vec{double(t.x*8),double(t.y*8),double(t.z*8)}*grid.pitch;box.hi=grid.origin+Vec{double(t.x*8+8),double(t.y*8+8),double(t.z*8+8)}*grid.pitch;
            for(int a=0;a<3;++a){box.lo[a]-=1e-9;box.hi[a]+=1e-9;}
            std::vector<std::uint32_t> candidates,faces;asset.envelope_tree.query(0,box,candidates);std::sort(candidates.begin(),candidates.end());
            for(auto f:candidates){const auto& ids=es.f[f];if(triangleBoxOverlap(es.v[ids[0]].p,es.v[ids[1]].p,es.v[ids[2]].p,box.lo,box.hi))faces.push_back(f);}
            list=std::make_shared<const std::vector<std::uint32_t>>(std::move(faces));
            std::lock_guard<std::mutex> lock(asset.envelope_tiles.mutex);if(asset.envelope_tiles.map.size()>=100000)asset.envelope_tiles.map.clear();list=asset.envelope_tiles.map.try_emplace(key,list).first->second;}
        return *tileCache.emplace(key,list).first->second;};
    // Envelope faces touching a quad (closed), ascending.
    std::vector<std::uint32_t> touchFaces,touchFirst{0};
    auto touched=[&](const core::Quad& q){const auto before=touchFaces.size();
        const Cell t{floorDiv(q.removedCell.x,8),floorDiv(q.removedCell.y,8),floorDiv(q.removedCell.z,8)};
        core::Bounds qb;for(auto p:q.p)qb.add(p);
        for(auto f:tileFaces(t)){const auto& ids=es.f[f];const Vec A=es.v[ids[0]].p,B=es.v[ids[1]].p,C=es.v[ids[2]].p;const Vec n=cross(B-A,C-A);
            double lo=1e300,hi=-1e300;for(const auto& p:q.p){const double d=dot(p-A,n);lo=std::min(lo,d);hi=std::max(hi,d);}
            if(lo>0||hi<0)continue;if(triangleBoxOverlap(A,B,C,qb.lo,qb.hi))touchFaces.push_back(f);}
        if(touchFaces.size()==before)return false;touchFirst.push_back(std::uint32_t(touchFaces.size()));return true;};
    // Per tile first: a tile no envelope face touches lies wholly inside (its quads are walls) or outside (no walls),
    // decided by one inside() per tile per asset; only the free quads of tiles the envelope crosses form components.
    auto tileSide=[&](Cell t)->signed char{const std::uint64_t key=WallWeights::key(t.x,t.y,t.z);
        {std::lock_guard<std::mutex> lock(asset.envelope_tiles.mutex);auto g=asset.envelope_tiles.sides.find(key);if(g!=asset.envelope_tiles.sides.end())return g->second;}
        const Vec centre=grid.origin+Vec{double(t.x*8+4),double(t.y*8+4),double(t.z*8+4)}*grid.pitch;const signed char s=asset.envelope_tree.inside(centre)?1:0;
        std::lock_guard<std::mutex> lock(asset.envelope_tiles.mutex);if(asset.envelope_tiles.sides.size()>=200000)asset.envelope_tiles.sides.clear();asset.envelope_tiles.sides[key]=s;return s;};
    std::vector<core::Quad> nearEnvelope;std::vector<std::uint32_t> mixedFree;std::vector<std::uint8_t> kind(quads.size(),0); // 0 out, 1 in, 2 near, 3 free quad of a mixed tile
    {Cell cur{INT_MIN,INT_MIN,INT_MIN};signed char curSide=0;
     for(std::uint32_t i=0;i<quads.size();++i){const auto& q=quads[i];const Cell t{floorDiv(q.removedCell.x,8),floorDiv(q.removedCell.y,8),floorDiv(q.removedCell.z,8)};
        if(!(t==cur)){cur=t;curSide=tileFaces(t).empty()?tileSide(t):-1;}
        if(curSide>=0){kind[i]=std::uint8_t(curSide);continue;}
        if(touched(q)){kind[i]=2;nearEnvelope.push_back(q);}else{kind[i]=3;mixedFree.push_back(i);}}}
    dstages.mark("touch");
    {std::vector<std::uint32_t> parent(mixedFree.size());for(std::uint32_t k=0;k<parent.size();++k)parent[k]=k;
     auto find=[&](std::uint32_t k){while(parent[k]!=k){parent[k]=parent[parent[k]];k=parent[k];}return k;};
     // Quad corners are lattice points: an edge key is its lower lattice corner (17 bits per axis) and its axis.
     struct EdgeKeyHash {std::size_t operator()(std::uint64_t k)const{return std::size_t(core::quantMix(k));}};
     const double invPitch=1/grid.pitch;
     auto ekey=[&](const Vec& p,const Vec& r){std::uint64_t key=0;int ax=0;for(int a=0;a<3;++a){const std::uint64_t x=std::uint64_t(std::llround((p[a]-grid.origin[a])*invPitch)),y=std::uint64_t(std::llround((r[a]-grid.origin[a])*invPitch));if(x!=y)ax=a;key=(key<<17)|std::min(x,y);}return (key<<2)|std::uint64_t(ax);};
     core::FlatMap<std::uint64_t,std::uint32_t,EdgeKeyHash> edgeOwner;edgeOwner.reserve(mixedFree.size()*4);
     for(std::uint32_t k=0;k<mixedFree.size();++k){const auto& q=quads[mixedFree[k]];
        for(int e=0;e<4;++e){auto it=edgeOwner.try_emplace(ekey(q.p[std::size_t(e)],q.p[std::size_t((e+1)%4)]),k);
            if(!it.second){const auto a=find(k),b=find(it.first->second);if(a!=b)parent[std::max(a,b)]=std::min(a,b);}}}
     std::vector<signed char> side(mixedFree.size(),-1);
     for(std::uint32_t k=0;k<mixedFree.size();++k){const auto root=find(k);
        if(side[root]<0){const auto& q=quads[mixedFree[k]];side[root]=asset.envelope_tree.inside((q.p[0]+q.p[1]+q.p[2]+q.p[3])*.25)?1:0;}
        if(!side[root])kind[mixedFree[k]]=0;}
     for(std::uint32_t i=0;i<quads.size();++i){if(kind[i]!=1&&kind[i]!=3)continue;const auto& q=quads[i];
        ++wallQuads;quadPoly.clear();for(int cc=3;cc>=0;--cc)quadPoly.push_back({q.p[std::size_t(cc)],{},q.material});emit(quadPoly,q.material);}}
    dstages.mark("far_walls");
#ifdef GFY_VOXEL_STAGE_PROFILE
    std::cout<<"direct_walls quads="<<quads.size()<<" near="<<nearEnvelope.size()<<" far_kept="<<wallQuads<<std::endl;
#endif
    // Touched quads: split by the planes of the faces touching them (as core::cavityPolygons, which also splits by
    // planes of faces that only pass near), each piece kept when its centre is inside.
    std::vector<core::Poly> clipped;std::size_t insideTests=0;
    for(std::size_t k=0;k<nearEnvelope.size();++k){const auto& q=nearEnvelope[k];
        core::Poly base;for(auto p:q.p)base.push_back({p,{},q.material});std::vector<core::Poly> pieces{base},next;
        for(auto i=touchFirst[k];i<touchFirst[k+1];++i){const auto& ids=es.f[touchFaces[i]];const Vec A=es.v[ids[0]].p;Vec n=cross(es.v[ids[1]].p-A,es.v[ids[2]].p-A);
            const double norm=length(n);if(norm<1e-14)continue;n=n*(1/norm);const double bound=dot(A,n);next.clear();
            for(auto& poly:pieces){double low=1e30,high=-1e30;for(const auto& v:poly){const double d=dot(v.p,n)-bound;low=std::min(low,d);high=std::max(high,d);}
                if(low>=-1e-12||high<=1e-12){next.push_back(std::move(poly));continue;}
                auto positive=core::planeClip(poly,n,bound,1),negative=core::planeClip(poly,n,bound,-1);
                if(!positive.empty())next.push_back(std::move(positive));if(!negative.empty())next.push_back(std::move(negative));}
            std::swap(pieces,next);}
        for(auto& poly:pieces){Vec center;for(const auto& v:poly)center=center+v.p;center=center*(1./double(poly.size()));++insideTests;
            if(!asset.envelope_tree.inside(center))continue;
            std::reverse(poly.begin(),poly.end());for(auto& v:poly)v.p=core::snap(v.p);core::cancelCavityBacktracks(poly);if(poly.size()>=3)clipped.push_back(std::move(poly));}}
    dstages.mark("clip");
#ifdef GFY_VOXEL_STAGE_PROFILE
    std::cout<<"direct_clip pieces="<<clipped.size()<<" inside_tests="<<insideTests<<std::endl;
#endif
    for(const auto& poly:clipped){if(poly.size()<3||core::area(poly)<1e-12)continue;++wallQuads;emit(poly,poly[0].material);}
    dstages.mark("near_walls");
    tissueCells=0;
    return out;
}
// Direct display with compact records: the packed row of every vertex is computed when the vertex is created (the
// same pure packing::vertex() call the rows stage made later), and only the row, its hash, the discarded weight, the
// double position (weld / sliver tests) and the source face are kept: 128 bytes instead of a 656-byte JoinedVertex
// and a 48-byte pointer map entry. Triangles hold slot indices. Everything else is buildDirect verbatim.
struct DirectSlot {gfy_packed::Vertex row{};std::uint64_t hash{};double discarded{};Vec p;std::uint32_t sourceFace{~std::uint32_t(0)};std::uint32_t error{~std::uint32_t(0)};};
struct DirectTri {std::uint32_t v[3];std::uint32_t material{};};
struct DirectMesh {std::vector<DirectSlot> slots;std::vector<DirectTri> triangles;std::vector<std::string> errors;core::JoinResult stats;};
inline std::uint32_t directSlot(DirectMesh& out,const core::JoinedVertex& j,std::uint32_t sourceFace){
    DirectSlot s;s.p=j.p;s.sourceFace=sourceFace;
    try{s.row=core::packing::vertex(j,&s.discarded);s.hash=core::packing::vertexHash(s.row);}
    catch(const core::packing::PackingInfluenceError& e){s.error=std::uint32_t(out.errors.size());out.errors.emplace_back(e.what());s.hash=0;s.discarded=-1;}
    out.slots.push_back(s);return std::uint32_t(out.slots.size()-1);}
inline DirectMesh buildDirectCompact(const Asset& asset,const Grid& grid,const core::Source& patch,const core::ClipResult& outside,
                                     const std::vector<FaceRec>& recs,const std::unordered_map<core::Quant,std::array<double,64>,core::QuantHash>& fixedSkin,
                                     const std::vector<std::uint32_t>& originals,std::size_t& wallQuads){
    DirectMesh out;core::StageProfile dstages("direct");
    for(const auto& fragment:outside.polygons){const auto& poly=fragment.polygon;if(poly.size()<3)continue;
        std::vector<std::uint32_t> vs;vs.reserve(poly.size());
        need(fragment.sourceFace<originals.size(),"fullbody_patch_provenance");
        for(const auto& v:poly){auto j=core::interpolateVertex(patch,fragment.sourceFace,v);j.p=v.p;
            auto fixed=fixedSkin.find(core::quantExact(floatRounded(v.p)));if(fixed!=fixedSkin.end()&&!j.hardware_corner)j.w=fixed->second;
            vs.push_back(directSlot(out,j,originals[fragment.sourceFace]));}
        // Orient by the source face (fragments come in either order); drop slivers whose float corners flip.
        const auto& ids=patch.f[fragment.sourceFace];const Vec fn=cross(patch.v[ids[1]].p-patch.v[ids[0]].p,patch.v[ids[2]].p-patch.v[ids[0]].p);
        Vec pn;for(std::size_t k=0;k<vs.size();++k){const Vec a=out.slots[vs[k]].p,b=out.slots[vs[(k+1)%vs.size()]].p;pn=pn+Vec{(a.y-b.y)*(a.z+b.z),(a.z-b.z)*(a.x+b.x),(a.x-b.x)*(a.y+b.y)};}
        if(dot(pn,fn)<0)std::reverse(vs.begin(),vs.end());
        for(std::size_t k=1;k+1<vs.size();++k){const Vec a=floatRounded(out.slots[vs[0]].p),b=floatRounded(out.slots[vs[k]].p),c=floatRounded(out.slots[vs[k+1]].p);const Vec m=cross(b-a,c-a);
            if(length(m)<1e-14||dot(m,fn)<=0)continue;
            DirectTri t;t.material=0;t.v[0]=vs[0];t.v[1]=vs[k];t.v[2]=vs[k+1];out.triangles.push_back(t);}}
    dstages.mark("skin");WallWeights weights(asset);
    struct VK{core::Quant q;unsigned m;bool operator==(const VK& o)const{return q==o.q&&m==o.m;}};
    struct VKH{std::size_t operator()(const VK& k)const{return std::size_t(core::quantMix(std::uint64_t(core::QuantHash{}(k.q))^(std::uint64_t(k.m)<<56)));}};
    core::FlatMap<VK,std::uint32_t,VKH> wallVertex;wallVertex.reserve(recs.size());
    out.triangles.reserve(recs.size()*2);
    std::vector<std::uint32_t> c;c.reserve(8);core::Poly quadPoly;quadPoly.reserve(4);
    // A wall polygon (cavity-facing winding) fanned into triangles, corners welded by position and material.
    auto emit=[&](const core::Poly& poly,unsigned material){if(poly.size()<3)return;
        Vec pn;for(std::size_t k=0;k<poly.size();++k){const Vec a=poly[k].p,b=poly[(k+1)%poly.size()].p;pn=pn+Vec{(a.y-b.y)*(a.z+b.z),(a.z-b.z)*(a.x+b.x),(a.x-b.x)*(a.y+b.y)};}
        const double len=length(pn);if(len<1e-14)return;
        const Vec nn=pn*(1/len);const Vec T=std::abs(nn.x)>.9?Vec{0,0,1}:Vec{1,0,0};const Vec B=cross(nn,T);
        c.resize(poly.size());
        for(std::size_t k=0;k<poly.size();++k){const VK key{core::quantExact(poly[k].p),material};auto it=wallVertex.find(key);
            if(it==wallVertex.end()){core::JoinedVertex j;j.p=poly[k].p;j.normal=nn;j.tangent=T;j.binormal=B;j.sourceFace=~std::uint32_t(0);
                j.uv={dot(j.p,T)/.08,dot(j.p,B)/.08}; /* wall texture tile 8 cm */if(!weights.sample(j.p,j.w))throw core::packing::PackingInfluenceError("wall_skin_unavailable");
                it=wallVertex.try_emplace(key,directSlot(out,j,~std::uint32_t(0))).first;}
            c[k]=it->second;}
        for(std::size_t k=1;k+1<c.size();++k){if(length(cross(out.slots[c[k]].p-out.slots[c[0]].p,out.slots[c[k+1]].p-out.slots[c[0]].p))<1e-14)continue;
            DirectTri t;t.material=material;t.v[0]=c[0];t.v[1]=c[k];t.v[2]=c[k+1];out.triangles.push_back(t);++out.stats.cavityTriangles;}};
    const auto& es=asset.envelope_tree.source;
    std::unordered_map<std::uint64_t,std::shared_ptr<const std::vector<std::uint32_t>>> tileCache;
    auto floorDiv=[](int x,int d){return x>=0?x/d:-((-x+d-1)/d);};
    auto tileFaces=[&](Cell t)->const std::vector<std::uint32_t>&{
        const std::uint64_t key=WallWeights::key(t.x,t.y,t.z);auto it=tileCache.find(key);if(it!=tileCache.end())return *it->second;
        std::shared_ptr<const std::vector<std::uint32_t>> list;
        {std::lock_guard<std::mutex> lock(asset.envelope_tiles.mutex);auto g=asset.envelope_tiles.map.find(key);if(g!=asset.envelope_tiles.map.end())list=g->second;}
        if(!list){core::Bounds box;box.lo=grid.origin+Vec{double(t.x*8),double(t.y*8),double(t.z*8)}*grid.pitch;box.hi=grid.origin+Vec{double(t.x*8+8),double(t.y*8+8),double(t.z*8+8)}*grid.pitch;
            for(int a=0;a<3;++a){box.lo[a]-=1e-9;box.hi[a]+=1e-9;}
            std::vector<std::uint32_t> candidates,faces;asset.envelope_tree.query(0,box,candidates);std::sort(candidates.begin(),candidates.end());
            for(auto f:candidates){const auto& ids=es.f[f];if(triangleBoxOverlap(es.v[ids[0]].p,es.v[ids[1]].p,es.v[ids[2]].p,box.lo,box.hi))faces.push_back(f);}
            list=std::make_shared<const std::vector<std::uint32_t>>(std::move(faces));
            std::lock_guard<std::mutex> lock(asset.envelope_tiles.mutex);if(asset.envelope_tiles.map.size()>=100000)asset.envelope_tiles.map.clear();list=asset.envelope_tiles.map.try_emplace(key,list).first->second;}
        return *tileCache.emplace(key,list).first->second;};
    // Envelope faces touching a face (closed), ascending.
    std::vector<std::uint32_t> touchFaces,touchFirst{0};
    auto touched=[&](const FaceRec& f,const Vec* p){const auto before=touchFaces.size();
        const Cell t{floorDiv(f.x,8),floorDiv(f.y,8),floorDiv(f.z,8)};
        core::Bounds qb;for(int k=0;k<4;++k)qb.add(p[k]);
        for(auto face:tileFaces(t)){const auto& ids=es.f[face];const Vec A=es.v[ids[0]].p,B=es.v[ids[1]].p,C=es.v[ids[2]].p;const Vec n=cross(B-A,C-A);
            double lo=1e300,hi=-1e300;for(int k=0;k<4;++k){const double d=dot(p[k]-A,n);lo=std::min(lo,d);hi=std::max(hi,d);}
            if(lo>0||hi<0)continue;if(triangleBoxOverlap(A,B,C,qb.lo,qb.hi))touchFaces.push_back(face);}
        if(touchFaces.size()==before)return false;touchFirst.push_back(std::uint32_t(touchFaces.size()));return true;};
    auto tileSide=[&](Cell t)->signed char{const std::uint64_t key=WallWeights::key(t.x,t.y,t.z);
        {std::lock_guard<std::mutex> lock(asset.envelope_tiles.mutex);auto g=asset.envelope_tiles.sides.find(key);if(g!=asset.envelope_tiles.sides.end())return g->second;}
        const Vec centre=grid.origin+Vec{double(t.x*8+4),double(t.y*8+4),double(t.z*8+4)}*grid.pitch;const signed char s=asset.envelope_tree.inside(centre)?1:0;
        std::lock_guard<std::mutex> lock(asset.envelope_tiles.mutex);if(asset.envelope_tiles.sides.size()>=200000)asset.envelope_tiles.sides.clear();asset.envelope_tiles.sides[key]=s;return s;};
    std::vector<FaceRec> nearEnvelope;std::vector<std::uint32_t> mixedFree;std::vector<std::uint8_t> kind(recs.size(),0); // 0 out, 1 in, 2 near, 3 free face of a mixed tile
    Vec cp[4];
    {Cell cur{INT_MIN,INT_MIN,INT_MIN};signed char curSide=0;
     for(std::uint32_t i=0;i<recs.size();++i){const auto& f=recs[i];const Cell t{floorDiv(f.x,8),floorDiv(f.y,8),floorDiv(f.z,8)};
        if(!(t==cur)){cur=t;curSide=tileFaces(t).empty()?tileSide(t):-1;}
        if(curSide>=0){kind[i]=std::uint8_t(curSide);continue;}
        faceCorners(grid,f,cp);
        if(touched(f,cp)){kind[i]=2;nearEnvelope.push_back(f);}else{kind[i]=3;mixedFree.push_back(i);}}}
    dstages.mark("touch");
    {std::vector<std::uint32_t> parent(mixedFree.size());for(std::uint32_t k=0;k<parent.size();++k)parent[k]=k;
     auto find=[&](std::uint32_t k){while(parent[k]!=k){parent[k]=parent[parent[k]];k=parent[k];}return k;};
     struct EdgeKeyHash {std::size_t operator()(std::uint64_t k)const{return std::size_t(core::quantMix(k));}};
     const double invPitch=1/grid.pitch;
     auto ekey=[&](const Vec& p,const Vec& r){std::uint64_t key=0;int ax=0;for(int a=0;a<3;++a){const std::uint64_t x=std::uint64_t(std::llround((p[a]-grid.origin[a])*invPitch)),y=std::uint64_t(std::llround((r[a]-grid.origin[a])*invPitch));if(x!=y)ax=a;key=(key<<17)|std::min(x,y);}return (key<<2)|std::uint64_t(ax);};
     core::FlatMap<std::uint64_t,std::uint32_t,EdgeKeyHash> edgeOwner;edgeOwner.reserve(mixedFree.size()*4);
     for(std::uint32_t k=0;k<mixedFree.size();++k){faceCorners(grid,recs[mixedFree[k]],cp);
        for(int e=0;e<4;++e){auto it=edgeOwner.try_emplace(ekey(cp[e],cp[(e+1)%4]),k);
            if(!it.second){const auto a=find(k),b=find(it.first->second);if(a!=b)parent[std::max(a,b)]=std::min(a,b);}}}
     std::vector<signed char> side(mixedFree.size(),-1);
     for(std::uint32_t k=0;k<mixedFree.size();++k){const auto root=find(k);
        if(side[root]<0){faceCorners(grid,recs[mixedFree[k]],cp);side[root]=asset.envelope_tree.inside((cp[0]+cp[1]+cp[2]+cp[3])*.25)?1:0;}
        if(!side[root])kind[mixedFree[k]]=0;}
     for(std::uint32_t i=0;i<recs.size();++i){if(kind[i]!=1&&kind[i]!=3)continue;const auto& f=recs[i];faceCorners(grid,f,cp);
        ++wallQuads;quadPoly.clear();for(int cc=3;cc>=0;--cc)quadPoly.push_back({cp[cc],{},unsigned(f.material)});emit(quadPoly,unsigned(f.material));}}
    dstages.mark("far_walls");
#ifdef GFY_VOXEL_STAGE_PROFILE
    std::cout<<"direct_walls quads="<<recs.size()<<" near="<<nearEnvelope.size()<<" far_kept="<<wallQuads<<std::endl;
#endif
    std::vector<core::Poly> clipped;std::size_t insideTests=0;
    for(std::size_t k=0;k<nearEnvelope.size();++k){const auto& f=nearEnvelope[k];faceCorners(grid,f,cp);
        core::Poly base;for(int cc=0;cc<4;++cc)base.push_back({cp[cc],{},unsigned(f.material)});std::vector<core::Poly> pieces{base},next;
        for(auto i=touchFirst[k];i<touchFirst[k+1];++i){const auto& ids=es.f[touchFaces[i]];const Vec A=es.v[ids[0]].p;Vec n=cross(es.v[ids[1]].p-A,es.v[ids[2]].p-A);
            const double norm=length(n);if(norm<1e-14)continue;n=n*(1/norm);const double bound=dot(A,n);next.clear();
            for(auto& poly:pieces){double low=1e30,high=-1e30;for(const auto& v:poly){const double d=dot(v.p,n)-bound;low=std::min(low,d);high=std::max(high,d);}
                if(low>=-1e-12||high<=1e-12){next.push_back(std::move(poly));continue;}
                auto positive=core::planeClip(poly,n,bound,1),negative=core::planeClip(poly,n,bound,-1);
                if(!positive.empty())next.push_back(std::move(positive));if(!negative.empty())next.push_back(std::move(negative));}
            std::swap(pieces,next);}
        for(auto& poly:pieces){Vec center;for(const auto& v:poly)center=center+v.p;center=center*(1./double(poly.size()));++insideTests;
            if(!asset.envelope_tree.inside(center))continue;
            std::reverse(poly.begin(),poly.end());for(auto& v:poly)v.p=core::snap(v.p);core::cancelCavityBacktracks(poly);if(poly.size()>=3)clipped.push_back(std::move(poly));}}
    dstages.mark("clip");
#ifdef GFY_VOXEL_STAGE_PROFILE
    std::cout<<"direct_clip pieces="<<clipped.size()<<" inside_tests="<<insideTests<<std::endl;
#endif
    for(const auto& poly:clipped){if(poly.size()<3||core::area(poly)<1e-12)continue;++wallQuads;emit(poly,poly[0].material);}
    dstages.mark("near_walls");
    return out;
}
inline std::shared_ptr<const RegionResult> computeRegion(const Asset& asset,const Grid& grid,const Region& region,
                                                         const BoundaryCache& boundaryCache,WorkerPhaseTimes& times,const TileRegionState* tileState=nullptr){
    auto result=std::make_shared<RegionResult>();auto& r=*result;
    r.keys=region.keys;r.masks=region.masks;r.hash=region.hash;r.faces=region.faces;r.selected=region.selected;
    r.tileLocal=region.tileLocal;r.expanded=region.expanded;r.protect=region.protect;r.relevant=region.relevant;
    r.chunked=region.chunked;for(int a=0;a<3;++a){r.seamLoM[a]=grid.origin[a]+region.seamLo[a]*grid.pitch;r.seamHiM[a]=grid.origin[a]+region.seamHi[a]*grid.pitch;}
    // Geometry-only proof: retain the complete region and damage mask, including original air.
    // Face selection includes a one-cell halo around every damaged tile. When faces is empty,
    // its lattice seam points cannot lock any source-skin vertex in a neighbouring chunk.
    if(region.tileLocal&&tileState&&directDisplay()&&compactWalls()&&!grid.ownership&&region.faces.empty()){
        bool empty=region.keys.empty();
        if(!empty){
            core::Bounds box;const auto first=region.keys.front();
            box.lo=grid.origin+Vec{double(first.x*8),double(first.y*8),double(first.z*8)}*grid.pitch;
            box.hi=box.lo+Vec{8.*grid.pitch,8.*grid.pitch,8.*grid.pitch};
            for(const auto& key:region.keys)for(int a=0;a<3;++a){
                box.lo[a]=std::min(box.lo[a],grid.origin[a]+double((&key.x)[a]*8)*grid.pitch);
                box.hi[a]=std::max(box.hi[a],grid.origin[a]+double((&key.x)[a]*8+8)*grid.pitch);
            }
            for(int a=0;a<3;++a){box.lo[a]-=1e-5;box.hi[a]+=1e-5;}
            std::vector<std::uint32_t> candidates;asset.envelope_tree.query(0,box,candidates);
            empty=candidates.empty()&&!asset.envelope_tree.inside((box.lo+box.hi)*.5);
        }
        if(empty){r.directDisplay=true;r.featureRadius=.05;r.defectChecked=true;
#ifdef GFY_VOXEL_STAGE_PROFILE
            std::cout<<"region_empty_skipped tiles="<<region.keys.size()<<"\n";
#endif
            return result;
        }
    }
    std::vector<core::Quad> boundary;std::vector<FaceRec> recs; // recs: the direct path's compact unit faces (GFY_COMPACT_WALLS)
    {PhaseTime boundaryTime(times.boundary_ms);std::vector<core::Quad> quads;
        if(region.tileLocal&&tileState&&directDisplay()){if(compactWalls())for(const auto& key:region.keys)tileBoundaryRecs(grid,key,recs);else for(const auto& key:region.keys)tileBoundary(grid,key,quads);} // no per-hit quad cache on this path
        else for(const auto& key:region.keys){auto found=boundaryCache.find(key);if(found!=boundaryCache.end())quads.insert(quads.end(),found->second->begin(),found->second->end());}
        core::StageProfile mergeStage("region");mergeStage.mark("gather_quads");
        if(region.tileLocal&&tileState&&directDisplay())boundary=std::move(quads); // unit voxel faces (direct display)
        else boundary=mergeBoundaryRects(grid,quads,r.rects,r.planeRows,region.chunked?region.seamLo:nullptr,region.chunked?region.seamHi:nullptr,region.tileLocal);mergeStage.mark("merge_rects");}
    PhaseTime preparationTime(times.join_ms);
    core::StageProfile stages("fullbody");
    const auto& originals=region.faces;
    // The patch shares the asset's vertex array: one copy per thread and asset, not per region.
    thread_local core::Source patch;thread_local Hash patchSource{};thread_local std::size_t patchVertices=~std::size_t(0);
    if(patchSource!=asset.source_hash||patchVertices!=asset.source.v.size()){patch.v=asset.source.v;patch.bones=asset.source.bones;patchSource=asset.source_hash;patchVertices=asset.source.v.size();}
    patch.f.clear();patch.f.reserve(originals.size());
    for(auto face:originals)patch.f.push_back(asset.source.f[face]);
    stages.mark("select_patch");preparationTime.finish();core::ClipResult outside;
    std::vector<Vec> fixedPositions;std::unordered_map<core::Quant,std::array<double,64>,core::QuantHash> fixedSkin;core::JoinExtras extras;
    const bool direct=region.tileLocal&&tileState&&directDisplay();
    if(region.tileLocal&&tileState){PhaseTime exteriorTime(times.exterior_ms);outside=tileLocalFragments(grid,region,*tileState,patch,fixedPositions,fixedSkin,*asset.fast);
        extras.weldBase=asset.weld_base.get();extras.fixedPositions=&fixedPositions;extras.fixedSkin=&fixedSkin;
        if(!direct)extras.extraOpening=tileLocalOpenings(grid,region,*tileState,LatticeSpec{grid.origin,grid.pitch},fixedPositions); // join-only
        if(region.chunked){
            // Seams with the neighbouring chunks of the same cluster: cavity edges lying on a chunk plane are
            // open by construction (the wall continues in the neighbour), their lattice points never move, and
            // cavity vertices on a seam take position-only skin weights (both chunks agree bit for bit).
            const Vec o=grid.origin;const double pitch=grid.pitch;int lo[3],hi[3];for(int a=0;a<3;++a){lo[a]=region.seamLo[a];hi[a]=region.seamHi[a];}
            auto planeOf=[o,pitch,lo,hi](Vec p,int& axis,int& which){for(int a=0;a<3;++a){const double x=(p[a]-o[a])/pitch;
                if(std::abs(x-lo[a])<=1e-3){axis=a;which=0;return true;}if(std::abs(x-hi[a])<=1e-3){axis=a;which=1;return true;}}return false;};
            // Only quads of cells in the chunk's border layers can have a corner on a seam plane.
            for(const auto& q:boundary){const Cell c=q.removedCell;bool border=false;
                for(int a=0;a<3;++a)border=border||(&c.x)[a]==lo[a]||(&c.x)[a]+1==hi[a];
                if(!border)continue;for(const auto& p:q.p){int axis,which;if(planeOf(p,axis,which))fixedPositions.push_back(p);}}
            for(const auto& f:recs){const Cell c{f.x,f.y,f.z};bool border=false;
                for(int a=0;a<3;++a)border=border||(&c.x)[a]==lo[a]||(&c.x)[a]+1==hi[a];
                if(!border)continue;Vec ps[4];faceCorners(grid,f,ps);for(const auto& p:ps){int axis,which;if(planeOf(p,axis,which))fixedPositions.push_back(p);}}
            if(!direct){
            auto base=extras.extraOpening;
            extras.extraOpening=[base,o,pitch,lo,hi](Vec a,Vec b){if(base&&base(a,b))return true;
                for(int k=0;k<3;++k){const double xa=(a[k]-o[k])/pitch,xb=(b[k]-o[k])/pitch;
                    if((std::abs(xa-lo[k])<=1e-3&&std::abs(xb-lo[k])<=1e-3)||(std::abs(xa-hi[k])<=1e-3&&std::abs(xb-hi[k])<=1e-3))return true;}
#ifdef GFY_CUT_DIAGNOSTICS
                std::cout<<"seam_opening_miss "<<(a[0]-o[0])/pitch<<' '<<(a[1]-o[1])/pitch<<' '<<(a[2]-o[2])/pitch<<" lo "<<lo[0]<<','<<lo[1]<<','<<lo[2]<<" hi "<<hi[0]<<','<<hi[1]<<','<<hi[2]<<'\n';
#endif
                return false;};
            const core::BodyBVH* tree=&asset.source_tree;
            // Cached top-4 weights: the same compact value for every chunk and every job (so seams agree).
            auto* cache=&asset.seam_weights;
            auto cachedWeights=[tree,cache](Vec p,std::array<double,64>& w){const auto key=core::quantExact(p);
                {std::lock_guard<std::mutex> lock(cache->mutex);auto it=cache->map.find(key);if(it!=cache->map.end()){w.fill(0.);for(const auto& e:it->second)if(e.second!=0.f)w[e.first]+=e.second;return true;}}
                std::array<double,64> full{};if(!nearestSkinWeights(*tree,p,full))return false;
                std::array<std::uint16_t,4> signedBones{};std::array<float,4> signedWeights{};const bool signedSkin=gfy_signed_hw::negative(full);if(signedSkin)gfy_signed_hw::pack(full,signedBones,signedWeights);
                std::array<unsigned,64> order;for(unsigned k=0;k<64;++k)order[k]=k;std::stable_sort(order.begin(),order.end(),[&](unsigned a,unsigned b){return full[a]>full[b];});
                std::array<std::pair<std::uint16_t,float>,4> top{};double sum=0;for(int k=0;k<4;++k){top[std::size_t(k)]={std::uint16_t(order[std::size_t(k)]),float(full[order[std::size_t(k)]])};sum+=top[std::size_t(k)].second;}
                if(!(sum>0))return false;for(auto& e:top)e.second=float(e.second/sum);
                if(signedSkin)for(unsigned k=0;k<4;++k)top[k]={signedBones[k],signedWeights[k]};
                {std::lock_guard<std::mutex> lock(cache->mutex);if(cache->map.size()>=400000)cache->map.clear();top=cache->map.try_emplace(key,top).first->second;}
                w.fill(0.);for(const auto& e:top)if(e.second!=0.f)w[e.first]+=e.second;return true;};
            extras.seamSkin=[planeOf,cachedWeights](Vec p,std::array<double,64>& w){int axis,which;if(!planeOf(p,axis,which))return false;return cachedWeights(p,w);};
            extras.anySkin=[cachedWeights](Vec p,std::array<double,64>& w){return cachedWeights(p,w);};
            }
        }
    }
    else{core::BodyBVH tree(patch);(void)tree;PhaseTime exteriorTime(times.exterior_ms);outside=core::exterior(patch,grid);}
    stages.mark("exterior");
    PhaseTime joinTime(times.join_ms);double tissueCells=0;std::size_t wallQuads=0;
    if(direct&&compactWalls()){
        DirectMesh mesh=buildDirectCompact(asset,grid,patch,outside,recs,fixedSkin,originals,wallQuads);stages.mark("join");
        {double removedCells=0;for(const auto& m:region.masks)for(auto word:m.removed)removedCells+=double(std::bitset<64>(word).count());(void)tissueCells;
         r.directDisplay=true;r.featureRadius=wallQuads?std::min(.1,2.*removedCells/double(wallQuads)*grid.pitch):.05;
         r.lockedSkinKeys.reserve(fixedPositions.size());for(const auto& p:fixedPositions)r.lockedSkinKeys.push_back(core::quantExact(floatRounded(p)));
         std::sort(r.lockedSkinKeys.begin(),r.lockedSkinKeys.end());r.lockedSkinKeys.erase(std::unique(r.lockedSkinKeys.begin(),r.lockedSkinKeys.end()),r.lockedSkinKeys.end());}
        const auto& joined=mesh.stats;
        r.triangles=mesh.triangles.size();r.originalOpen=joined.originalOpen;r.newOpen=joined.newOpen;r.nonmanifold=joined.nonmanifold;
        r.numericalOpenEdges=joined.numericalOpenEdges;r.numericalSeams=joined.numericalSeams;r.numericalSpurs=joined.numericalSpurs;
        r.interiorSheetJunctions=joined.interiorSheetJunctions;r.cavityTriangles=joined.cavityTriangles;r.nonmanifoldCavityDoubled=joined.nonmanifoldCavityDoubled;
        r.maxConflict=joined.maxJoinWeightConflict;r.numericalOpenLength=joined.numericalOpenLength;
#ifdef GFY_VOXEL_STAGE_PROFILE
        std::cout<<"region_info tiles="<<region.keys.size()<<" expanded="<<region.expanded.size()<<" faces="<<region.faces.size()<<" fragments="<<outside.polygons.size()<<" quads="<<recs.size()<<" tris="<<r.triangles<<" cavity="<<r.cavityTriangles<<"\n";
#endif
        stages.mark("region_stats");stages.mark("region_rect_index");
        // Packing rows: the same checks in the same order as the join path; rows were computed per slot at emission
        // (a failing slot raises the identical error at the first corner the serial loop reaches). The direct path has
        // no merged rectangles, so every cavity triangle's rect is ~0 as before.
        try{
            const auto& faceChildren=asset.face_children;const auto fallback=faceChildren.front();
            std::map<GroupKey,std::vector<PackedTri>> groups;
            std::size_t capacity=16;while(capacity<2*mesh.slots.size()+16)capacity*=2;
            std::vector<std::uint32_t> slots(capacity,~0u);const std::size_t mask=capacity-1;
            std::vector<std::uint32_t> rowOf(mesh.slots.size(),~0u);
            for(const auto& triangle:mesh.triangles){
                std::uint16_t source=fallback;std::uint32_t original=~std::uint32_t(0);
                if(!triangle.material){const auto face=mesh.slots[triangle.v[0]].sourceFace;
                    core::packing::packingRequire(face<faceChildren.size()&&mesh.slots[triangle.v[1]].sourceFace==face&&mesh.slots[triangle.v[2]].sourceFace==face,"Exterior source face");
                    source=faceChildren[face];original=face;}
                core::packing::packingRequire(triangle.material<64,"Interior material slot limit");
                PackedTri packed;packed.face=original;
                for(int k=0;k<3;++k){const std::uint32_t n=triangle.v[k];const auto& slot=mesh.slots[n];
                    if(slot.error!=~std::uint32_t(0))throw core::packing::PackingInfluenceError(mesh.errors[slot.error]);
                    if(rowOf[n]==~0u){const auto& row=slot.row;const auto hash=slot.hash;const double discarded=slot.discarded;
                        std::uint32_t id=~0u;std::size_t at=std::size_t(hash)&mask;
                        for(;slots[at]!=~0u;at=(at+1)&mask){const auto candidate=slots[at];
                            if(r.hashes[candidate]==hash&&!std::memcmp(&r.rows[candidate],&row,sizeof(row))&&r.discarded[candidate]==discarded){id=candidate;break;}}
                        if(id==~0u){id=std::uint32_t(r.rows.size());r.rows.push_back(row);r.hashes.push_back(hash);r.discarded.push_back(discarded);slots[at]=id;}
                        rowOf[n]=id;}
                    packed.row[k]=rowOf[n];}
                groups[{triangle.material,source}].push_back(packed);
            }
            for(auto& g:groups)r.groups.emplace_back(g.first,std::move(g.second));
        }catch(const core::packing::PackingInfluenceError& e){r.packingError=e.what();r.groups.clear();}
        stages.mark("region_rows");
        r.exactDefect=regionDefect(asset,r);r.defectChecked=true;
        lowpoly::reduceRegion(asset,r);stages.mark("region_lowpoly");
#ifdef GFY_VOXEL_STAGE_PROFILE
        std::cout<<"lowpoly_region chunked="<<r.chunked<<" wedges="<<r.lowpolyWedges<<" locked="<<r.lowpolyLocked<<" seam_dropped="<<r.lowpolySeamDropped<<" in="<<r.lowpolyInput<<" out="<<r.triangles<<" cavity="<<r.cavityTriangles<<" mixed="<<r.lowpolyMixed<<" target="<<r.lowpolyTarget<<"\n";
#endif
        return result;
    }
    auto indexed=direct?buildDirect(asset,grid,patch,outside,boundary,fixedSkin,tissueCells,wallQuads)
                       :core::joinIndexed(patch,asset.envelope_tree,grid,outside,boundary,region.tileLocal?&extras:nullptr);stages.mark("join");
    if(direct){double removedCells=0;for(const auto& m:region.masks)for(auto word:m.removed)removedCells+=double(std::bitset<64>(word).count());(void)tissueCells;
        r.directDisplay=true;r.featureRadius=wallQuads?std::min(.1,2.*removedCells/double(wallQuads)*grid.pitch):.05;
        r.lockedSkinKeys.reserve(fixedPositions.size());for(const auto& p:fixedPositions)r.lockedSkinKeys.push_back(core::quantExact(floatRounded(p)));
        std::sort(r.lockedSkinKeys.begin(),r.lockedSkinKeys.end());r.lockedSkinKeys.erase(std::unique(r.lockedSkinKeys.begin(),r.lockedSkinKeys.end()),r.lockedSkinKeys.end());}
    const auto& joined=indexed.stats;auto& triangles=indexed.triangles;
#ifdef GFY_CS_VERIFY
    {// Verification build: the legacy join must produce the identical triangles.
        auto legacy=core::join(patch,tree,asset.envelope_tree,grid,outside,boundary);
        bool same=legacy.triangles.size()==triangles.size()&&legacy.newOpen==joined.newOpen&&legacy.nonmanifold==joined.nonmanifold&&legacy.originalOpen==joined.originalOpen
            &&legacy.cavityTriangles==joined.cavityTriangles&&legacy.numericalSpurs==joined.numericalSpurs&&legacy.numericalSeams==joined.numericalSeams
            &&legacy.interiorSheetJunctions==joined.interiorSheetJunctions&&legacy.numericalOpenEdges==joined.numericalOpenEdges
            &&!std::memcmp(&legacy.maxJoinWeightConflict,&joined.maxJoinWeightConflict,8)&&!std::memcmp(&legacy.numericalOpenLength,&joined.numericalOpenLength,8)&&!std::memcmp(&legacy.cutPerimeter,&joined.cutPerimeter,8);
        for(std::size_t t=0;same&&t<triangles.size();++t){same=legacy.triangles[t].material==triangles[t].material;
            for(int k=0;same&&k<3;++k){const auto&x=legacy.triangles[t].v[k];const auto&y=*triangles[t].v[k];
                same=!std::memcmp(&x.p,&y.p,sizeof(Vec)*4)&&x.sourceFace==y.sourceFace&&!std::memcmp(x.sourceBary.data(),y.sourceBary.data(),24)
                    &&!std::memcmp(x.uv.data(),y.uv.data(),16)&&!std::memcmp(x.w.data(),y.w.data(),512);}}
        if(!same)throw std::runtime_error("verify_indexed_join_mismatch");
    }
#endif
    {// Exterior provenance to original faces, once per shared vertex.
        core::FlatMap<const core::JoinedVertex*,char,PointerHash> remapped;remapped.reserve(triangles.size());
        for(auto& t:triangles)if(!t.material)for(auto* v:t.v){if(!remapped.try_emplace(v,0).second)continue;
            need(v->sourceFace<originals.size(),"fullbody_patch_provenance");v->sourceFace=originals[v->sourceFace];}}
    r.triangles=triangles.size();r.originalOpen=joined.originalOpen;r.newOpen=joined.newOpen;r.nonmanifold=joined.nonmanifold;
    r.numericalOpenEdges=joined.numericalOpenEdges;r.numericalSeams=joined.numericalSeams;r.numericalSpurs=joined.numericalSpurs;
    r.interiorSheetJunctions=joined.interiorSheetJunctions;r.cavityTriangles=joined.cavityTriangles;r.nonmanifoldCavityDoubled=joined.nonmanifoldCavityDoubled;
    r.maxConflict=joined.maxJoinWeightConflict;r.numericalOpenLength=joined.numericalOpenLength;
#ifdef GFY_VOXEL_STAGE_PROFILE
    std::cout<<"region_info tiles="<<region.keys.size()<<" expanded="<<region.expanded.size()<<" faces="<<region.faces.size()<<" fragments="<<outside.polygons.size()<<" quads="<<boundary.size()<<" tris="<<r.triangles<<" cavity="<<r.cavityTriangles<<"\n";
#endif
    stages.mark("region_stats");
    // Cavity triangles follow their rectangles in emission order (quads are
    // clipped, conformed and fanned in order), so one forward scan finds each
    // triangle's rectangle; a miss leaves ~0 (only the ORDER may then differ).
    std::size_t rectCursor=0;
    auto locateRect=[&](const core::IndexedTriangle& t)->std::uint32_t{
        const Vec p0=t.v[0]->p,p1=t.v[1]->p,p2=t.v[2]->p;
        const Vec n=cross(p1-p0,p2-p0);int axis=0;for(int a=1;a<3;++a)if(std::abs(n[a])>std::abs(n[axis]))axis=a;
        const int u=(axis+1)%3,v=(axis+2)%3;const Vec c=(p0+p1+p2)*(1./3);
        const int plane=int(std::llround((c[axis]-grid.origin[axis])/grid.pitch));
        const int a=int(std::floor((c[u]-grid.origin[u])/grid.pitch)),b=int(std::floor((c[v]-grid.origin[v])/grid.pitch));
        for(std::size_t i=rectCursor;i<r.rects.size();++i){const auto& m=r.rects[i];
            if(m.plane[0]==axis&&m.plane[1]==plane&&m.plane[3]==int(t.material)&&a>=m.u0&&a<m.u1&&b>=m.v0&&b<m.v1){rectCursor=i;return std::uint32_t(i);}}
        return ~0u;};
    // Packing rows. Same checks and same vertex() conversion as packMulti; the
    // first packing failure is deferred until after the union topology gate.
    stages.mark("region_rect_index");
    try{
        const auto& faceChildren=asset.face_children;const auto fallback=faceChildren.front();
        std::map<GroupKey,std::vector<PackedTri>> groups;
        // packing::vertex() is a pure function of a vertex: computed once per
        // distinct vertex (in parallel), consumed per corner in serial order (a
        // failing vertex rethrows at the first corner the serial loop reached).
        core::FlatMap<const core::JoinedVertex*,std::uint32_t,PointerHash> vertexIndex;vertexIndex.reserve(indexed.vertices.size());
        std::vector<const core::JoinedVertex*> unique;unique.reserve(indexed.vertices.size());
        for(const auto& t:triangles)for(auto* v:t.v)if(vertexIndex.try_emplace(v,std::uint32_t(unique.size())).second)unique.push_back(v);
        std::vector<gfy_packed::Vertex> vertexRow(unique.size());std::vector<std::uint64_t> vertexHashes(unique.size());std::vector<double> vertexDiscarded(unique.size(),0.);
        std::atomic<bool> anyError{false};
        core::parallelChunks(unique.size(),4096,[&](std::size_t first,std::size_t last,std::size_t){
            for(std::size_t n=first;n<last;++n){
                try{vertexRow[n]=core::packing::vertex(*unique[n],&vertexDiscarded[n]);vertexHashes[n]=core::packing::vertexHash(vertexRow[n]);}
                catch(const core::packing::PackingInfluenceError&){anyError=true;vertexHashes[n]=0;vertexDiscarded[n]=-1;}}});
        // Open-addressed row index (exact bytes + exact discarded value).
        std::size_t capacity=16;while(capacity<2*unique.size()+16)capacity*=2;
        std::vector<std::uint32_t> slots(capacity,~0u);const std::size_t mask=capacity-1;
        std::vector<std::uint32_t> rowOf(unique.size(),~0u);
        bool unlocated=false;
        for(const auto& triangle:triangles){
            std::uint16_t source=fallback;std::uint32_t original=~std::uint32_t(0);
            if(!triangle.material){const auto face=triangle.v[0]->sourceFace;
                core::packing::packingRequire(face<faceChildren.size()&&triangle.v[1]->sourceFace==face&&triangle.v[2]->sourceFace==face,"Exterior source face");
                source=faceChildren[face];original=face;}
            core::packing::packingRequire(triangle.material<64,"Interior material slot limit");
            PackedTri packed;packed.face=original;if(triangle.material&&!unlocated){packed.rect=locateRect(triangle);unlocated=packed.rect==~0u;}
            for(int k=0;k<3;++k){const std::uint32_t n=vertexIndex.find(triangle.v[k])->second;
                if(anyError&&vertexDiscarded[n]<0){double ignored;core::packing::vertex(*triangle.v[k],&ignored);} // rethrows the identical error
                if(rowOf[n]==~0u){const auto& row=vertexRow[n];const auto hash=vertexHashes[n];const double discarded=vertexDiscarded[n];
                    std::uint32_t id=~0u;std::size_t at=std::size_t(hash)&mask;
                    for(;slots[at]!=~0u;at=(at+1)&mask){const auto candidate=slots[at];
                        if(r.hashes[candidate]==hash&&!std::memcmp(&r.rows[candidate],&row,sizeof(row))&&r.discarded[candidate]==discarded){id=candidate;break;}}
                    if(id==~0u){id=std::uint32_t(r.rows.size());r.rows.push_back(row);r.hashes.push_back(hash);r.discarded.push_back(discarded);slots[at]=id;}
                    rowOf[n]=id;}
                packed.row[k]=rowOf[n];}
            groups[{triangle.material,source}].push_back(packed);
        }
        for(auto& g:groups)r.groups.emplace_back(g.first,std::move(g.second));
    }catch(const core::packing::PackingInfluenceError& e){r.packingError=e.what();r.groups.clear();}
    stages.mark("region_rows");
    // Low-poly display: reduce this region's cut skin + walls to game resolution (the exact join above stays the
    // gate input; its statistics were recorded before the reduction).
    r.exactDefect=regionDefect(asset,r);r.defectChecked=true;
    lowpoly::reduceRegion(asset,r);stages.mark("region_lowpoly");
#ifdef GFY_VOXEL_STAGE_PROFILE
    std::cout<<"lowpoly_region chunked="<<r.chunked<<" wedges="<<r.lowpolyWedges<<" locked="<<r.lowpolyLocked<<" seam_dropped="<<r.lowpolySeamDropped<<" in="<<r.lowpolyInput<<" out="<<r.triangles<<" cavity="<<r.cavityTriangles<<" mixed="<<r.lowpolyMixed<<" target="<<r.lowpolyTarget<<" fixed_ext="<<r.lowpolyFixedExt<<" locked_skin="<<r.lowpolyFreeExt<<" flipped="<<r.lowpolyFlipped<<" flip_area_mm2="<<r.lowpolyFlipArea*1e6<<" retries="<<r.lowpolyRetries<<" deep="<<r.lowpolyDeepIn<<">"<<r.lowpolyDeepOut<<">"<<r.lowpolyDeepFinal<<" deep_ms="<<r.lowpolyDeepMs<<" error_mm="<<r.lowpolyError*1e3<<" limit_mm="<<r.lowpolyLimit*1e3<<" ms="<<r.lowpolyMs<<"\n";
#endif
    return result;
}

// Computes every null entry of `results` (index-aligned with `regions`).
// Independent regions of one batch (for example a burst of separated hits) run
// concurrently on at most four threads, including the calling worker. Each
// region records its own phase times; the reported phases are their SUM (CPU
// time), while Output::worker_ms remains wall time. The first failing region
// in region order rethrows, exactly as a sequential loop would.
inline void computeMissingRegions(const Asset& asset,const Grid& grid,const std::vector<Region>& regions,const BoundaryCache& boundaryCache,
                                  RegionList& results,WorkerPhaseTimes& times,const TileRegionState* tileState=nullptr,std::vector<std::string>* failures=nullptr){
    std::vector<std::size_t> missing;for(std::size_t n=0;n<regions.size();++n)if(!results[n])missing.push_back(n);
    std::vector<WorkerPhaseTimes> phase(missing.size());std::vector<std::exception_ptr> failure(missing.size());
    std::atomic<std::size_t> cursor{0};
    auto work=[&]{for(std::size_t slot;(slot=cursor++)<missing.size();){
        try{results[missing[slot]]=computeRegion(asset,grid,regions[missing[slot]],boundaryCache,phase[slot],tileState);}
        catch(...){failure[slot]=std::current_exception();}}};
    // Region threads scale with the CPU (player floor 4C/8T -> 4; 6C/12T -> 6; 8C/16T+ -> 8); env GFY_REGION_THREADS.
    static const std::size_t regionThreads=core::regionThreadCount();
    const std::size_t threads=std::min<std::size_t>(missing.size(),regionThreads);
    if(threads>1){const auto latch=std::make_shared<core::Latch>(threads-1);auto& pool=core::workerPool(); // shared: see core::Latch
        for(std::size_t t=1;t<threads;++t)pool.submit([&work,latch]{work();latch->done();});
        {const bool prev=core::painterInlineParallel;core::painterInlineParallel=true;work();core::painterInlineParallel=prev;}
        latch->wait();}
    else work();
    for(const auto& p:phase){times.boundary_ms+=p.boundary_ms;times.exterior_ms+=p.exterior_ms;times.join_ms+=p.join_ms;times.pack_ms+=p.pack_ms;times.removal_ms+=p.removal_ms;}
    // An allocation failure is never a region verdict: it must not route the
    // region to the fallback path or enter the failure memo. It fails the job.
    for(const auto& f:failure)if(f){try{std::rethrow_exception(f);}catch(const std::bad_alloc&){throw;}catch(...){}}
    if(failures){failures->assign(regions.size(),std::string());
        for(std::size_t slot=0;slot<missing.size();++slot)if(failure[slot]){try{std::rethrow_exception(failure[slot]);}catch(const std::exception& e){(*failures)[missing[slot]]=e.what();}catch(...){(*failures)[missing[slot]]="unknown";}}
        return;}
    for(const auto& f:failure)if(f)std::rethrow_exception(f);
}
// The former (int10) region for the removed tiles of ONE tile-local region: its
// faces are the tiles' active faces plus EVERY face sharing a vertex position
// with them (slab splits of a clipped face can end anywhere on its edges), so
// no face outside the set can receive a point. Used as a per-region fallback.
inline Region fallbackRegion(const Grid& grid,const core::BodyBVH& tree,const AssetData& data,TileFaceMemo& memo,const Region& tl){
    Region region;region.keys=tl.keys;region.masks=tl.masks;
    std::uint64_t h=1469598103934665603ull;auto mix=[&](std::uint64_t v){h=core::quantMix(h^v);};
    std::vector<std::uint32_t> active;
    for(std::size_t i=0;i<tl.keys.size();++i){const Cell c=tl.keys[i];const auto& f=tileFaces(grid,tree,data.clean,memo,c);
        mix(std::uint64_t(std::uint32_t(c.x))|(std::uint64_t(std::uint32_t(c.y))<<32));mix(std::uint64_t(std::uint32_t(c.z)));for(auto word:tl.masks[i].removed)mix(word);
        active.insert(active.end(),f.active.begin(),f.active.end());region.selected.insert(region.selected.end(),f.selected.begin(),f.selected.end());}
    std::sort(active.begin(),active.end());active.erase(std::unique(active.begin(),active.end()),active.end());
    region.faces=active;for(auto f:active)for(auto r=data.ringFirst[f];r<data.ringFirst[f+1];++r)region.faces.push_back(data.ringFaces[r]);
    region.selected.insert(region.selected.end(),region.faces.begin(),region.faces.end());
    for(auto* list:{&region.faces,&region.selected}){std::sort(list->begin(),list->end());list->erase(std::unique(list->begin(),list->end()),list->end());}
    for(auto f:region.faces)mix(0x9E3779B97F4A7C15ull^f);mix(0xFA11BAC4ull);
    region.hash=h;return region;
}
// Numerical defects a region's gate does not see: an exterior triangle whose
// float32 orientation opposes its source face (a flipped sliver).
inline std::string regionDefect(const Asset& asset,const RegionResult& r){
    if(r.defectChecked)return r.exactDefect;
    auto pos=[&](std::uint32_t row){float p[3];std::memcpy(p,&r.rows[row],sizeof p);return Vec{p[0],p[1],p[2]};};
    for(const auto& g:r.groups){if(g.first.first!=0)continue;
        for(const auto& t:g.second){if(t.face>=asset.source.f.size())continue;const auto& ids=asset.source.f[t.face];
            const Vec n=cross(asset.source.v[ids[1]].p-asset.source.v[ids[0]].p,asset.source.v[ids[2]].p-asset.source.v[ids[0]].p);
            const Vec a=pos(t.row[0]),b=pos(t.row[1]),c=pos(t.row[2]);const Vec m=cross(b-a,c-a);
            if(length(m)>=1e-14&&dot(m,n)<=0)return "flipped_exterior_triangle";}}
    return {};
}
// Exact packMulti over: region triangles (exterior merged by original face,
// cavity concatenated in region order) followed by untouched source faces.
// Display list for assemble(): every chunked cluster becomes ONE result, its chunks' reduced meshes welded
// (identical seam rows merge) and reduced again without seam locks, so seams cost no triangles and the rim
// is seen whole. Cached per cluster; the chunk results themselves stay the join/reuse unit.
inline RegionList mergedDisplayList(const Asset& asset,const std::vector<Region>& regions,const RegionList& results,TileRegionState& state){
    // Merge groups: blocks of 3x3x3 chunks of one cluster (each re-reduced only when one of its chunks
    // changed; block planes are chunk planes, whose thinned seam vertices both blocks share and keep). A
    // cluster holding a boxed (failure) region merges whole.
    std::map<std::uint64_t,std::vector<std::size_t>> clusters;RegionList out;out.reserve(results.size());
    std::unordered_set<std::uint64_t> boxedClusters;for(std::size_t n=0;n<results.size();++n)if(regions[n].chunked&&regions[n].boxed)boxedClusters.insert(regions[n].cluster);
    const int ct=chunkTiles()*8;
    auto floorDiv=[](int x,int d){return x>=0?x/d:-((-x+d-1)/d);};
    std::unordered_map<std::uint64_t,std::array<int,3>> blockOf;
    for(std::size_t n=0;n<results.size();++n){if(!(regions[n].chunked&&results[n]&&results[n]->chunked)||regions[n].boxed||ct<=0){out.push_back(results[n]);continue;}
        std::array<int,3> b{0,0,0};for(int a=0;a<3;++a)b[std::size_t(a)]=floorDiv(floorDiv(regions[n].seamLo[a],ct),3);
        const std::uint64_t key=core::quantMix(regions[n].cluster^(std::uint64_t(std::uint32_t(b[0]))|(std::uint64_t(std::uint32_t(b[1]))<<21)|(std::uint64_t(std::uint32_t(b[2]))<<42)));
        clusters[key].push_back(n);blockOf[key]=b;}
    std::unordered_map<std::uint64_t,TileRegionState::MergedDisplay> next;
    // Unchanged blocks reuse their result; changed blocks are rebuilt concurrently, gathered in key order.
    struct Job {std::uint64_t key;const std::vector<std::size_t>* regionsOf;std::vector<const RegionResult*> members;std::shared_ptr<const RegionResult> result;};
    std::vector<Job> jobs;jobs.reserve(clusters.size());
    for(auto& c:clusters){Job job;job.key=c.first;job.regionsOf=&c.second;for(auto n:c.second)job.members.push_back(results[n].get());
        auto old=state.mergedDisplay.find(c.first);if(old!=state.mergedDisplay.end()&&old->second.members==job.members)job.result=old->second.result;jobs.push_back(std::move(job));}
    auto buildBlock=[&](Job& job){const auto& members=job.members;const auto& c=*job.regionsOf;const std::uint64_t blockKey=job.key;
        auto merged=std::make_shared<RegionResult>();auto& m=*merged;m.tileLocal=true;
        std::map<GroupKey,std::vector<PackedTri>> groups;
        std::size_t totalRows=0;for(const auto* r:members)totalRows+=r->rows.size();
        std::size_t cap=16;while(cap<2*totalRows+16)cap*=2;std::vector<std::uint32_t> slots(cap,~0u);const std::size_t mask=cap-1;
        m.rows.reserve(totalRows);m.hashes.reserve(totalRows);m.discarded.reserve(totalRows);
        for(const auto* r:members){std::vector<std::uint32_t> map(r->rows.size());
            for(std::size_t k=0;k<r->rows.size();++k){const auto h=r->hashes[k];std::size_t at=std::size_t(h)&mask;std::uint32_t id=~0u;
                for(;slots[at]!=~0u;at=(at+1)&mask){const auto cand=slots[at];if(m.hashes[cand]==h&&!std::memcmp(&m.rows[cand],&r->rows[k],sizeof(gfy_packed::Vertex))){id=cand;break;}}
                if(id==~0u){id=std::uint32_t(m.rows.size());slots[at]=id;m.rows.push_back(r->rows[k]);m.hashes.push_back(h);m.discarded.push_back(r->discarded[k]);}
                map[k]=id;}
            for(const auto& g:r->groups){auto& list=groups[g.first];for(auto t:g.second){for(auto& row:t.row)row=map[row];t.rect=~0u;list.push_back(t);}}
            m.faces.insert(m.faces.end(),r->faces.begin(),r->faces.end());m.selected.insert(m.selected.end(),r->selected.begin(),r->selected.end());
            m.keys.insert(m.keys.end(),r->keys.begin(),r->keys.end());}
        for(auto* list:{&m.faces,&m.selected}){std::sort(list->begin(),list->end());list->erase(std::unique(list->begin(),list->end()),list->end());}
        for(auto& g:groups){m.triangles+=g.second.size();if(g.first.first)m.cavityTriangles+=g.second.size();m.groups.emplace_back(g.first,std::move(g.second));}
        // Direct members: the smallest wall feature radius and every member's locked skin points.
        {double sum=0,weight=0;bool direct=false;for(const auto* r:members)if(r->directDisplay){direct=true;if(r->lowpolyInput||r->cavityTriangles){const double wgt=double(std::max<std::size_t>(r->lowpolyInput,r->cavityTriangles));sum+=wgt*r->featureRadius;weight+=wgt;}
                m.lockedSkinKeys.insert(m.lockedSkinKeys.end(),r->lockedSkinKeys.begin(),r->lockedSkinKeys.end());}
            if(direct){m.directDisplay=true;m.featureRadius=weight>0?sum/weight:.05;std::sort(m.lockedSkinKeys.begin(),m.lockedSkinKeys.end());
                m.lockedSkinKeys.erase(std::unique(m.lockedSkinKeys.begin(),m.lockedSkinKeys.end()),m.lockedSkinKeys.end());}}
        // Group locks: vertices on chunk planes toward chunks outside this group stay (shared with them).
        {const auto& first=*members.front();const auto n0=c.front();(void)blockKey;
            for(int a=0;a<3;++a){const double pitch=(first.seamHiM[a]-first.seamLoM[a])/double(regions[n0].seamHi[a]-regions[n0].seamLo[a]);
                m.chunkOrigin[a]=first.seamLoM[a]-regions[n0].seamLo[a]*pitch;m.chunkSize=ct*pitch;}
            for(auto n:c)m.lockMembers.push_back(Cell{floorDiv(regions[n].seamLo[0],ct),floorDiv(regions[n].seamLo[1],ct),floorDiv(regions[n].seamLo[2],ct)});
            std::sort(m.lockMembers.begin(),m.lockMembers.end());m.lockMembers.erase(std::unique(m.lockMembers.begin(),m.lockMembers.end()),m.lockMembers.end());}
        lowpoly::reduceRegion(asset,m);
#ifdef GFY_VOXEL_STAGE_PROFILE
        std::cout<<"lowpoly_cluster chunks="<<members.size()<<" in="<<m.lowpolyInput<<" out="<<m.triangles<<" cavity="<<m.cavityTriangles<<" deep="<<m.lowpolyDeepIn<<">"<<m.lowpolyDeepOut<<">"<<m.lowpolyDeepFinal<<" deep_ms="<<m.lowpolyDeepMs<<" error_mm="<<m.lowpolyError*1e3<<" limit_mm="<<m.lowpolyLimit*1e3<<" ms="<<m.lowpolyMs<<"\n";
#endif
        job.result=merged;};
    {std::vector<std::size_t> todo;for(std::size_t k=0;k<jobs.size();++k)if(!jobs[k].result)todo.push_back(k);std::atomic<std::size_t> cursor{0};
     core::parallelChunks(todo.size()>=2?std::min<std::size_t>(todo.size(),core::parallelWidth()):1,1,[&](std::size_t,std::size_t,std::size_t){for(std::size_t k;(k=cursor++)<todo.size();)buildBlock(jobs[todo[k]]);});}
    for(auto& job:jobs){next[job.key]={job.members,job.result};out.push_back(job.result);}
    state.mergedDisplay=std::move(next);
    return out;
}
inline gfy_packed::Mesh assemble(const Asset& asset,const AssetData& data,const RegionList& regions,std::size_t& untouchedCount,TileRegionState* tileState=nullptr){
    const auto& faceChildren=asset.face_children;const auto& faceIndices=asset.face_indices;
    std::vector<char> inPatch(faceChildren.size(),0),selectedFace(faceChildren.size(),0);
    for(const auto& region:regions){for(auto f:region->faces)inPatch[f]=1;for(auto f:region->selected)selectedFace[f]=1;}
    std::map<GroupKey,std::vector<TriRef>> groups;std::vector<char> touched(data.child_faces.size(),0);
    for(std::size_t c=0;c<data.child_faces.size();++c)if(!data.child_faces[c].empty())groups[{0,std::uint16_t(c)}]; // same key set as one entry per face
    for(std::uint32_t f=0;f<faceChildren.size();++f)if(selectedFace[f])touched[faceChildren[f]]=1;
    auto ref=[&](const RegionResult& region,const PackedTri& t){TriRef out;
        for(int k=0;k<3;++k){out.row[k]=&region.rows[t.row[k]];out.hash[k]=region.hashes[t.row[k]];out.discarded[k]=region.discarded[t.row[k]];}
        out.original_face=t.face==~std::uint32_t(0)?~std::uint32_t(0):faceIndices[t.face];return out;};
    // Exterior: a face belongs to exactly one region; stable order by face.
    std::map<GroupKey,std::vector<std::pair<std::uint32_t,TriRef>>> exterior;
    for(const auto& region:regions)for(const auto& g:region->groups){
        if(g.first.first==0){auto& list=exterior[g.first];for(const auto& t:g.second)list.emplace_back(t.face,ref(*region,t));}
        else{auto& list=groups[g.first];for(const auto& t:g.second)list.push_back(ref(*region,t));}}
    // Cavity groups: the union path emitted merged rectangles plane by plane,
    // each at the first row of its plane (over ALL regions) at or after its
    // end row, ties by run. Reproduce that order across regions.
    if(regions.size()>1){
        std::map<PlaneKey,std::vector<int>> unionRows;
        for(const auto& region:regions)for(const auto& p:region->planeRows){auto& rows=unionRows[p.first];rows.insert(rows.end(),p.second.begin(),p.second.end());}
        for(auto& p:unionRows){std::sort(p.second.begin(),p.second.end());p.second.erase(std::unique(p.second.begin(),p.second.end()),p.second.end());}
        for(auto& item:groups){if(item.first.first==0)continue;
            // Triangles of one rectangle are contiguous; order whole runs.
            struct Run {PlaneKey plane;int emit,u0,u1;const RegionResult* region;const PackedTri* begin;const PackedTri* end;};
            std::vector<Run> runs;bool located=true;std::size_t total=0;
            for(const auto& region:regions)for(const auto& g:region->groups)if(g.first==item.first){total+=g.second.size();
                for(std::size_t i=0;i<g.second.size()&&located;){const auto rect=g.second[i].rect;if(rect==~0u){located=false;break;}
                    std::size_t j=i+1;while(j<g.second.size()&&g.second[j].rect==rect)++j;
                    const auto& m=region->rects[rect];const auto& rows=unionRows[m.plane];auto at=std::lower_bound(rows.begin(),rows.end(),m.v1);
                    runs.push_back({m.plane,at==rows.end()?std::numeric_limits<int>::max():*at,m.u0,m.u1,region.get(),&g.second[i],g.second.data()+j});i=j;}}
            if(!located||total!=item.second.size())continue;
            std::stable_sort(runs.begin(),runs.end(),[](const Run& a,const Run& b){return std::tie(a.plane,a.emit,a.u0,a.u1)<std::tie(b.plane,b.emit,b.u0,b.u1);});
            std::size_t at=0;for(const auto& run:runs)for(auto t=run.begin;t!=run.end;++t)item.second[at++]=ref(*run.region,*t);}
    }
    untouchedCount=0;
    for(auto& item:groups){if(item.first.first!=0)continue;const auto child=item.first.second;if(!touched[child]){untouchedCount+=data.child_faces[child].size();continue;}
        auto& list=item.second;auto& e=exterior[item.first];
        if(tileState){
            // Tile-local: per face, its joined pieces (all regions) then its
            // static remainder; faces not joined keep their untouched rows.
            std::stable_sort(e.begin(),e.end(),[](const auto& a,const auto& b){return a.first<b.first;});std::size_t at=0;
            for(auto f:data.child_faces[child]){
                if(!inPatch[f]){list.push_back(data.untouched(f,faceIndices));++untouchedCount;continue;}
                while(at<e.size()&&e[at].first<f)++at;while(at<e.size()&&e[at].first==f)list.push_back(e[at++].second);
                if(tileState->current.count(f))appendStatic(*tileState,f,faceIndices,list,untouchedCount);}
            continue;}
        // Former union order: every selected face (clipped fragments of active
        // faces, untouched rows of passive ones) by face index, then the rest.
        for(auto f:data.child_faces[child])if(selectedFace[f]&&!inPatch[f]){e.emplace_back(f,data.untouched(f,faceIndices));++untouchedCount;}
        std::stable_sort(e.begin(),e.end(),[](const auto& a,const auto& b){return a.first<b.first;});
        list.reserve(e.size()+data.child_faces[child].size());for(auto& t:e)list.push_back(t.second);
        for(auto f:data.child_faces[child])if(!selectedFace[f]){list.push_back(data.untouched(f,faceIndices));++untouchedCount;}}
    // Touched groups pack concurrently into meshes of their own (a group's children, and which of them take a child
    // slot, are pure functions of its triangles), then take their slots in group order, exactly as one serial loop.
    std::vector<const decltype(groups)::value_type*> items;items.reserve(groups.size());for(const auto& item:groups)items.push_back(&item);
    std::vector<gfy_packed::Mesh> packed(items.size());
    {std::atomic<std::size_t> cursor{0};
     core::parallelChunks(std::min<std::size_t>(items.size(),core::parallelWidth()),1,[&](std::size_t,std::size_t,std::size_t){VertexIndex vertices;
        for(std::size_t k;(k=cursor++)<items.size();){const auto& item=*items[k];if(item.first.first==0&&!touched[item.first.second])continue;
            unsigned provisional=0;packGroup(packed[k],provisional,vertices,item.first.first,item.first.second,item.second,true);}});}
    gfy_packed::Mesh out;unsigned next=asset.original_children;
    for(std::size_t k=0;k<items.size();++k){const auto& item=*items[k];
        if(item.first.first==0&&!touched[item.first.second]){const auto& s=data.static_child[item.first.second];
            for(std::size_t n=0;n<s.chunks.size();++n){auto chunk=s.chunks[n];
                if(n){core::packing::packingRequire(next<gfy_packed::max_child_slots,"Runtime child slots exhausted");chunk.child_index=std::uint16_t(next++);}
                out.children.push_back(std::move(chunk));}
            out.reduced_skin_vertices+=s.reduced;out.max_discarded_skin_weight=std::max(out.max_discarded_skin_weight,s.max_discarded);continue;}
        auto& m=packed[k];
        for(auto& child:m.children){
            if(child.child_index==provisionalSlot){core::packing::packingRequire(next<gfy_packed::max_child_slots,"Runtime child slots exhausted");child.child_index=std::uint16_t(next++);}
            out.children.push_back(std::move(child));}
        out.reduced_skin_vertices+=m.reduced_skin_vertices;out.max_discarded_skin_weight=std::max(out.max_discarded_skin_weight,m.max_discarded_skin_weight);}
    return out;
}
}
