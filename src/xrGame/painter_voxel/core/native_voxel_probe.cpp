#include "../../../Include/xrRender/RuntimeSignedSkin.h"
// Isolated CPU kernel, not a linked engine feature. No physics or game pointers.
#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <climits>
#ifdef _MSC_VER
#include <intrin.h>
#endif
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include "SparseMaterialVolume.h"

using Clock=std::chrono::steady_clock;
struct StageProfile {
#ifdef GFY_VOXEL_STAGE_PROFILE
    const char*group;Clock::time_point last=Clock::now();
    explicit StageProfile(const char*g):group(g){}
    void mark(const char*stage){auto now=Clock::now();std::cout<<"voxel_stage "<<group<<'.'<<stage<<' '<<std::chrono::duration<double,std::milli>(now-last).count()<<'\n';last=now;}
#else
    explicit StageProfile(const char*){}void mark(const char*){}
#endif
};

struct Vec { double x{},y{},z{}; double& operator[](int i){return (&x)[i];} double operator[](int i)const{return (&x)[i];} };
Vec operator+(Vec a,Vec b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
Vec operator-(Vec a,Vec b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
Vec operator*(Vec a,double s){return {a.x*s,a.y*s,a.z*s};}
double dot(Vec a,Vec b){return a.x*b.x+a.y*b.y+a.z*b.z;}
Vec cross(Vec a,Vec b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
double length(Vec a){return std::sqrt(dot(a,a));}
using Quant=std::array<std::int64_t,3>;
Quant quant(Vec p){return {std::llround(p.x*1e8),std::llround(p.y*1e8),std::llround(p.z*1e8)};}
Vec snap(Vec p){auto q=quant(p);return {q[0]*1e-8,q[1]*1e-8,q[2]*1e-8};}
struct Cell { int x{},y{},z{}; int operator[](int i)const{return (&x)[i];} bool operator==(const Cell& b)const{return x==b.x&&y==b.y&&z==b.z;} bool operator<(const Cell&b)const{return x!=b.x?x<b.x:(y!=b.y?y<b.y:z<b.z);} }; // lexicographic (x,y,z); inline: std::array's < calls the out-of-line vectorized mismatch
struct CellHash { std::size_t operator()(Cell c)const{return (std::size_t(c.x)*73856093u)^(std::size_t(c.y)*19349663u)^(std::size_t(c.z)*83492791u);} };
struct Tile { std::array<std::uint64_t,8> removed{}; };
// Exact surviving-component ownership in the original rest-space lattice.
// Columns are sorted by (y,x); their half-open z spans do not overlap.
struct OwnerSpan {int lo{},hi{};};
struct OwnerColumn {int x{},y{};std::uint32_t first{},count{};};
struct PieceOwnership {
    // Parent source triangles cannot be used to publish a wound on this grid.
    bool source_is_piece_local{};
    std::vector<OwnerColumn> body_columns,cloth_columns;
    std::vector<OwnerSpan> body_spans,cloth_spans;
    static bool in(const std::vector<OwnerColumn>&columns,const std::vector<OwnerSpan>&spans,Cell c) noexcept {
        auto col=std::lower_bound(columns.begin(),columns.end(),c,[](const OwnerColumn&a,Cell b){return a.y<b.y||(a.y==b.y&&a.x<b.x);});
        if(col==columns.end()||col->x!=c.x||col->y!=c.y)return false;
        auto first=spans.begin()+col->first,last=first+col->count;
        auto run=std::upper_bound(first,last,c.z,[](int z,const OwnerSpan&s){return z<s.lo;});
        if(run==first)return false;--run;return c.z<run->hi;
    }
    bool body(Cell c)const noexcept{return in(body_columns,body_spans,c);}
    bool cuttable(Cell c)const noexcept{return body(c)||in(cloth_columns,cloth_spans,c);}
    // Tiles (8^3 cells) of the box [pure_lo, pure_lo+pure_size) holding no other component's remaining tissue: there
    // a cell that is not stored removed is this owner's or air (fast path of Grid::ownerCuttable and materialBase).
    Cell pure_lo{},pure_size{};std::vector<std::uint64_t> pure;
    bool pureTile(Cell c)const noexcept{const int x=(c.x>>3)-pure_lo.x,y=(c.y>>3)-pure_lo.y,z=(c.z>>3)-pure_lo.z;
        if(pure.empty()||x<0||y<0||z<0||x>=pure_size.x||y>=pure_size.y||z>=pure_size.z)return false;
        const std::size_t i=(std::size_t(x)*pure_size.y+std::size_t(y))*pure_size.z+std::size_t(z);return ((pure[i>>6]>>(i&63))&1u)!=0;}
};
template<class T>T read(std::ifstream& f){T v{};f.read(reinterpret_cast<char*>(&v),sizeof(v));if(!f)throw std::runtime_error("Truncated input");return v;}
struct SourceVertex {Vec p;Vec normal,tangent,binormal;std::array<float,2>uv{};std::array<std::uint16_t,4>ids{};std::array<float,4>w{};bool authored_hardware{};};
struct Source {std::vector<SourceVertex>v;std::vector<std::array<std::uint32_t,3>>f;std::uint32_t bones{};};
Source loadSource(const std::string& path){
    std::ifstream in(path,std::ios::binary);auto magic=read<std::array<char,8>>(in);const bool hw=std::string(magic.data(),8)=="VXHEAD02";if(!hw&&std::string(magic.data(),8)!="VXHEAD01")throw std::runtime_error("Bad geometry magic");
    Source s;auto nv=read<std::uint32_t>(in),nf=read<std::uint32_t>(in);s.bones=read<std::uint32_t>(in);if(s.bones>64)throw std::runtime_error("Probe supports up to64 bones");
    s.v.resize(nv);s.f.resize(nf);for(auto&v:s.v){v.p={read<float>(in),read<float>(in),read<float>(in)};v.uv=read<std::array<float,2>>(in);v.ids=read<std::array<std::uint16_t,4>>(in);v.w=read<std::array<float,4>>(in);v.authored_hardware=hw;if(hw&&!gfy_signed_hw::valid4(v.w,true))throw std::runtime_error("Invalid authored hardware skin");}
    for(auto&f:s.f)f=read<std::array<std::uint32_t,3>>(in);return s;
}
struct Grid {
    Cell size;Vec origin;double pitch{};std::shared_ptr<const std::vector<std::uint8_t>>base;
    std::shared_ptr<const material_volume::Sparse> anatomy;
    std::shared_ptr<const std::vector<std::uint16_t>> material_slots;
    std::shared_ptr<const PieceOwnership> ownership;
    using FloorTiles=std::unordered_map<Cell,Tile,CellHash>;
// Included by Grid. Damage cells retain the identity of the first round that loaded them.
    struct BoneShot {
        std::uint64_t id{};
        bool local{};
        std::uint64_t owner{};
        bool operator==(const BoneShot& other) const {
            return id == other.id && local == other.local && owner == other.owner;
        }
    };
    struct BoneContactTile {
        Tile all;
        std::vector<std::pair<BoneShot,Tile>> first;
    };
    using BoneBucket = std::unordered_map<Cell,BoneContactTile,CellHash>;
    struct BoneBuckets { std::array<std::shared_ptr<BoneBucket>,32> pages; };
    std::shared_ptr<BoneBuckets> boneStrikes;
    BoneShot boneShot;
    const BoneContactTile* boneContacts(Cell k) const {
        if (!boneStrikes) return nullptr;
        const auto& page=boneStrikes->pages[CellHash{}(k)%32];
        if (!page) return nullptr;
        const auto it=page->find(k);
        return it==page->end() ? nullptr : &it->second;
    }
    std::uint64_t boneStrikeWord(Cell k,unsigned word) const {
        const auto* tile=boneContacts(k);
        return tile ? tile->all.removed[word] : 0;
    }
    bool bonePreviouslyHit(Cell c) const {
        if (!boneShot.id) return false;
        const auto* tile=boneContacts(tileKey(c));
        const auto bit=local(c);
        const auto mask=std::uint64_t(1)<<(bit%64);
        if (!tile || !(tile->all.removed[bit/64]&mask)) return false;
        for (const auto& first:tile->first)
            if (first.first==boneShot) return !(first.second.removed[bit/64]&mask);
        return true;
    }
    void noteBoneStrike(Cell c) {
        if (!boneShot.id) return;
        const auto k=tileKey(c);
        const auto bit=local(c);
        const auto mask=std::uint64_t(1)<<(bit%64);
        if (boneStrikeWord(k,bit/64)&mask) return;
        if (!boneStrikes) boneStrikes=std::make_shared<BoneBuckets>();
        else if (!boneStrikes.unique()) boneStrikes=std::make_shared<BoneBuckets>(*boneStrikes);
        auto& page=boneStrikes->pages[CellHash{}(k)%32];
        if (!page) page=std::make_shared<BoneBucket>();
        else if (!page.unique()) page=std::make_shared<BoneBucket>(*page);
        auto& tile=(*page)[k];
        auto it=std::find_if(tile.first.begin(),tile.first.end(),[&](const auto& e){return e.first==boneShot;});
        if (it==tile.first.end()) it=tile.first.emplace(tile.first.end(),boneShot,Tile{});
        it->second.removed[bit/64]|=mask;
        tile.all.removed[bit/64]|=mask;
    }
    FloorTiles tiles;
    bool lodAttached = true;
    std::shared_ptr<const FloorTiles> blowoutTags;

    // The outer hit journal may already contain the bullet tunnel. A nested journal
    // gives the crater its own before-image even when the two touch the same tile.
    template<class Cut> auto captureBulletCut(Cut&& cut){
        if(!lodAttached)return cut();
        std::unordered_map<Cell,std::pair<bool,Tile>,CellHash> before;
        auto* parent=journal;
        journal=&before;
        try {
            auto result=cut();
            journal=parent;
            if(result.done&&!result.refusal){
                std::shared_ptr<FloorTiles> next;
                for(const auto& entry:before){
                    const auto* current=findTile(entry.first);
                    if(!current)continue;
                    Tile delta;std::uint64_t any=0;
                    for(unsigned word=0;word<8;++word){
                        const auto old=entry.second.first?entry.second.second.removed[word]:0;
                        delta.removed[word]=current->removed[word]&~old;
                        any|=delta.removed[word];
                    }
                    if(!any)continue;
                    if(!next)next=blowoutTags?std::make_shared<FloorTiles>(*blowoutTags):std::make_shared<FloorTiles>();
                    // Untracked cells keep full detail if the optional map fills.
                    if(next->size()>=8192&&next->find(entry.first)==next->end())continue;
                    auto& saved=(*next)[entry.first];
                    for(unsigned word=0;word<8;++word)saved.removed[word]|=delta.removed[word];
                }
                if(next)blowoutTags=std::move(next);
            }
            if(parent)parent->merge(before);
            return result;
        } catch(...) {
            journal=parent;
            if(parent)parent->merge(before);
            throw;
        }
    }

    // Cosmetic cells of an irregular crater that retain their original material.
    // Physics, damage admission and connectivity read only tiles.
    std::shared_ptr<const FloorTiles> visibleFloor;
    static void applyDisplayTile(Tile& physical,const Tile& relief){
        for(unsigned i=0;i<8;++i)physical.removed[i]&=~relief.removed[i];
    }
    // These are copies made for publication. Foreign and ownership marks are added afterwards.
    void applyDisplayFloor(FloorTiles& display)const{
        if(!visibleFloor)return;
        for(const auto& item:*visibleFloor){
            auto found=display.find(item.first);
            if(found!=display.end())applyDisplayTile(found->second,item.second);
        }
    }
    // Undo journal (optional): the prior state of every tile erase() first touches (existed?, tile). Lets a
    // failed cut roll back without copying the whole grid.
    std::unordered_map<Cell,std::pair<bool,Tile>,CellHash>* journal{};
    // Copy-on-write overlay (parallel tear lanes, LiveDamageWorker.cpp): a tile absent from `tiles` is read from
    // overlay_base, and the first write to it copies it in, so a lane holds only the tiles it changed instead of a full
    // copy of the grid. Every tile access goes through findTile()/writeTile() (or TileCache/TileWindow, which use them).
    const Grid* overlay_base{};
    const Tile* findTile(Cell k)const{auto it=tiles.find(k);if(it!=tiles.end())return &it->second;
        if(overlay_base)return overlay_base->findTile(k);return nullptr;}
    Tile& writeTile(Cell k){auto it=tiles.find(k);if(it!=tiles.end())return it->second;
        if(overlay_base)if(const Tile* b=overlay_base->findTile(k))return tiles.emplace(k,*b).first->second;return tiles[k];}
    bool valid(Cell c)const{return c.x>=0&&c.y>=0&&c.z>=0&&c.x<size.x&&c.y<size.y&&c.z<size.z;}
    std::uint16_t rawMaterialBase(Cell c)const{if(!valid(c))return 0;if(anatomy){const auto p=center(c);return anatomy->point(p.x,p.y,p.z);}const auto i=(std::size_t(c.x)*size.y+c.y)*size.z+c.z;return base&&(((*base)[i/8]>>(i%8))&1u)?1:0;}
    std::uint16_t materialBase(Cell c)const{return ownership&&!ownership->pureTile(c)&&!ownership->body(c)?0:rawMaterialBase(c);}
    bool solidBase(Cell c)const{return materialBase(c)!=0;}
    unsigned renderMaterial(Cell c)const{auto label=materialBase(c);return label&&material_slots?(*material_slots)[label]:1;}
    static Cell tileKey(Cell c){return {c.x/8,c.y/8,c.z/8};}
    static unsigned local(Cell c){return unsigned(((c.x&7)*8+(c.y&7))*8+(c.z&7));}
    // The tunnel may mark original anatomical air, needed by the exact
    // whole-parent skin join. Foreign surviving anatomy remains immutable.
    bool ownerCuttable(Cell c)const{return !ownership||ownership->pureTile(c)||ownership->cuttable(c)||rawMaterialBase(c)==0;}
    bool removed(Cell c)const{if(!valid(c))return false;if(!ownerCuttable(c))return true;const Tile* t=findTile(tileKey(c));if(!t)return false;auto bit=local(c);return (t->removed[bit/64]>>(bit%64))&1u;}
    bool erase(Cell c){if(!valid(c)||!ownerCuttable(c))return false;
        if(journal){const Cell k=tileKey(c);if(!journal->count(k)){const Tile* t=findTile(k);if(!t)journal->emplace(k,std::make_pair(false,Tile{}));else journal->emplace(k,std::make_pair(true,*t));}}
        auto bit=local(c);auto&word=writeTile(tileKey(c)).removed[bit/64];auto mask=std::uint64_t(1)<<(bit%64);bool fresh=(word&mask)==0;word|=mask;return fresh;}
    Vec center(Cell c)const{return origin+Vec{c.x+.5,c.y+.5,c.z+.5}*pitch;}
};
// A copy-on-write view of `base` (Grid::overlay_base): the same lattice, anatomy and ownership, no tiles of its own.
inline Grid overlayOf(const Grid& base){Grid g;g.size=base.size;g.origin=base.origin;g.pitch=base.pitch;g.base=base.base;g.anatomy=base.anatomy;
    g.material_slots=base.material_slots;g.ownership=base.ownership;g.boneStrikes=base.boneStrikes;g.boneShot=base.boneShot;g.overlay_base=&base;return g;}
// GFY_TEAR_OVERLAY=0: the parallel tear lanes copy the whole grid as before (A/B measurements; results are identical).
inline bool tearOverlayEnabled(){static const bool on=[]{const char* e=std::getenv("GFY_TEAR_OVERLAY");return !e||std::atoi(e)!=0;}();return on;}
Grid loadGrid(const std::string& path){
    std::ifstream in(path,std::ios::binary);auto magic=read<std::array<char,8>>(in);if(std::string(magic.data(),8)!="VXBASE01")throw std::runtime_error("Bad base magic");
    Grid g;g.size={int(read<std::uint32_t>(in)),int(read<std::uint32_t>(in)),int(read<std::uint32_t>(in))};g.origin={read<double>(in),read<double>(in),read<double>(in)};g.pitch=read<double>(in);
    auto storage=std::make_shared<std::vector<std::uint8_t>>((std::size_t(g.size.x)*g.size.y*g.size.z+7)/8);in.read(reinterpret_cast<char*>(storage->data()),std::streamsize(storage->size()));if(!in)throw std::runtime_error("Truncated base");g.base=std::move(storage);return g;
}
// ---- Tile-mask access helpers (raster-exterior latency work) ----
// All helpers read/write the SAME Grid::tiles representation (512 removed bits
// per 8^3 tile); they only replace repeated hash lookups by cached pointers.
// Stored removed bit of a VALID cell. Callers apply ownerCuttable semantics.
inline unsigned lowestBit(std::uint64_t x){
#ifdef _MSC_VER
    unsigned long b;_BitScanForward64(&b,x);return unsigned(b);
#else
    return unsigned(__builtin_ctzll(x));
#endif
}
inline bool tileBit(const Tile&t,Cell c){const auto l=Grid::local(c);return ((t.removed[l/64]>>(l%64))&1u)!=0;}
// Direct-mapped cache of tile pointers (including misses). Never use across a
// grid mutation that may create tiles: a cached miss would become stale.
struct TileCache {
    const Grid&g;struct Slot{Cell key{INT_MIN,INT_MIN,INT_MIN};const Tile*tile=nullptr;};std::array<Slot,64>slots{};
    explicit TileCache(const Grid&grid):g(grid){}
    const Tile*find(Cell k){auto&s=slots[(unsigned(k.x)*7u+unsigned(k.y)*13u+unsigned(k.z)*29u)&63u];
        if(!(s.key==k)){s.key=k;s.tile=g.findTile(k);}return s.tile;}
    bool bit(Cell c){const Tile*t=find(Grid::tileKey(c));return t&&tileBit(*t,c);}
    // Exactly Grid::removed(c).
    bool removed(Cell c){if(!g.valid(c))return false;if(g.ownership&&!g.ownerCuttable(c))return true;return bit(c);}
};
// Dense tile-pointer window covering a cell box plus a one-cell halo. It is an
// exact replacement for Grid::tiles lookups inside the window and creates
// tiles through Grid::tiles exactly when the former code did. The window is
// disabled above maxTiles (then every access hashes), bounding its memory.
struct TileWindow {
    Grid&g;Cell lo{},n{};std::vector<Tile*>ptr;bool enabled=false;
    TileWindow(Grid&grid,Cell low,Cell high,std::size_t maxTiles=std::size_t(1)<<20):g(grid){
        if(g.overlay_base)return; // overlay grids: every access through findTile/writeTile
        std::size_t count=1;
        for(int a=0;a<3;++a){const int l=std::max(0,low[a]-1)/8,h=std::max(0,high[a]+1)/8;(&lo.x)[a]=l;(&n.x)[a]=h-l+1;count*=std::size_t((&n.x)[a]);if(count>maxTiles)return;}
        ptr.assign(count,nullptr);enabled=true;
        if(count<g.tiles.size()){for(int x=0;x<n.x;++x)for(int y=0;y<n.y;++y)for(int z=0;z<n.z;++z){auto it=g.tiles.find({lo.x+x,lo.y+y,lo.z+z});if(it!=g.tiles.end())ptr[index({lo.x+x,lo.y+y,lo.z+z})]=&it->second;}}
        else for(auto&item:g.tiles)if(inside(item.first))ptr[index(item.first)]=&item.second;
    }
    bool inside(Cell k)const{return k.x>=lo.x&&k.y>=lo.y&&k.z>=lo.z&&k.x<lo.x+n.x&&k.y<lo.y+n.y&&k.z<lo.z+n.z;}
    std::size_t index(Cell k)const{return (std::size_t(k.x-lo.x)*std::size_t(n.y)+std::size_t(k.y-lo.y))*std::size_t(n.z)+std::size_t(k.z-lo.z);}
    const Tile*find(Cell k)const{if(enabled&&inside(k))return ptr[index(k)];return g.findTile(k);}
    // Stored bit of a VALID cell.
    bool bit(Cell c)const{const Tile*t=find(Grid::tileKey(c));return t&&tileBit(*t,c);}
    // Same effect as Grid::tiles[k]: creates the tile when absent.
    // Journaled like Grid::erase (the first write access records the tile's prior state).
    Tile&tile(Cell k){if(g.journal&&!g.journal->count(k)){const Tile* t=g.findTile(k);if(!t)g.journal->emplace(k,std::make_pair(false,Tile{}));else g.journal->emplace(k,std::make_pair(true,*t));}
        if(enabled&&inside(k)){auto&p=ptr[index(k)];if(!p)p=&g.tiles[k];return *p;}return g.writeTile(k);}
    // True the first time a window tile is reported (always true outside the
    // window, where callers deduplicate by sorting).
    std::vector<std::uint8_t>seen;
    bool firstTouch(Cell k){if(!enabled||!inside(k))return true;if(seen.empty())seen.assign(ptr.size(),0);auto&s=seen[index(k)];if(s)return false;s=1;return true;}
};
// TileWindow::tile without a window: journal the tile's prior state (first write access) and return it, created when
// absent (copied in from an overlay base). For a known list of tiles, where a dense window would cost more to build.
inline Tile& journaledTile(Grid&g,Cell k){
    if(g.journal&&!g.journal->count(k)){const Tile* t=g.findTile(k);if(!t)g.journal->emplace(k,std::make_pair(false,Tile{}));else g.journal->emplace(k,std::make_pair(true,*t));}
    return g.writeTile(k);}
// std::sort + std::unique of a short list with many repeats (per-triangle tile lists of the low-poly pieces): the
// repeats are dropped first with a small open-addressed set, then the few distinct cells are sorted - the same vector.
inline void sortUniqueCells(std::vector<Cell>&v){
    if(v.size()<=24){std::sort(v.begin(),v.end());v.erase(std::unique(v.begin(),v.end()),v.end());return;}
    thread_local std::vector<std::uint64_t> keys;thread_local std::vector<std::uint32_t> used;thread_local std::uint32_t epoch=0;
    std::size_t cap=1024;while(cap<2*v.size())cap*=2;
    if(used.size()<cap){keys.assign(cap,0);used.assign(cap,0);epoch=0;}
    const std::size_t mask=cap-1;const unsigned shift=unsigned(64-__popcnt64(mask));
    if(++epoch==0){std::fill(used.begin(),used.end(),0u);epoch=1;}
    std::size_t out=0;
    for(std::size_t i=0;i<v.size();++i){const Cell c=v[i];
        const std::uint64_t key=(std::uint64_t(std::uint32_t(c.x+(1<<20))&0x1fffffu)<<42)|(std::uint64_t(std::uint32_t(c.y+(1<<20))&0x1fffffu)<<21)|std::uint64_t(std::uint32_t(c.z+(1<<20))&0x1fffffu);
        std::size_t h=std::size_t((key*0x9E3779B97F4A7C15ull)>>shift)&mask;bool seen=false;
        while(used[h]==epoch){if(keys[h]==key){seen=true;break;}h=(h+1)&mask;}
        if(seen)continue;used[h]=epoch;keys[h]=key;v[out++]=c;}
    v.resize(out);std::sort(v.begin(),v.end());
}
// Exactly {t+d : t in touched, d in {-1,0,1}^3}, built in ascending order
// instead of 27 ordered-set insertions per touched tile.
std::set<Cell>expandDirtyTiles(const std::set<Cell>&touched){
    std::set<Cell>out;if(touched.empty())return out;
    Cell lo{INT_MAX,INT_MAX,INT_MAX},hi{INT_MIN,INT_MIN,INT_MIN};
    for(auto t:touched)for(int a=0;a<3;++a){(&lo.x)[a]=std::min(lo[a],t[a]-1);(&hi.x)[a]=std::max(hi[a],t[a]+1);}
    const std::uint64_t nx=std::uint64_t(hi.x-lo.x+1),ny=std::uint64_t(hi.y-lo.y+1),nz=std::uint64_t(hi.z-lo.z+1);
    if(nx*ny*nz>(std::uint64_t(1)<<26)){
        std::vector<Cell>keys;keys.reserve(touched.size()*27);
        for(auto t:touched)for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z)keys.push_back({t.x+x,t.y+y,t.z+z});
        std::sort(keys.begin(),keys.end());keys.erase(std::unique(keys.begin(),keys.end()),keys.end());
        return std::set<Cell>(keys.begin(),keys.end());
    }
    std::vector<std::uint8_t>mark(std::size_t(nx*ny*nz));
    auto at=[&](int x,int y,int z){return (std::size_t(x-lo.x)*ny+std::size_t(y-lo.y))*nz+std::size_t(z-lo.z);};
    for(auto t:touched)for(int x=-1;x<=1;++x)for(int y=-1;y<=1;++y)for(int z=-1;z<=1;++z)mark[at(t.x+x,t.y+y,t.z+z)]=1;
    for(int x=lo.x;x<=hi.x;++x)for(int y=lo.y;y<=hi.y;++y)for(int z=lo.z;z<=hi.z;++z)if(mark[at(x,y,z)])out.emplace_hint(out.end(),Cell{x,y,z});
    return out;
}
struct HitResult {std::set<Cell>dirty;std::size_t freshSolid{},freshMask{};Vec cutter;double surface{};
    std::array<std::size_t,64> freshBySlot{}; // fresh solid cells per render material slot (1 connective, 2 bone, 3 brain, 5 muscle, 7+ organs)
};
HitResult hit(Grid&g,double x,double y,double radius=.017,double depth=.008){
    HitResult result;std::set<Cell>touched;int ix=int(std::floor((x-g.origin.x)/g.pitch)),iy=int(std::floor((y-g.origin.y)/g.pitch));int iz=g.size.z-1;
    // Match closest center rather than floor at an exact half-cell tie.
    auto nearest=[&](double q,double o,int n){int best=0;double distance=1e9;for(int i=0;i<n;++i){double d=std::abs(o+(i+.5)*g.pitch-q);if(d<distance){distance=d;best=i;}}return best;};
    ix=nearest(x,g.origin.x,g.size.x);iy=nearest(y,g.origin.y,g.size.y);
    while(iz>=0&&(!g.solidBase({ix,iy,iz})||g.removed({ix,iy,iz})))--iz;
    if(iz<0)throw std::runtime_error("No solid along impact ray");result.surface=g.center({ix,iy,iz}).z;result.cutter={x,y,result.surface-depth};
    Cell lo,hi;for(int a=0;a<3;++a){(&lo.x)[a]=std::max(0,int(std::floor((result.cutter[a]-radius-g.origin[a])/g.pitch)));(&hi.x)[a]=std::min(g.size[a]-1,int(std::floor((result.cutter[a]+radius-g.origin[a])/g.pitch)));}
    for(int a=lo.x;a<=hi.x;++a)for(int b=lo.y;b<=hi.y;++b)for(int c=lo.z;c<=hi.z;++c){Cell cell{a,b,c};Vec d=g.center(cell)-result.cutter;if(dot(d,d)>radius*radius)continue;
        if(g.erase(cell)){++result.freshMask;if(g.solidBase(cell))++result.freshSolid;touched.insert(Grid::tileKey(cell));}}
    // Expand a unique dirty-tile set once, never27 set operations per voxel.
    for(auto t:touched)for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)for(int dz=-1;dz<=1;++dz)result.dirty.insert({t.x+dx,t.y+dy,t.z+dz});
    return result;
}
struct Quad {std::array<Vec,4>p;Cell removedCell;unsigned material{1};};
std::vector<Quad>mergeBoundaryQuads(const Grid&g,const std::vector<Quad>&input){
    // Exact union of coplanar voxel faces. Preserve every stair step while
    // removing subdivisions along flat runs; no smoothing or density change.
    using Plane=std::array<int,4>; // normal axis, integer plane, normal sign, tissue
    std::map<Plane,std::map<int,std::vector<int>>>planes;
    for(auto&q:input){Vec n=cross(q.p[1]-q.p[0],q.p[2]-q.p[0]);int axis=0;
        for(int a=1;a<3;++a)if(std::abs(n[a])>std::abs(n[axis]))axis=a;
        int u=(axis+1)%3,v=(axis+2)%3;Vec lo=q.p[0];for(auto p:q.p)for(int a=0;a<3;++a)lo[a]=std::min(lo[a],p[a]);
        planes[{axis,int(std::llround((lo[axis]-g.origin[axis])/g.pitch)),n[axis]>0?1:-1,int(q.material)}]
            [int(std::llround((lo[v]-g.origin[v])/g.pitch))].push_back(int(std::llround((lo[u]-g.origin[u])/g.pitch)));
    }
    std::vector<Quad>out;
    for(auto&plane:planes){int axis=plane.first[0],u=(axis+1)%3,v=(axis+2)%3;
        struct Rect{int u0,u1,v0,v1;};std::map<std::pair<int,int>,Rect>active;
        auto emit=[&](Rect r){Quad q;q.material=unsigned(plane.first[3]);int uv[4][2]={{r.u0,r.v0},{r.u1,r.v0},{r.u1,r.v1},{r.u0,r.v1}};
            for(int k=0;k<4;++k){Vec p;p[axis]=plane.first[1];p[u]=uv[k][0];p[v]=uv[k][1];q.p[k]=g.origin+p*g.pitch;}
            if(plane.first[2]<0)std::reverse(q.p.begin(),q.p.end());
            (&q.removedCell.x)[axis]=plane.first[1]-(plane.first[2]>0?1:0);
            (&q.removedCell.x)[u]=r.u0;(&q.removedCell.x)[v]=r.v0;out.push_back(q);
        };
        for(auto&row:plane.second){auto&xs=row.second;std::sort(xs.begin(),xs.end());xs.erase(std::unique(xs.begin(),xs.end()),xs.end());
            std::map<std::pair<int,int>,Rect>next;
            for(std::size_t k=0;k<xs.size();){int lo=xs[k],hi=lo+1;++k;while(k<xs.size()&&xs[k]==hi){++hi;++k;}
                std::pair<int,int> key{lo,hi};auto it=active.find(key);
                if(it!=active.end()&&it->second.v1==row.first){Rect r=it->second;r.v1=row.first+1;next.emplace(key,r);active.erase(it);}
                else next.emplace(key,Rect{lo,hi,row.first,row.first+1});
            }
            for(auto&item:active)emit(item.second);active=std::move(next);
        }
        for(auto&item:active)emit(item.second);
    }
    return out;
}
std::vector<Quad>dirtyBoundary(const Grid&g,const std::set<Cell>&dirty){
    std::vector<Quad>out;const Cell delta[6]={{-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1}};
    const int corners[6][4][3]={{{0,0,0},{0,0,1},{0,1,1},{0,1,0}},{{1,0,0},{1,1,0},{1,1,1},{1,0,1}},{{0,0,0},{1,0,0},{1,0,1},{0,0,1}},{{0,1,0},{0,1,1},{1,1,1},{1,1,0}},{{0,0,0},{0,1,0},{1,1,0},{1,0,0}},{{0,0,1},{1,0,1},{1,1,1},{0,1,1}}};
    for(auto t:dirty){if(t.x<0||t.y<0||t.z<0||g.tiles.find(t)==g.tiles.end())continue;
        for(int x=0;x<8;++x)for(int y=0;y<8;++y)for(int z=0;z<8;++z){Cell c{t.x*8+x,t.y*8+y,t.z*8+z};if(!g.removed(c))continue;
            for(int a=0;a<6;++a){Cell n{c.x+delta[a].x,c.y+delta[a].y,c.z+delta[a].z};if(g.removed(n)||(g.ownership&&!g.solidBase(n)))continue;Quad q;q.removedCell=c;q.material=g.renderMaterial(n);for(int k=0;k<4;++k)q.p[k]=g.origin+Vec{double(c.x+corners[a][k][0]),double(c.y+corners[a][k][1]),double(c.z+corners[a][k][2])}*g.pitch;out.push_back(q);}}}
    return out;
}
struct V {Vec p;std::array<double,3>b{};unsigned material{1};};
using Poly=std::vector<V>;
double area(const Poly&p){double sum=0;for(std::size_t i=1;i+1<p.size();++i)sum+=length(cross(p[i].p-p[0].p,p[i+1].p-p[0].p))*.5;return sum;}
// clip() into a caller-owned buffer (cleared first); identical arithmetic.
void clipInto(const Poly&p,int axis,double bound,double sign,Poly&out){
    // `last` caches quant(out.back().p): the same dedup decisions with one
    // quantization per candidate vertex instead of two.
    out.clear();if(p.empty())return;Quant last{};
    auto append=[&](const V&v){const Quant q=quant(v.p);if(out.empty()||last!=q){out.push_back(v);last=q;}};
    for(std::size_t i=0;i<p.size();++i){const auto&a=p[i];const auto&b=p[(i+1)%p.size()];double da=(a.p[axis]-bound)*sign,db=(b.p[axis]-bound)*sign;bool ia=da>=-1e-11,ib=db>=-1e-11;
        // Keep exact double intersections while clipping successive cells.
        // Repeated intermediate quantization creates nanometre sliver polygons
        // at overlaps. Canonicalize once, at the shared-provenance boundary.
        if(ia)append(a);if(ia!=ib){double t=da/(da-db);V v;v.p=a.p+(b.p-a.p)*t;v.p[axis]=bound;for(int k=0;k<3;++k)v.b[k]=a.b[k]+(b.b[k]-a.b[k])*t;append(v);}}
    if(out.size()>1&&quant(out.front().p)==quant(out.back().p))out.pop_back();if(out.size()<3||area(out)<1e-16)out.clear();
}
Poly clip(const Poly&p,int axis,double bound,double sign){Poly out;out.reserve(p.size()+2);clipInto(p,axis,bound,sign,out);return out;}
bool overlaps(const Poly&p,Vec lo,Vec hi){for(int a=0;a<3;++a){double mn=1e9,mx=-1e9;for(auto&v:p){mn=std::min(mn,v.p[a]);mx=std::max(mx,v.p[a]);}if(mx<lo[a]-1e-11||mn>hi[a]+1e-11)return false;}return true;}
// Same clip() calls as subtract(p,lo,hi,true) on the same inputs, ordered so
// the six "inside" clips (which ARE subtract's emptiness test) run once:
// returns false exactly when subtract would return {p} unchanged; otherwise
// appends subtract's parts, in its order, to `parts`.
bool subtractParts(const Poly&p,Vec lo,Vec hi,std::vector<Poly>&parts){
    // Reused per-thread buffers: no allocation unless a part is emitted.
    thread_local Poly chain[6],outside;const Poly*in=&p;
    for(int a=0;a<3;++a){clipInto(*in,a,lo[a],1,chain[2*a]);if(chain[2*a].empty())return false;clipInto(chain[2*a],a,hi[a],-1,chain[2*a+1]);if(chain[2*a+1].empty())return false;in=&chain[2*a+1];}
    for(int a=0;a<3;++a){const Poly&before=a?chain[2*a-1]:p;
        clipInto(before,a,lo[a],-1,outside);if(!outside.empty())parts.emplace_back(outside.begin(),outside.end());
        clipInto(chain[2*a],a,hi[a],1,outside);if(!outside.empty())parts.emplace_back(outside.begin(),outside.end());}
    return true;
}
std::vector<Poly>subtract(const Poly&p,Vec lo,Vec hi,bool overlapKnown=false){
    if(!overlapKnown&&!overlaps(p,lo,hi))return {p};Poly test=p;for(int a=0;a<3&&!test.empty();++a){test=clip(test,a,lo[a],1);test=clip(test,a,hi[a],-1);}if(test.empty())return {p};
    std::vector<Poly>out;Poly inside=p;for(int a=0;a<3&&!inside.empty();++a){auto outside=clip(inside,a,lo[a],-1);if(!outside.empty())out.push_back(std::move(outside));inside=clip(inside,a,lo[a],1);outside=clip(inside,a,hi[a],1);if(!outside.empty())out.push_back(std::move(outside));inside=clip(inside,a,hi[a],-1);}return out;
}
struct Fragment {std::uint32_t sourceFace;Poly polygon;};
struct ClipResult {std::vector<Fragment>polygons;std::size_t unchanged{},candidateCells{};double originalArea{},remainingArea{};};
using Edge=std::array<Quant,2>;
Edge edgeKey(Vec a,Vec b){Edge key{quant(a),quant(b)};if(key[1]<key[0])std::swap(key[0],key[1]);return key;}
std::array<double,3>faceBary(const Source&s,std::uint32_t face,Vec p){auto&id=s.f[face];Vec a=s.v[id[0]].p,b=s.v[id[1]].p-a,c=s.v[id[2]].p-a,d=p-a;double bb=dot(b,b),bc=dot(b,c),cc=dot(c,c),db=dot(d,b),dc=dot(d,c),denom=bb*cc-bc*bc;double y=(cc*db-bc*dc)/denom,z=(bb*dc-bc*db)/denom;std::array<double,3> bary{1-y-z,y,z};
    // Shared cut points are snapped by nanometres. Recomputing barycentrics on
    // the adjacent source face can put that point fractionally past its edge.
    // Project only this bounded rounding error back onto the triangle; never
    // allow a negative skin weight or mask a genuinely unrelated source face.
    if(*std::min_element(bary.begin(),bary.end())<0){
        std::array<double,3> corrected{};double best=1e30;
        for(int k=0;k<3;++k){int next=(k+1)%3;Vec start=s.v[id[k]].p,edge=s.v[id[next]].p-start;
            double t=std::clamp(dot(p-start,edge)/dot(edge,edge),0.,1.);double distance=length(start+edge*t-p);
            if(distance<best){best=distance;corrected={};corrected[k]=1-t;corrected[next]=t;}}
        if(best>3e-8)throw std::runtime_error("source_barycentric_outside_snap_tolerance");
        bary=corrected;
    }return bary;}
void conform(const Source&s,ClipResult&result){
    std::vector<std::map<Quant,V>>points(s.f.size());std::map<Edge,std::map<Quant,Vec>>sharedEdges;std::vector<bool>affected(s.f.size());
    auto originalCorner=[](const V&v){return (v.b[0]==1&&v.b[1]==0&&v.b[2]==0)||(v.b[0]==0&&v.b[1]==1&&v.b[2]==0)||(v.b[0]==0&&v.b[1]==0&&v.b[2]==1);};
    for(auto&f:result.polygons)for(auto&v:f.polygon)if(!originalCorner(v))v.p=snap(v.p);
    for(auto&f:result.polygons)for(auto&v:f.polygon)if(!originalCorner(v)){affected[f.sourceFace]=true;for(int a=0;a<3;++a)if(std::abs(v.b[a])<1e-7){auto&id=s.f[f.sourceFace];sharedEdges[edgeKey(s.v[id[(a+1)%3]].p,s.v[id[(a+2)%3]].p)].emplace(quant(v.p),v.p);}}
    for(auto&f:result.polygons)if(affected[f.sourceFace])for(auto&v:f.polygon)points[f.sourceFace].emplace(quant(v.p),v);
    for(std::uint32_t face=0;face<s.f.size();++face){auto&id=s.f[face];for(int a=0;a<3;++a){auto it=sharedEdges.find(edgeKey(s.v[id[a]].p,s.v[id[(a+1)%3]].p));if(it!=sharedEdges.end())for(auto&item:it->second)points[face].emplace(item.first,V{item.second,faceBary(s,face,item.second)});}}
    // Per face: its points in map order plus an x-sorted index. A point that
    // passes the exact on-edge test lies within the edge's box +-3e-8, so only
    // points whose x is inside the edge's x range +-1e-7 are tested, in the
    // original map order; the unstable t sort then sees the same sequence.
    struct FaceIndex{bool built=false;std::vector<const V*>ordered;std::vector<std::pair<double,std::uint32_t>>byX;};
    std::vector<FaceIndex>index(s.f.size());std::vector<std::uint32_t>candidates;
    for(auto&f:result.polygons){if(points[f.sourceFace].empty())continue;auto&fi=index[f.sourceFace];
        if(!fi.built){fi.built=true;for(auto&item:points[f.sourceFace]){fi.byX.push_back({item.second.p.x,std::uint32_t(fi.ordered.size())});fi.ordered.push_back(&item.second);}
            std::sort(fi.byX.begin(),fi.byX.end());}
        Poly output;for(std::size_t i=0;i<f.polygon.size();++i){auto a=f.polygon[i],b=f.polygon[(i+1)%f.polygon.size()];output.push_back(a);Vec edge=b.p-a.p;double dd=dot(edge,edge);if(dd<1e-20)continue;std::vector<std::pair<double,V>>inside;
            const double xlo=std::min(a.p.x,b.p.x)-1e-7,xhi=std::max(a.p.x,b.p.x)+1e-7;candidates.clear();
            for(auto it=std::lower_bound(fi.byX.begin(),fi.byX.end(),std::make_pair(xlo,std::uint32_t(0)));it!=fi.byX.end()&&it->first<=xhi;++it)candidates.push_back(it->second);
            if(candidates.empty())continue;std::sort(candidates.begin(),candidates.end());const Quant qa=quant(a.p),qb=quant(b.p);
            for(auto c:candidates){const auto&v=*fi.ordered[c];const Quant qv=quant(v.p);if(qv==qa||qv==qb)continue;double t=dot(v.p-a.p,edge)/dd;if(t>1e-8&&t<1-1e-8&&length(v.p-(a.p+edge*t))<3e-8)inside.push_back({t,v});}
            std::sort(inside.begin(),inside.end(),[](auto&a1,auto&b1){return a1.first<b1.first;});for(auto&item:inside)output.push_back(item.second);}
        f.polygon=std::move(output);}
}
// Former implementation, unchanged. Kept as the exact reference for owned
// (piece) grids, whose Grid::removed also reports foreign cells, and for tests.
ClipResult exteriorReference(const Source&s,const Grid&g){
    StageProfile stages("exterior");ClipResult result;for(std::uint32_t face=0;face<s.f.size();++face){const auto&id=s.f[face];Poly p;for(int k=0;k<3;++k){V v;v.p=s.v[id[k]].p;v.b[k]=1;p.push_back(v);}result.originalArea+=area(p);
        Cell lo,hi;bool range=true;for(int a=0;a<3;++a){double mn=std::min({p[0].p[a],p[1].p[a],p[2].p[a]}),mx=std::max({p[0].p[a],p[1].p[a],p[2].p[a]});int l=int(std::floor((mn-g.origin[a])/g.pitch)),h=int(std::floor((mx-g.origin[a])/g.pitch));if(h<0||l>=g.size[a])range=false;(&lo.x)[a]=std::max(0,l);(&hi.x)[a]=std::min(g.size[a]-1,h);}
        std::vector<Poly>pieces{p};if(range)for(int x=lo.x;x<=hi.x&&!pieces.empty();++x)for(int y=lo.y;y<=hi.y&&!pieces.empty();++y)for(int z=lo.z;z<=hi.z&&!pieces.empty();++z){Cell c{x,y,z};if(!g.removed(c))continue;const int first=z;while(z<hi.z&&g.removed({x,y,z+1}))++z;result.candidateCells+=z-first+1;Vec a=g.origin+Vec{double(x),double(y),double(first)}*g.pitch,b=a+Vec{g.pitch,g.pitch,(z-first+1)*g.pitch};std::vector<Poly>next;
            next.reserve(pieces.size()+4);for(auto&piece:pieces){if(!overlaps(piece,a,b)){next.push_back(std::move(piece));continue;}auto cut=subtract(piece,a,b,true);for(auto&part:cut)next.push_back(std::move(part));}pieces=std::move(next);}
        bool unchanged=pieces.size()==1&&pieces[0].size()==3;for(int k=0;k<3&&unchanged;++k)unchanged=pieces[0][k].p.x==p[k].p.x&&pieces[0][k].p.y==p[k].p.y&&pieces[0][k].p.z==p[k].p.z;if(unchanged)++result.unchanged;
        for(auto&piece:pieces){result.remainingArea+=area(piece);result.polygons.push_back({face,std::move(piece)});}}
    stages.mark("subtract_cells");conform(s,result);stages.mark("conform_faces");return result;
}
// Removed tiles grouped by lattice (x/8,y/8) column, ascending z tile.
struct TileColumns {
    int nx{},ny{};std::vector<std::uint32_t>start;std::vector<std::pair<int,const Tile*>>entries;
    TileColumns()=default;
    explicit TileColumns(const Grid&g){build(g.size,g.tiles.begin(),g.tiles.end(),[](const auto&item){return &item.second;});}
    template<class It,class Ptr>void build(Cell size,It first,It last,Ptr ptr){
        nx=(size.x+7)/8;ny=(size.y+7)/8;start.assign(std::size_t(nx)*std::size_t(ny)+1,0);
        for(It it=first;it!=last;++it)if(inColumns(it->first))++start[column(it->first.x,it->first.y)+1];
        for(std::size_t i=1;i<start.size();++i)start[i]+=start[i-1];
        entries.resize(start.back());std::vector<std::uint32_t>fill(start.begin(),start.end()-1);
        for(It it=first;it!=last;++it)if(inColumns(it->first))entries[fill[column(it->first.x,it->first.y)]++]={it->first.z,ptr(*it)};
        for(std::size_t c=0;c+1<start.size();++c)std::sort(entries.begin()+start[c],entries.begin()+start[c+1],[](const auto&a,const auto&b){return a.first<b.first;});
    }
    bool inColumns(Cell k)const{return k.x>=0&&k.y>=0&&k.x<nx&&k.y<ny;}
    std::size_t column(int tx,int ty)const{return std::size_t(tx)*std::size_t(ny)+std::size_t(ty);}
    // True when any stored tile mask has a set bit inside the closed cell box.
    bool anyBit(Cell lo,Cell hi)const{
        const int tz0=lo.z/8,tz1=hi.z/8;
        for(int tx=lo.x/8;tx<=hi.x/8;++tx)for(int ty=lo.y/8;ty<=hi.y/8;++ty){
            if(!inColumns({tx,ty,0}))continue;const auto c=column(tx,ty);
            auto e=entries.begin()+start[c],last=entries.begin()+start[c+1];if(e==last)continue;
            e=std::lower_bound(e,last,tz0,[](const auto&a,int z){return a.first<z;});
            for(;e!=last&&e->first<=tz1;++e){
                std::uint64_t zbits=0;for(int b=0;b<8;++b){const int z=e->first*8+b;if(z>=lo.z&&z<=hi.z)zbits|=std::uint64_t(1)<<b;}
                std::uint64_t rows=0;for(int r=0;r<8;++r){const int y=ty*8+r;if(y>=lo.y&&y<=hi.y)rows|=zbits<<(8*r);}
                for(int w=0;w<8;++w){const int x=tx*8+w;if(x<lo.x||x>hi.x)continue;if(e->second->removed[w]&rows)return true;}
            }
        }
        return false;
    }
};
// Conservative filter geometry only (never emitted): polygon in -> part with
// lo <= p[axis] <= hi, Sutherland-Hodgman against the two planes.
inline void clipSlab(const std::vector<Vec>&in,int axis,double lo,double hi,std::vector<Vec>&out,std::vector<Vec>&tmp){
    auto half=[axis](const std::vector<Vec>&src,std::vector<Vec>&dst,double bound,double sign){dst.clear();
        for(std::size_t i=0;i<src.size();++i){const Vec&a=src[i];const Vec&b=src[(i+1)%src.size()];
            const double da=(a[axis]-bound)*sign,db=(b[axis]-bound)*sign;
            if(da>=0)dst.push_back(a);
            if((da>=0)!=(db>=0)){const double t=da/(da-db);Vec v=a+(b-a)*t;v[axis]=bound;dst.push_back(v);}}};
    half(in,tmp,lo,1);half(tmp,out,hi,-1);
}
// Vertex bounds of a polygon computed exactly as overlaps() does, so
// PolyBounds::of(p).overlaps(lo,hi)==overlaps(p,lo,hi) bit for bit.
struct PolyBounds {
    double mn[3],mx[3];
    static PolyBounds of(const Poly&p){PolyBounds b;for(int a=0;a<3;++a){double lo=1e9,hi=-1e9;for(auto&v:p){lo=std::min(lo,v.p[a]);hi=std::max(hi,v.p[a]);}b.mn[a]=lo;b.mx[a]=hi;}return b;}
    bool overlaps(Vec lo,Vec hi)const{for(int a=0;a<3;++a)if(mx[a]<lo[a]-1e-11||mn[a]>hi[a]+1e-11)return false;return true;}
};
// Cross-hit memo of the pre-conform exterior pieces of each source face.
// A face's pieces are a pure function of the removed bits inside the face's
// AABB cell range (and of the fixed lattice). The memo records the grid
// content it was computed against (`basis`); reuse is decided by comparing
// that content with the current grid, never by trusting commit history, so a
// memo from a rejected, canceled or stale candidate is still exact (at worst
// useless). Owned (piece) grids never use it.
struct FaceClip {std::vector<Poly>pieces;std::size_t candidateCells{};};
struct ExteriorMemo {
    std::shared_ptr<const Grid>basis; // Set by the producer once the grid is frozen.
    Cell size;Vec origin;double pitch{};
    std::unordered_map<std::uint32_t,FaceClip>faces; // key: caller face id (original source face)
};
struct ExteriorStats {std::size_t reused{},computed{},occupiedFaces{};};
// Tiles whose removed words differ between two grids, as XOR masks.
struct TileDelta {
    std::vector<std::pair<Cell,Tile>>tiles;TileColumns columns;
    TileDelta(const Grid&before,const Grid&after){
        for(const auto&item:after.tiles){auto it=before.tiles.find(item.first);Tile x=item.second;bool any=false;
            for(int w=0;w<8;++w){if(it!=before.tiles.end())x.removed[w]^=it->second.removed[w];any=any||x.removed[w];}
            if(any)tiles.push_back({item.first,x});}
        for(const auto&item:before.tiles)if(!after.tiles.count(item.first)){bool any=false;for(auto w:item.second.removed)any=any||w;if(any)tiles.push_back(item);}
        columns.build(after.size,tiles.begin(),tiles.end(),[](const auto&item){return &item.second;});
    }
};
// Exact replacement of exteriorReference for unowned grids.
//
// The reference visits every cell of each source triangle's AABB (millions of
// hash lookups for the large faces of low-poly bodies, on every hit) and
// subtracts every maximal removed z-run box in (x,y,z) order. This version:
//  * rejects faces whose AABB tile box holds no removed bit;
//  * reuses memoized pieces of faces whose AABB saw no changed bit;
//  * otherwise produces the SAME ordered sequence of runs from the removed
//    tile bits only, so candidateCells, the early exit when no piece remains
//    and the piece order are unchanged;
//  * does not subtract a run box whose eight corners all lie more than
//    1e-6*|n| (1 micrometre) on one side of the triangle's plane. For such a
//    box every piece (a subset of the triangle, on its plane up to ~1e-15 m)
//    is separated from the box by far more than the clip tolerance (1e-11 m),
//    so subtract() would return the piece unchanged. Degenerate triangles
//    subtract every run, as before.
// `keys[i]` identifies face i for the memo (identity when null).
// Declared here, defined in join_cavity.hpp (same translation unit).
template<class F>void parallelChunks(std::size_t n,std::size_t grain,F&&body);
inline std::size_t parallelWidth();
ClipResult exterior(const Source&s,const Grid&g,const ExteriorMemo*reuse,const std::vector<std::uint32_t>*keys,ExteriorMemo*produce,ExteriorStats*stats=nullptr){
    if(g.ownership)return exteriorReference(s,g);
    StageProfile stages("exterior");ClipResult result;const TileColumns columns(g);
    if(reuse&&(!reuse->basis||reuse->basis->ownership||!(reuse->size==g.size)||reuse->pitch!=g.pitch||reuse->origin.x!=g.origin.x||reuse->origin.y!=g.origin.y||reuse->origin.z!=g.origin.z))reuse=nullptr;
    std::unique_ptr<TileDelta>delta;if(reuse)delta=std::make_unique<TileDelta>(*reuse->basis,g);
    if(produce){produce->faces.clear();produce->basis.reset();produce->size=g.size;produce->origin=g.origin;produce->pitch=g.pitch;}
    ExteriorStats local;
    auto triangleOf=[&](std::uint32_t face){const auto&id=s.f[face];Poly p;for(int k=0;k<3;++k){V v;v.p=s.v[id[k]].p;v.b[k]=1;p.push_back(v);}return p;};
    auto rangeOf=[&](const Poly&p,Cell&lo,Cell&hi){bool range=true;for(int a=0;a<3;++a){double mn=std::min({p[0].p[a],p[1].p[a],p[2].p[a]}),mx=std::max({p[0].p[a],p[1].p[a],p[2].p[a]});int l=int(std::floor((mn-g.origin[a])/g.pitch)),h=int(std::floor((mx-g.origin[a])/g.pitch));if(h<0||l>=g.size[a])range=false;(&lo.x)[a]=std::max(0,l);(&hi.x)[a]=std::min(g.size[a]-1,h);}return range;};
    // Pass 1 (serial, cheap): occupancy and memo decisions.
    struct FaceOut{std::vector<Poly>pieces;std::size_t cells{};bool occupied{};const FaceClip*memo{};};
    std::vector<FaceOut>outs(s.f.size());std::vector<std::uint32_t>work;
    for(std::uint32_t face=0;face<s.f.size();++face){const Poly p=triangleOf(face);Cell lo,hi;const bool range=rangeOf(p,lo,hi);auto&o=outs[face];
        o.occupied=range&&columns.anyBit(lo,hi);if(!o.occupied)continue;++local.occupiedFaces;
        if(reuse){auto it=reuse->faces.find(keys?(*keys)[face]:face);if(it!=reuse->faces.end()&&!delta->columns.anyBit(lo,hi))o.memo=&it->second;}
        if(o.memo)++local.reused;else{++local.computed;work.push_back(face);}}
    // Pass 2: clip every face that needs it. Faces are independent; each
    // writes only its own slot, so up to four threads pulling faces from an
    // atomic counter fill the same slots as a serial loop.
    struct Scratch{std::vector<std::uint64_t>bits;std::vector<Vec>prismX,prismXY,clipScratch;std::vector<Poly>next;std::vector<PolyBounds>nextBounds;};
    auto clipFace=[&](std::uint32_t face,Scratch&sc){auto&o=outs[face];const Poly p=triangleOf(face);Cell lo,hi;rangeOf(p,lo,hi);
        auto&pieces=o.pieces;auto&cells=o.cells;auto&bits=sc.bits;auto&prismX=sc.prismX;auto&prismXY=sc.prismXY;auto&clipScratch=sc.clipScratch;auto&next=sc.next;auto&nextBounds=sc.nextBounds;
        pieces.push_back(p);std::vector<PolyBounds>bounds{PolyBounds::of(p)};
        const Vec normal=cross(p[1].p-p[0].p,p[2].p-p[0].p);const double norm=length(normal);
        const bool prune=std::isfinite(norm)&&norm>1e-30;const double margin=norm*1e-6,offset=dot(normal,p[0].p);
        const int zspan=hi.z-lo.z+1,tz0=lo.z/8,tz1=hi.z/8;bits.assign(std::size_t(zspan+63)/64+1,0);
        bool done=false;
        // Column-prism filter (exact no-op skip): pieces lie inside the
        // triangle, so a run box that misses the triangle's part inside
        // its column prism (by more than 1 um) cannot change any piece.
        const double m=1e-6;const std::vector<Vec>triangle{p[0].p,p[1].p,p[2].p};
        for(int x=lo.x;x<=hi.x&&!done;++x){const int tx=x/8;
            int ylo=lo.y,yhi=hi.y;
            if(prune){clipSlab(triangle,0,g.origin.x+x*g.pitch-m,g.origin.x+(x+1)*g.pitch+m,prismX,clipScratch);if(prismX.empty())continue;
                double mn=1e300,mx=-1e300;for(auto&v:prismX){mn=std::min(mn,v.y);mx=std::max(mx,v.y);}
                ylo=std::max(lo.y,int(std::floor((mn-m-g.origin.y)/g.pitch)));yhi=std::min(hi.y,int(std::floor((mx+m-g.origin.y)/g.pitch)));}
            for(int y=ylo;y<=yhi&&!done;++y){const int ty=y/8;
                const auto c=columns.column(tx,ty);auto e=columns.entries.begin()+columns.start[c],last=columns.entries.begin()+columns.start[c+1];
                if(e==last){y=std::min(yhi,ty*8+7);continue;}
                int window0=0,window1=zspan-1;
                if(prune){clipSlab(prismX,1,g.origin.y+y*g.pitch-m,g.origin.y+(y+1)*g.pitch+m,prismXY,clipScratch);if(prismXY.empty())continue;
                    double mn=1e300,mx=-1e300;for(auto&v:prismXY){mn=std::min(mn,v.z);mx=std::max(mx,v.z);}
                    window0=std::max(0,int(std::floor((mn-m-g.origin.z)/g.pitch))-lo.z);window1=std::min(zspan-1,int(std::floor((mx+m-g.origin.z)/g.pitch))-lo.z);
                    if(window0>window1)continue;}
                e=std::lower_bound(e,last,tz0,[](const auto&a,int z){return a.first<z;});
                bool any=false;std::fill(bits.begin(),bits.end(),0);
                for(;e!=last&&e->first<=tz1;++e){unsigned row=unsigned((e->second->removed[x&7]>>((y&7)*8))&255u);
                    for(;row;row&=row-1){const int z=e->first*8+int(lowestBit(row));if(z<lo.z||z>hi.z)continue;const int at=z-lo.z;bits[std::size_t(at)>>6]|=std::uint64_t(1)<<(at&63);any=true;}}
                if(!any)continue;
                for(int at=0;at<zspan&&!done;){
                    std::size_t w=std::size_t(at)>>6;std::uint64_t word=bits[w]&(~std::uint64_t(0)<<(at&63));
                    while(!word&&(w+1)*64<std::size_t(zspan)){++w;word=bits[w];}
                    if(!word)break;const int first=int(w*64+lowestBit(word));if(first>=zspan)break;
                    int end;{std::size_t v=std::size_t(first)>>6;std::uint64_t inv=~bits[v]&(~std::uint64_t(0)<<(first&63));
                        while(!inv){++v;inv=~bits[v];}end=std::min(zspan,int(v*64+lowestBit(inv)));}
                    at=end;const int z0=lo.z+first,count=end-first;
                    if(end-1<window0)continue;if(first>window1)break;
                    cells+=std::size_t(count);
                    Vec a=g.origin+Vec{double(x),double(y),double(z0)}*g.pitch,b=a+Vec{g.pitch,g.pitch,count*g.pitch};
                    if(prune){double base=dot(normal,a)-offset,low=base,high=base;
                        for(int k=0;k<3;++k){const double d=normal[k]*(b[k]-a[k]);if(d<0)low+=d;else high+=d;}
                        if(low>margin||high< -margin)continue;}
                    // Same overlaps() decision from each piece's cached vertex bounds.
                    {bool anyOverlap=false;for(const auto&bb:bounds)if(bb.overlaps(a,b)){anyOverlap=true;break;}if(!anyOverlap)continue;}
                    next.clear();nextBounds.clear();
                    for(std::size_t k=0;k<pieces.size();++k){
                        if(bounds[k].overlaps(a,b)){const std::size_t mark=next.size();
                            if(subtractParts(pieces[k],a,b,next)){for(std::size_t n=mark;n<next.size();++n)nextBounds.push_back(PolyBounds::of(next[n]));continue;}}
                        next.push_back(std::move(pieces[k]));nextBounds.push_back(bounds[k]);}
                    std::swap(pieces,next);std::swap(bounds,nextBounds);if(pieces.empty())done=true;
                }
            }
        }
    };
    if(work.size()>=4){std::atomic<std::size_t>cursor{0};
        parallelChunks(parallelWidth(),1,[&](std::size_t,std::size_t,std::size_t){Scratch sc;for(std::size_t i;(i=cursor.fetch_add(1))<work.size();)clipFace(work[i],sc);});}
    else{Scratch sc;for(auto face:work)clipFace(face,sc);}
    // Pass 3 (serial, face order): identical accumulation order to the former loop.
    for(std::uint32_t face=0;face<s.f.size();++face){const Poly p=triangleOf(face);result.originalArea+=area(p);auto&o=outs[face];
        if(!o.occupied)o.pieces.push_back(p);else if(o.memo){o.pieces=o.memo->pieces;o.cells=o.memo->candidateCells;}
        if(produce&&o.occupied){auto&m=produce->faces[keys?(*keys)[face]:face];m.pieces=o.pieces;m.candidateCells=o.cells;}
        result.candidateCells+=o.cells;auto&pieces=o.pieces;
        bool unchanged=pieces.size()==1&&pieces[0].size()==3;for(int k=0;k<3&&unchanged;++k)unchanged=pieces[0][k].p.x==p[k].p.x&&pieces[0][k].p.y==p[k].p.y&&pieces[0][k].p.z==p[k].p.z;if(unchanged)++result.unchanged;
        for(auto&piece:pieces){result.remainingArea+=area(piece);result.polygons.push_back({face,std::move(piece)});}}
    if(stats)*stats=local;
    stages.mark("subtract_cells");conform(s,result);stages.mark("conform_faces");return result;
}
ClipResult exterior(const Source&s,const Grid&g){return exterior(s,g,nullptr,nullptr,nullptr);}

std::array<double,64>weights(const Source&s,std::uint32_t face,const V&v){std::array<double,64>w{};for(int c=0;c<3;++c){auto&src=s.v[s.f[face][c]];for(int k=0;k<4;++k)w[src.ids[k]]+=v.b[c]*src.w[k];}return w;}
struct Provenance {std::size_t repeatedPositions{};double maxWeightConflict{},maxBarySumError{},maxReconstructionError{};};
Provenance inspectProvenance(const Source&s,const ClipResult&result){
    Provenance out;std::map<Quant,std::array<double,64>>shared;
    for(auto&fragment:result.polygons)for(auto&v:fragment.polygon){auto w=weights(s,fragment.sourceFace,v);auto[it,fresh]=shared.emplace(quant(v.p),w);if(!fresh){++out.repeatedPositions;for(std::uint32_t k=0;k<s.bones;++k)out.maxWeightConflict=std::max(out.maxWeightConflict,std::abs(w[k]-it->second[k]));}
        out.maxBarySumError=std::max(out.maxBarySumError,std::abs(v.b[0]+v.b[1]+v.b[2]-1));Vec reconstructed;for(int k=0;k<3;++k)reconstructed=reconstructed+s.v[s.f[fragment.sourceFace][k]].p*v.b[k];out.maxReconstructionError=std::max(out.maxReconstructionError,length(reconstructed-v.p));}
    return out;
}
struct EdgeProof {std::size_t originalBoundary{},cutBoundary{},unexpectedOpen{},nonmanifold{};};
EdgeProof inspectEdges(const Source&s,const Grid&g,const ClipResult&r){
    std::map<Edge,unsigned>original,output;std::map<Quant,Vec>coordinates;
    for(auto&f:s.f)for(int i=0;i<3;++i)++original[edgeKey(s.v[f[i]].p,s.v[f[(i+1)%3]].p)];
    for(auto&f:r.polygons)for(std::size_t i=0;i<f.polygon.size();++i){auto a=f.polygon[i].p,b=f.polygon[(i+1)%f.polygon.size()].p;auto key=edgeKey(a,b);if(key[0]==key[1])continue;++output[key];coordinates[quant(a)]=a;coordinates[quant(b)]=b;}
    EdgeProof proof;for(auto&item:output){if(item.second>2){++proof.nonmanifold;auto a=coordinates[item.first[0]],b=coordinates[item.first[1]];std::cerr<<"NONMANIFOLD "<<item.second<<" length "<<length(b-a)<<" at "<<std::setprecision(12)<<a.x<<","<<a.y<<","<<a.z<<" -> "<<b.x<<","<<b.y<<","<<b.z<<'\n';continue;}if(item.second==2)continue;auto key=item.first;Vec a=coordinates[key[0]],b=coordinates[key[1]],mid=(a+b)*.5;bool source=false;
        if(original.count(key)&&original[key]==1)source=true;
        if(source){++proof.originalBoundary;continue;}bool cut=false;for(int axis=0;axis<3;++axis){double plane=std::round((mid[axis]-g.origin[axis])/g.pitch)*g.pitch+g.origin[axis];if(std::abs(a[axis]-plane)>2e-8||std::abs(b[axis]-plane)>2e-8)continue;Vec below=mid,above=mid;below[axis]-=1e-7;above[axis]+=1e-7;auto cell=[&](Vec p){return Cell{int(std::floor((p.x-g.origin.x)/g.pitch)),int(std::floor((p.y-g.origin.y)/g.pitch)),int(std::floor((p.z-g.origin.z)/g.pitch))};};if(g.removed(cell(below))!=g.removed(cell(above)))cut=true;}
        if(cut)++proof.cutBoundary;else++proof.unexpectedOpen;}
    return proof;
}
#include "join_cavity.hpp"
void tests(){
    Poly p={{{0,0,0},{1,0,0}},{{1,0,0},{0,1,0}},{{0,1,0},{0,0,1}}};auto cut=subtract(p,{.25,-1,-1},{.75,2,1});double sum=0;for(auto&q:cut)sum+=area(q);if(std::abs(sum-.25)>1e-9)throw std::runtime_error("Analytic triangle strip area failed");
    auto unchanged=subtract(p,{2,2,2},{3,3,3});if(unchanged.size()!=1||area(unchanged[0])!=.5)throw std::runtime_error("Outside-cell identity failed");
    auto gone=subtract(p,{-1,-1,-1},{2,2,2});if(!gone.empty())throw std::runtime_error("Full removal failed");
    // Two adjacent triangles with genuinely mixed skin weights. Independent
    // clipping must agree at their shared diagonal under either winding.
    Source joint;joint.bones=2;for(Vec v:std::array<Vec,4>{{{0,0,0},{1,0,0},{1,1,0},{0,1,0}}}){SourceVertex a;a.p=v;a.ids={0,1,0,0};a.w={float(1-v.x),float(v.x),0,0};joint.v.push_back(a);}joint.f={{{0,1,2}},{{0,2,3}}};ClipResult pieces;
    for(std::uint32_t f=0;f<2;++f){Poly q;for(int k=0;k<3;++k){V v;v.p=joint.v[joint.f[f][k]].p;v.b[k]=1;q.push_back(v);}for(auto&r:subtract(q,{.23,.21,-1},{.67,.73,1}))pieces.polygons.push_back({f,std::move(r)});}
    conform(joint,pieces);auto proof=inspectProvenance(joint,pieces);if(proof.maxWeightConflict>1e-7||proof.maxReconstructionError>2e-8)throw std::runtime_error("Mixed-joint shared provenance failed");
    std::mt19937 random(731991);std::uniform_real_distribution<double>unit(0.,1.);
    for(int sample=0;sample<96;++sample){
        Vec u{unit(random)-.5,unit(random)-.5,unit(random)-.5};u=u*(1/length(u));Vec hint{unit(random)-.5,unit(random)-.5,unit(random)-.5};Vec v=cross(u,hint);v=v*(1/length(v));Vec center{.01,1.71,.12};Vec normal=cross(u,v);u=u*.031;v=v*.022;
        Source tilted;tilted.bones=2;const Vec corners[4]={center-u-v,center+u-v,center+u+v,center-u+v};
        for(int i=0;i<4;++i){SourceVertex a;a.p=corners[i];a.ids={0,1,0,0};float w=(i==1||i==2)?1.f:0.f;a.w={1-w,w,0,0};tilted.v.push_back(a);}tilted.f=joint.f;
        Vec cutter=center+u*(unit(random)*.8-.4)+v*(unit(random)*.8-.4),lo=cutter-Vec{.012,.012,.012},hi=cutter+Vec{.012,.012,.012};ClipResult fragments;
        for(std::uint32_t face=0;face<2;++face){Poly tri;for(int k=0;k<3;++k){V a;a.p=tilted.v[tilted.f[face][k]].p;a.b[k]=1;tri.push_back(a);}auto parts=subtract(tri,lo,hi);
            // Independent barycentric point coverage against the analytic box;
            // verifies a partition, not just the clipper's own area arithmetic.
            for(int pick=0;pick<128;++pick){double a=unit(random),b=unit(random);if(a+b>1){a=1-a;b=1-b;}Vec point=tri[0].p*(1-a-b)+tri[1].p*a+tri[2].p*b;bool removed=true,near=false;for(int axis=0;axis<3;++axis){removed=removed&&(point[axis]>lo[axis]&&point[axis]<hi[axis]);near=near||std::abs(point[axis]-lo[axis])<1e-7||std::abs(point[axis]-hi[axis])<1e-7;}if(near)continue;
                unsigned covers=0;for(auto&poly:parts){bool contains=true;for(std::size_t k=0;k<poly.size();++k)contains=contains&&(dot(cross(poly[(k+1)%poly.size()].p-poly[k].p,point-poly[k].p),normal)>=-1e-12);if(contains)++covers;}if(covers!=(removed?0u:1u))throw std::runtime_error("Independent randomized point coverage failed");}
            for(auto&poly:parts)fragments.polygons.push_back({face,std::move(poly)});}
        conform(tilted,fragments);auto check=inspectProvenance(tilted,fragments);if(check.maxWeightConflict>2e-6||check.maxBarySumError>1e-10||check.maxReconstructionError>3e-8)throw std::runtime_error("Angled mixed-joint provenance failed");
    }
}
double ms(Clock::time_point start){return std::chrono::duration<double,std::milli>(Clock::now()-start).count();}
int main(int argc,char**argv){try{
    if(argc<2)throw std::runtime_error("Pass input folder");tests();std::string folder=argv[1];Source source=loadSource(folder+"/head_geometry.bin"),envelope=loadSource(folder+"/body_envelope.bin");BodyBVH sourceTree(source),bodyTree(envelope);Grid base=loadGrid(folder+"/base_occupancy.bin");
    constexpr int hitCount=4;
    const std::array<std::array<double,2>,hitCount>aim={{{.048,1.729},{.048,1.729},{.048,1.729},{.039,1.741}}};std::array<std::vector<double>,hitCount>occupancyTimes,boundaryTimes,clipTimes;std::array<HitResult,hitCount>lastHit;std::array<ClipResult,hitCount>lastClip;std::array<Provenance,hitCount>lastProof;std::array<EdgeProof,hitCount>edgeProof;std::array<std::size_t,hitCount>quads{},tiles{};
    std::array<std::vector<double>,hitCount>joinTimes;std::array<JoinResult,hitCount>joined;
    const int repeats=40;for(int repetition=0;repetition<repeats;++repetition){Grid grid=base;for(int i=0;i<hitCount;++i){auto start=Clock::now();auto h=hit(grid,aim[i][0],aim[i][1]);occupancyTimes[i].push_back(ms(start));start=Clock::now();auto boundary=dirtyBoundary(grid,h.dirty);boundaryTimes[i].push_back(ms(start));start=Clock::now();auto cut=exterior(source,grid);clipTimes[i].push_back(ms(start));if(repetition>=repeats-3){start=Clock::now();std::set<Cell>allTiles;for(auto&item:grid.tiles)allTiles.insert(item.first);auto fullBoundary=dirtyBoundary(grid,allTiles);auto complete=join(source,sourceTree,bodyTree,grid,cut,fullBoundary);joinTimes[i].push_back(ms(start));if(repetition==repeats-1){joined[i]=std::move(complete);writeJoined("joined_state_"+std::to_string(i+1)+".bin",joined[i],source.bones);}}if(repetition==repeats-1){lastProof[i]=inspectProvenance(source,cut);edgeProof[i]=inspectEdges(source,grid,cut);lastHit[i]=std::move(h);lastClip[i]=std::move(cut);quads[i]=boundary.size();tiles[i]=grid.tiles.size();}}}
    auto percentile=[](std::vector<double>v,double p){std::sort(v.begin(),v.end());return v[std::min(v.size()-1,std::size_t(p*double(v.size()-1)))];};
    for(int i=0;i<hitCount;++i)if(edgeProof[i].unexpectedOpen||edgeProof[i].nonmanifold||lastProof[i].maxWeightConflict>2e-6)throw std::runtime_error("Actual GFY exterior topology/provenance gate failed");
    std::cout<<std::setprecision(12)<<"{\n\"analytical_and_mixed_joint_tests_passed\":true,\n\"randomized_angled_joint_cases\":96,\n\"independent_point_coverage_samples_max\":24576,\n\"repetitions\":"<<repeats<<",\n\"source_faces\":"<<source.f.size()<<",\n\"hits\":[\n";
    for(int i=0;i<hitCount;++i){auto&h=lastHit[i];auto&c=lastClip[i];auto&p=lastProof[i];if(i)std::cout<<",\n";std::size_t triangles=0;for(auto&f:c.polygons)triangles+=f.polygon.size()-2;
        std::cout<<"{\"hit\":"<<i+1<<",\"surface_z\":"<<h.surface<<",\"fixed_radius_m\":0.017,\"new_solid_voxels\":"<<h.freshSolid<<",\"new_removed_voxels\":"<<h.freshMask<<",\"allocated_tiles\":"<<tiles[i]<<",\"tile_mask_bytes\":"<<tiles[i]*64<<",\"dirty_tiles_with_halo\":"<<h.dirty.size()<<",\"dirty_voxel_boundary_quads\":"<<quads[i]<<",\"source_triangles_unchanged\":"<<c.unchanged<<",\"clipped_exterior_triangles\":"<<triangles<<",\"candidate_cell_tests\":"<<c.candidateCells<<",\"original_exterior_area_m2\":"<<c.originalArea<<",\"remaining_exterior_area_m2\":"<<c.remainingArea<<",\"max_shared_weight_conflict\":"<<p.maxWeightConflict<<",\"max_barycentric_sum_error\":"<<p.maxBarySumError<<",\"max_position_from_provenance_error_m\":"<<p.maxReconstructionError;
        std::cout<<",\"preserved_original_open_edges\":"<<edgeProof[i].originalBoundary<<",\"intentional_cut_boundary_edges\":"<<edgeProof[i].cutBoundary<<",\"unexpected_open_edges\":"<<edgeProof[i].unexpectedOpen<<",\"nonmanifold_polygon_edges\":"<<edgeProof[i].nonmanifold;
        std::cout<<",\"joined_triangles\":"<<joined[i].triangles.size()<<",\"joined_cavity_triangles\":"<<joined[i].cavityTriangles<<",\"joined_original_open_edges\":"<<joined[i].originalOpen<<",\"joined_new_open_edges\":"<<joined[i].newOpen<<",\"joined_nonmanifold_edges\":"<<joined[i].nonmanifold<<",\"joined_max_weight_conflict\":"<<joined[i].maxJoinWeightConflict<<",\"cut_perimeter_m\":"<<joined[i].cutPerimeter<<",\"cavity_join_median_ms\":"<<percentile(joinTimes[i],.5)<<",\"cavity_join_repetitions\":3";
        std::cout<<",\"occupancy_median_ms\":"<<percentile(occupancyTimes[i],.5)<<",\"occupancy_p95_ms\":"<<percentile(occupancyTimes[i],.95)<<",\"dirty_boundary_median_ms\":"<<percentile(boundaryTimes[i],.5)<<",\"dirty_boundary_p95_ms\":"<<percentile(boundaryTimes[i],.95)<<",\"exterior_clip_median_ms\":"<<percentile(clipTimes[i],.5)<<",\"exterior_clip_p95_ms\":"<<percentile(clipTimes[i],.95)<<"}";}
    bool sealed=true;for(auto&j:joined)sealed=sealed&&j.originalOpen==65&&j.newOpen==0&&j.nonmanifold==0&&j.maxJoinWeightConflict<2e-6;
    std::cout<<"\n],\n\"fully_joined_cavity_implemented\":"<<(sealed?"true":"false")<<",\n\"native_engine_linked\":false,\n\"runtime_admitted\":false\n}\n";
    return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
