#pragma once
#include <thread>
#include <atomic>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
// Worker-only connectivity of the *remaining* anatomy. No cutter, joint-plane
// assumption, health/chance policy, mesh mutation or physics object access.
// Face-connected column runs avoid a per-voxel graph for the full human body.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <type_traits>
#include <vector>
#if defined(_MSC_VER)
#include <intrin.h>
#endif
#include "separation/ThreadBudget.h"
namespace gfy_connectivity {
struct Limits {
    std::size_t columns{8000000}, spans{4000000}, removal_runs{4000000};
    std::uint64_t work{64000000};
};
struct Span {int lo{},hi{};std::uint32_t component{};};
struct Column {int x{},y{};std::uint32_t first{},count{};};
struct Component {
    std::uint64_t cells{};
    std::array<int,3> lo{{INT32_MAX,INT32_MAX,INT32_MAX}}, hi{};
};
inline void require(bool valid,const char* reason){if(!valid)throw std::runtime_error(reason);}
inline bool xyLess(int ax,int ay,int bx,int by){return ay<by||(ay==by&&ax<bx);}
struct Topology {
    std::array<int,3> size{};
    std::array<double,3> origin{};
    double pitch{};
    std::vector<Column> columns;
    std::vector<Span> spans;
    // ID zero denotes air/removed material. IDs are local to this snapshot.
    std::vector<Component> components{1};
    std::uint64_t work{},cells{};
    const Column* column(int x,int y)const noexcept {
        auto i=std::lower_bound(columns.begin(),columns.end(),std::array<int,2>{{x,y}},
            [](const Column&c,const std::array<int,2>&p){return xyLess(c.x,c.y,p[0],p[1]);});
        return i!=columns.end()&&i->x==x&&i->y==y?&*i:nullptr;
    }
    std::uint32_t at(int x,int y,int z)const noexcept {
        auto c=column(x,y);if(!c)return 0;
        auto first=spans.begin()+c->first,last=first+c->count;
        auto i=std::upper_bound(first,last,z,[](int z,const Span&s){return z<s.lo;});
        if(i==first)return 0;--i;return z<i->hi?i->component:0;
    }
};
namespace detail {
struct Removed {int x,y,lo,hi;};
struct Union {
    std::vector<std::uint32_t> parent;
    std::vector<std::uint8_t> rank;
    explicit Union(std::size_t n):parent(n),rank(n){for(std::size_t i=0;i<n;++i)parent[i]=std::uint32_t(i);}
    std::uint32_t root(std::uint32_t i){while(parent[i]!=i){parent[i]=parent[parent[i]];i=parent[i];}return i;}
    void join(std::uint32_t a,std::uint32_t b){a=root(a);b=root(b);if(a==b)return;
        if(rank[a]<rank[b])std::swap(a,b);parent[b]=a;if(rank[a]==rank[b])++rank[a];}
};
}
template<class Grid> Topology build(const Grid&g,Limits limits={}){
    Topology out;out.pitch=g.pitch;
#ifdef GFY_RX_PROF
    auto rxT=std::chrono::steady_clock::now();auto rxMark=[&](const char*n){auto now=std::chrono::steady_clock::now();std::cerr<<"rxprof conn."<<n<<' '<<std::chrono::duration<double,std::milli>(now-rxT).count()<<std::endl;rxT=now;};
#else
    auto rxMark=[](const char*){};
#endif
    require(std::isfinite(g.pitch)&&g.pitch>0,"connectivity_pitch");
    for(int a=0;a<3;++a){out.origin[a]=g.origin[a];out.size[a]=g.size[a];
        require(std::isfinite(out.origin[a])&&out.size[a]>0&&out.size[a]<=65535,"connectivity_domain");}
    auto spend=[&](std::uint64_t n=1){require(n<=limits.work-out.work,"connectivity_work_budget");out.work+=n;};
    auto firstAnatomyCell=[&](int boundaryCell,int axis){
        // Correct floating-point ceil against the SAME point sampler used by
        // Grid. This preserves Low/Medium/High quality lattice semantics.
        const auto&a=*g.anatomy;const double boundary=a.origin[axis]+boundaryCell*a.pitch;
        int c=std::clamp(int(std::ceil((boundary-g.origin[axis])/g.pitch-.5)),0,g.size[axis]);
        auto sample=[&](int i){return int(std::floor((g.origin[axis]+(i+.5)*g.pitch-a.origin[axis])/a.pitch));};
        while(c>0&&sample(c-1)>=boundaryCell)--c;
        while(c<g.size[axis]&&sample(c)<boundaryCell)++c;return c;
    };
    std::vector<detail::Removed> removed;
    auto appendRemoved=[&](int x,int y,int lo,int hi){if(lo>=hi)return;
        require(removed.size()<limits.removal_runs,"connectivity_removal_budget");
        removed.push_back({x,y,lo,hi});};
    for(const auto&tile:g.tiles){
        // Grid stores one x slab per word and one y row per byte.
        for(int x=0;x<8;++x)for(int y=0;y<8;++y){spend();
            unsigned bits=unsigned((tile.second.removed[x]>>(y*8))&255u);
            if(!bits)continue;
            const int gx=tile.first.x*8+x,gy=tile.first.y*8+y;
            if(gx<0||gy<0||gx>=g.size.x||gy>=g.size.y)continue;
            using OwnerColumn=std::decay_t<decltype(g.ownership->body_columns.front())>;
            const OwnerColumn*ownerColumn=nullptr;
            std::size_t anatomyColumn=0;bool hasBase=true;
            if(g.ownership){const auto&columns=g.ownership->body_columns;
                auto c=std::lower_bound(columns.begin(),columns.end(),std::array<int,2>{{gx,gy}},
                    [](const auto&column,const std::array<int,2>&p){return xyLess(column.x,column.y,p[0],p[1]);});
                if(c==columns.end()||c->x!=gx||c->y!=gy)hasBase=false;
                else ownerColumn=&*c;
            }else if(g.anatomy){const auto&a=*g.anatomy;
                const int ax=int(std::floor((g.origin[0]+(gx+.5)*g.pitch-a.origin[0])/a.pitch));
                const int ay=int(std::floor((g.origin[1]+(gy+.5)*g.pitch-a.origin[1])/a.pitch));
                if(ax<0||ay<0||ax>=int(a.size[0])||ay>=int(a.size[1]))hasBase=false;
                else{anatomyColumn=std::size_t(ay)*a.size[0]+ax;
                    hasBase=a.offsets[anatomyColumn]!=a.offsets[anatomyColumn+1];}
            }
            if(!hasBase)continue;
            for(int z=0;z<8;){if(!(bits&(1u<<z))){++z;continue;}int lo=z++;
                while(z<8&&(bits&(1u<<z)))++z;
                spend();lo+=tile.first.z*8;const int hi=tile.first.z*8+z;
                if(g.ownership){const auto&owner=*g.ownership;
                    auto first=owner.body_spans.begin()+ownerColumn->first,last=first+ownerColumn->count;
                    auto s=std::lower_bound(first,last,lo,[](const auto&span,int cell){return span.hi<=cell;});
                    for(;s!=last&&s->lo<hi;++s){spend();appendRemoved(gx,gy,std::max(lo,s->lo),std::min(hi,s->hi));}
                }else if(g.anatomy){const auto&a=*g.anatomy;
                    const int az=int(std::floor((g.origin[2]+(lo+.5)*g.pitch-a.origin[2])/a.pitch));
                    auto first=a.runs.begin()+a.offsets[anatomyColumn],last=a.runs.begin()+a.offsets[anatomyColumn+1];
                    auto s=std::lower_bound(first,last,az,[](const auto&run,int cell){return run.hi<=cell;});
                    for(;s!=last;++s){spend();const int start=firstAnatomyCell(s->lo,2);
                        if(start>=hi)break;
                        appendRemoved(gx,gy,std::max(lo,start),std::min(hi,firstAnatomyCell(s->hi,2)));}
                }else appendRemoved(gx,gy,lo,hi);}
        }
    }
    rxMark("collect_removed");
    // Same order as sorting by (y,x,lo): every field is within [0,65535]
    // (domain limit), so a packed 64-bit key y|x|lo|hi sorts identically.
    {std::vector<std::uint64_t>keys(removed.size());
     for(std::size_t i=0;i<removed.size();++i){const auto&r=removed[i];
      keys[i]=(std::uint64_t(std::uint16_t(r.y))<<48)|(std::uint64_t(std::uint16_t(r.x))<<32)|(std::uint64_t(std::uint16_t(r.lo))<<16)|std::uint64_t(std::uint16_t(r.hi));}
     // Keys are distinct integers: sorting four quarters concurrently and
     // merging gives exactly the std::sort result.
     if(keys.size()>=(1u<<16)){const std::size_t n=keys.size();const std::size_t cut[5]={0,n/4,n/2,3*n/4,n};
      std::vector<std::thread>threads;
      try{for(int q=1;q<4;++q)threads.emplace_back([&keys,&cut,q]{gfy_live::core::enterPainterThread();std::sort(keys.begin()+std::ptrdiff_t(cut[q]),keys.begin()+std::ptrdiff_t(cut[q+1]));});}catch(...){}
      std::sort(keys.begin(),keys.begin()+std::ptrdiff_t(cut[1]));
      for(auto&t:threads)t.join();
      for(std::size_t q=threads.size()+1;q<4;++q)std::sort(keys.begin()+std::ptrdiff_t(cut[q]),keys.begin()+std::ptrdiff_t(cut[q+1]));
      // inplace_merge's temporary buffer uses nothrow new; with the Painter
      // allocation policy a failure falls back to the in-place merge.
      std::thread upper([&]{gfy_live::core::enterPainterThread();std::inplace_merge(keys.begin()+std::ptrdiff_t(cut[2]),keys.begin()+std::ptrdiff_t(cut[3]),keys.end());});
      std::inplace_merge(keys.begin(),keys.begin()+std::ptrdiff_t(cut[1]),keys.begin()+std::ptrdiff_t(cut[2]));upper.join();
      std::inplace_merge(keys.begin(),keys.begin()+std::ptrdiff_t(cut[2]),keys.end());}
     else std::sort(keys.begin(),keys.end());
     for(std::size_t i=0;i<keys.size();++i){const auto k=keys[i];removed[i]={int((k>>32)&0xFFFF),int(k>>48),int((k>>16)&0xFFFF),int(k&0xFFFF)};}}
    int x0=0,y0=0,x1=g.size.x,y1=g.size.y;
    if(g.anatomy){const auto&a=*g.anatomy;
        x0=firstAnatomyCell(0,0);x1=firstAnatomyCell(int(a.size[0]),0);
        y0=firstAnatomyCell(0,1);y1=firstAnatomyCell(int(a.size[1]),1);
    }
    if(!g.anatomy&&!g.ownership)require(std::uint64_t(x1-x0)*(y1-y0)<=limits.columns,"connectivity_column_budget");
    rxMark("sort_removed");std::size_t removalAt=0;
    std::size_t visitedColumns=0;
    std::vector<std::array<int,2>> base;
    std::vector<int> activeXs;
    std::size_t ownedAt=0;
    for(int y=y0;y<y1;++y){
      activeXs.clear();
      if(g.ownership){const auto&columns=g.ownership->body_columns;
        while(ownedAt<columns.size()&&columns[ownedAt].y<y)++ownedAt;
        auto i=ownedAt;while(i<columns.size()&&columns[i].y==y){activeXs.push_back(columns[i].x);++i;}
        ownedAt=i;
      }else if(g.anatomy){const auto&a=*g.anatomy;
        int ay=int(std::floor((g.origin[1]+(y+.5)*g.pitch-a.origin[1])/a.pitch));
        if(ay<0||ay>=int(a.size[1]))continue;
        for(int ax=0;ax<int(a.size[0]);++ax){spend();auto c=std::size_t(ay)*a.size[0]+ax;
          if(a.offsets[c]==a.offsets[c+1])continue;
          const int lo=firstAnatomyCell(ax,0),hi=firstAnatomyCell(ax+1,0);
          for(int x=lo;x<hi;++x)activeXs.push_back(x);}
      }else for(int x=x0;x<x1;++x)activeXs.push_back(x);
      for(int x:activeXs){spend();base.clear();
        require(++visitedColumns<=limits.columns,"connectivity_column_budget");
        auto appendBase=[&](int lo,int hi){if(lo>=hi)return;
            if(!base.empty()&&lo<=base.back()[1])base.back()[1]=std::max(base.back()[1],hi);
            else base.push_back({{lo,hi}});};
        if(g.ownership){const auto&owner=*g.ownership;auto column=std::lower_bound(owner.body_columns.begin(),owner.body_columns.end(),std::array<int,2>{{x,y}},
                [](const auto&c,const std::array<int,2>&p){return c.y<p[1]||(c.y==p[1]&&c.x<p[0]);});
            require(column!=owner.body_columns.end()&&column->x==x&&column->y==y,"piece_connectivity_column_missing");
            for(auto i=column->first;i<column->first+column->count;++i){spend();appendBase(owner.body_spans[i].lo,owner.body_spans[i].hi);}
        }else if(g.anatomy){const auto&a=*g.anatomy;
            const int ax=int(std::floor((g.origin[0]+(x+.5)*g.pitch-a.origin[0])/a.pitch));
            const int ay=int(std::floor((g.origin[1]+(y+.5)*g.pitch-a.origin[1])/a.pitch));
            if(ax<0||ay<0||ax>=int(a.size[0])||ay>=int(a.size[1]))continue;
            auto column=std::size_t(ay)*a.size[0]+ax;
            for(auto r=a.offsets[column];r<a.offsets[column+1];++r){spend();const auto&s=a.runs[r];
                appendBase(firstAnatomyCell(s.lo,2),firstAnatomyCell(s.hi,2));}
        }else{
            for(int z=0;z<g.size.z;){spend();if(!g.solidBase({x,y,z})){++z;continue;}int lo=z++;
                while(z<g.size.z&&g.solidBase({x,y,z})){spend();++z;}appendBase(lo,z);}
        }
        if(base.empty())continue;
        while(removalAt<removed.size()&&xyLess(removed[removalAt].x,removed[removalAt].y,x,y))++removalAt;
        auto end=removalAt;while(end<removed.size()&&removed[end].x==x&&removed[end].y==y)++end;
        Column column{x,y,std::uint32_t(out.spans.size()),0};
        auto append=[&](int lo,int hi){if(lo>=hi)return;
            if(out.spans.size()>column.first&&lo==out.spans.back().hi){out.spans.back().hi=hi;return;}
            require(out.spans.size()<limits.spans&&out.spans.size()<UINT32_MAX,"connectivity_span_budget");
            out.spans.push_back({lo,hi,0});};
        auto r=removalAt;
        for(auto b:base){int lo=b[0],hi=b[1];while(r<end&&removed[r].hi<=lo)++r;
            while(r<end&&removed[r].lo<hi){spend();append(lo,std::min(hi,removed[r].lo));
                lo=std::max(lo,removed[r].hi);if(removed[r].hi>=hi)break;++r;}
            append(lo,hi);}
        column.count=std::uint32_t(out.spans.size()-column.first);
        if(column.count)out.columns.push_back(column);
      }
    }
    rxMark("spans");detail::Union sets(out.spans.size());
    auto connect=[&](const Column&a,const Column*b){if(!b)return;
        auto i=a.first,j=b->first;const auto ie=i+a.count,je=j+b->count;
        while(i<ie&&j<je){spend();const auto&u=out.spans[i];const auto&v=out.spans[j];
            // Positive face area only. Corner/edge contacts cannot hold a limb.
            if(std::max(u.lo,v.lo)<std::min(u.hi,v.hi))sets.join(i,j);
            if(u.hi<v.hi)++i;else ++j;}
    };
    // Neighbour columns found by a forward scan instead of binary searches
    // (columns are sorted by y then x); same neighbours, same join order.
    {std::size_t below=0;
     for(std::size_t k=0;k<out.columns.size();++k){const auto&c=out.columns[k];
      const Column*left=k&&out.columns[k-1].y==c.y&&out.columns[k-1].x==c.x-1?&out.columns[k-1]:nullptr;
      while(below<out.columns.size()&&xyLess(out.columns[below].x,out.columns[below].y,c.x,c.y-1))++below;
      const Column*down=below<out.columns.size()&&out.columns[below].x==c.x&&out.columns[below].y==c.y-1?&out.columns[below]:nullptr;
      connect(c,left);connect(c,down);}}
    rxMark("union");std::vector<std::uint32_t> ids(out.spans.size());
    for(const auto&c:out.columns)for(auto i=c.first;i<c.first+c.count;++i){spend();
        auto root=sets.root(i);if(!ids[root]){ids[root]=std::uint32_t(out.components.size());out.components.push_back({});}
        auto&s=out.spans[i];s.component=ids[root];auto&component=out.components[s.component];
        component.cells+=std::uint64_t(s.hi-s.lo);out.cells+=std::uint64_t(s.hi-s.lo);
        component.lo[0]=std::min(component.lo[0],c.x);component.hi[0]=std::max(component.hi[0],c.x+1);
        component.lo[1]=std::min(component.lo[1],c.y);component.hi[1]=std::max(component.hi[1],c.y+1);
        component.lo[2]=std::min(component.lo[2],s.lo);component.hi[2]=std::max(component.hi[2],s.hi);
    }
    rxMark("label");
#ifdef GFY_RX_PROF
    std::cerr<<"rxprof conn.sizes columns="<<out.columns.size()<<" spans="<<out.spans.size()<<" removed_runs="<<removed.size()<<" components="<<out.components.size()<<std::endl;
#endif
    return out;
}
struct Descendant {std::uint32_t component{};std::uint64_t cells{};};
struct Split {std::uint32_t original{};std::vector<Descendant> pieces;};
inline std::vector<Split> compare(const Topology&before,const Topology&after){
    require(before.size==after.size&&before.origin==after.origin&&before.pitch==after.pitch,"connectivity_snapshot_domain");
    std::map<std::uint32_t,std::map<std::uint32_t,std::uint64_t>> family;
    for(const auto&c:after.columns){const auto*b=before.column(c.x,c.y);require(b!=nullptr,"connectivity_material_regrowth");
        auto j=b->first;for(auto i=c.first;i<c.first+c.count;++i){const auto&s=after.spans[i];
            while(j<b->first+b->count&&before.spans[j].hi<=s.lo)++j;
            require(j<b->first+b->count&&before.spans[j].lo<=s.lo&&before.spans[j].hi>=s.hi,"connectivity_material_regrowth");
            family[before.spans[j].component][s.component]+=std::uint64_t(s.hi-s.lo);}
    }
    std::vector<Split> result;
    for(const auto&f:family)if(f.second.size()>1){Split split;split.original=f.first;
        for(const auto&p:f.second)split.pieces.push_back({p.first,p.second});result.push_back(std::move(split));}
    return result;
}
// ---- Derived connectivity (separation-track P1.4 / P1.7) -------------------
// derive(parent,g) returns build(g) in every field except the `work` counter,
// provided `parent` is build(g0) of a grid g0 with the same lattice and the
// same base as g (the same anatomy object without ownership, or the same
// ownership) whose removed cells are a subset of g's removed cells. Every
// call site guarantees that relation structurally; GFY_SEP_VERIFY builds
// check derive()==build() field by field.
// Why the result equals build(g):
//  * In a column, build(g) emits the maximal intervals of B\R: B (base runs)
//    is merged when touching (appendBase), removal runs are disjoint, pieces
//    of one base interval are separated by removed cells and base intervals by
//    gaps, so no two emitted spans touch. The anatomy clip (:114-120) and the
//    owner clip (:110-113) of the removal runs only intersect R with B, which
//    leaves B\R unchanged.
//  * The parent's spans are the maximal intervals of B\R0 with R0 a subset of
//    R, hence (parent spans)\R = B\R. Subtracting g's raw removed bits (runs
//    merged across tile slabs) span by span emits the same maximal intervals
//    in the same (y,x,lo) order, and a column left empty is dropped as in
//    build (:202).
//  * Union (positive face overlap with the left and the lower column) and
//    the first-appearance labelling are build()'s code (:205-229) on the same
//    spans. Component ids depend only on the partition into connected sets.
// Budgets are never stricter than build(g) (same Limits values):
//  * removal runs: maximal runs that intersect a parent span; each contains
//    at least one run that build(g) appends, so the count is <= build(g)'s;
//  * spans: the same limit on the same output spans;
//  * work: one unit per parent span, per subtraction step, per union step and
//    per label, each bounded by a build(g) spend on the same or a coarser
//    object.
// So a grid that build(g) accepts is accepted here with the same output; a
// grid build(g) rejects on a budget may be accepted (failure threshold only).
namespace detail {
struct TileRef {int ty,tx,tz;const std::uint64_t* words;};
inline bool tileRefLess(const TileRef&a,const TileRef&b){return a.ty!=b.ty?a.ty<b.ty:(a.tx!=b.tx?a.tx<b.tx:a.tz<b.tz);}
// Removed z-runs of one band of 8 grid rows (tiles of one ty, sorted by tx,tz):
// maximal per column (merged across tile slabs), in (y,x,lo) order.
inline void bandRuns(const TileRef*first,const TileRef*last,std::vector<Removed>&out){
    out.clear();
    for(int dy=0;dy<8;++dy){const int y=first->ty*8+dy;
        for(const TileRef*group=first;group!=last;){const TileRef*groupEnd=group;
            while(groupEnd!=last&&groupEnd->tx==group->tx)++groupEnd;
            for(int dx=0;dx<8;++dx){const int x=group->tx*8+dx;const std::size_t start=out.size();
                for(const TileRef*t=group;t!=groupEnd;++t){
                    // Grid stores one x slab per word and one y row per byte.
                    unsigned bits=unsigned((t->words[dx]>>(dy*8))&255u);
                    const int zBase=t->tz*8;
                    while(bits){int lo=0;while(!(bits&(1u<<lo)))++lo;int hi=lo;while(hi<8&&(bits&(1u<<hi)))++hi;
                        bits&=~(((1u<<hi)-1u)&~((1u<<lo)-1u));
                        if(out.size()>start&&out.back().hi==zBase+lo)out.back().hi=zBase+hi;
                        else out.push_back({x,y,zBase+lo,zBase+hi});}
                }
            }
            group=groupEnd;
        }
    }
}
// Union and first-appearance labelling of build() (:205-229), unchanged.
template<class Spend> void labelComponents(Topology&out,Spend&&spend){
    Union sets(out.spans.size());
    auto connect=[&](const Column&a,const Column*b){if(!b)return;
        auto i=a.first,j=b->first;const auto ie=i+a.count,je=j+b->count;
        while(i<ie&&j<je){spend();const auto&u=out.spans[i];const auto&v=out.spans[j];
            if(std::max(u.lo,v.lo)<std::min(u.hi,v.hi))sets.join(i,j);
            if(u.hi<v.hi)++i;else ++j;}
    };
    {std::size_t below=0;
     for(std::size_t k=0;k<out.columns.size();++k){const auto&c=out.columns[k];
      const Column*left=k&&out.columns[k-1].y==c.y&&out.columns[k-1].x==c.x-1?&out.columns[k-1]:nullptr;
      while(below<out.columns.size()&&xyLess(out.columns[below].x,out.columns[below].y,c.x,c.y-1))++below;
      const Column*down=below<out.columns.size()&&out.columns[below].x==c.x&&out.columns[below].y==c.y-1?&out.columns[below]:nullptr;
      connect(c,left);connect(c,down);}}
    std::vector<std::uint32_t> ids(out.spans.size());
    for(const auto&c:out.columns)for(auto i=c.first;i<c.first+c.count;++i){spend();
        auto root=sets.root(i);if(!ids[root]){ids[root]=std::uint32_t(out.components.size());out.components.push_back({});}
        auto&s=out.spans[i];s.component=ids[root];auto&component=out.components[s.component];
        component.cells+=std::uint64_t(s.hi-s.lo);out.cells+=std::uint64_t(s.hi-s.lo);
        component.lo[0]=std::min(component.lo[0],c.x);component.hi[0]=std::max(component.hi[0],c.x+1);
        component.lo[1]=std::min(component.lo[1],c.y);component.hi[1]=std::max(component.hi[1],c.y+1);
        component.lo[2]=std::min(component.lo[2],s.lo);component.hi[2]=std::max(component.hi[2],s.hi);
    }
}
}
// R67 (exact): derive() with its bands and its labelling side by side (see derive below).
namespace detail {
struct BandOut {std::vector<Column> columns;std::vector<Span> spans;std::vector<std::uint32_t> parentSpans;std::uint64_t work{},removalRuns{};std::size_t spanCount{},columnCount{};bool copied{};};
// The serial code's three caps, shared by the threads of one side-by-side derive: each thread adds what it did every few columns;
// the first sum past a cap stops every thread (the serial code then refuses exactly where it does).
struct DeriveBudget {
    // a thread reports after at most this many counted units; with T threads the sums never run more than T*quantum units (for
    // spans: T*quantum*(12+4) bytes of span + parent-span rows) past a crossed limit before every thread has stopped
    static constexpr std::uint64_t quantum=1024;
    std::atomic<std::uint64_t> work{0},spans{0},runs{0};std::atomic<bool> crossed{false};std::uint64_t workLimit,spanLimit,runLimit;
    explicit DeriveBudget(const Limits&l):workLimit(l.work),spanLimit(std::min<std::uint64_t>(l.spans,UINT32_MAX)),runLimit(l.removal_runs){}
    bool add(std::uint64_t w,std::uint64_t s,std::uint64_t r){
        if(crossed.load(std::memory_order_relaxed))return false;
        const auto W=work.fetch_add(w,std::memory_order_relaxed)+w,S=spans.fetch_add(s,std::memory_order_relaxed)+s,R=runs.fetch_add(r,std::memory_order_relaxed)+r;
        if(W>workLimit||S>spanLimit||R>runLimit){crossed.store(true,std::memory_order_relaxed);return false;}
        return true;}
};
// What a side-by-side derive did before it stopped (probe evidence; null in the game).
struct DeriveAbort {bool crossed{};std::uint64_t work{},spans{},runs{},unionSteps{};const char*phase{""};};
// The serial labelling's result from spans whose columns are final. False when the serial code would refuse on its work budget
// (union steps against room = limit - subtract work, then the N label units); out.spans' labels are then incomplete (discarded).
inline bool labelSideBySide(Topology&out,const Limits&limits,std::uint64_t&labelWork,unsigned threads,DeriveAbort*abort){
    const std::size_t N=out.spans.size(),NC=out.columns.size();
    if(!N||N>=std::size_t(UINT32_MAX))return false;
    const std::uint64_t room=limits.work>=out.work?limits.work-out.work:0;
    std::unique_ptr<std::atomic<std::uint32_t>[]> par(new std::atomic<std::uint32_t>[N]);
    gfy_live::sep::parallel(N,1u<<15,threads,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t i=a;i<b;++i)par[i].store(std::uint32_t(i),std::memory_order_relaxed);});
    auto find=[&](std::uint32_t a){for(;;){std::uint32_t q=par[a].load(std::memory_order_relaxed);if(q==a)return a;const std::uint32_t g2=par[q].load(std::memory_order_relaxed);
        if(g2!=q)par[a].compare_exchange_weak(q,g2,std::memory_order_relaxed);a=g2;}};
    auto unite=[&](std::uint32_t a,std::uint32_t b){for(;;){a=find(a);b=find(b);if(a==b)return;if(a<b)std::swap(a,b);std::uint32_t e=a;
        if(par[a].compare_exchange_strong(e,b,std::memory_order_relaxed))return;}};
    std::atomic<std::uint64_t> steps{0};std::atomic<bool> crossed{false};
    gfy_live::sep::parallel(NC,4096,threads,[&](std::size_t k0,std::size_t k1,std::size_t){
        if(k0>=k1)return;std::uint64_t w=0;const auto&cols=out.columns;
        // union steps reported every DeriveBudget::quantum steps inside the merge walk: at most T*quantum steps past the crossing
        auto flush=[&]{const auto total=steps.fetch_add(w,std::memory_order_relaxed)+w;w=0;if(total>room)crossed.store(true,std::memory_order_relaxed);return !crossed.load(std::memory_order_relaxed);};
        bool stop=false;
        auto connect=[&](const Column&a,const Column*b){if(!b||stop)return;
            auto i=a.first,j=b->first;const auto ie=i+a.count,je=j+b->count;
            while(i<ie&&j<je){++w;if(w>=DeriveBudget::quantum&&!flush()){stop=true;return;}const auto&u=out.spans[i];const auto&v=out.spans[j];
                if(std::max(u.lo,v.lo)<std::min(u.hi,v.hi))unite(i,j);
                if(u.hi<v.hi)++i;else ++j;}};
        std::size_t below=std::size_t(std::lower_bound(cols.begin(),cols.end(),std::array<int,2>{{cols[k0].x,cols[k0].y-1}},
            [](const Column&c,const std::array<int,2>&q){return xyLess(c.x,c.y,q[0],q[1]);})-cols.begin());
        for(std::size_t k=k0;k<k1;++k){const auto&c=cols[k];
            if((k&2047u)==0)gfy_live::sep::throwIfCanceled(); // split2: a derive nobody waits for any more stops here (prepare(): still connected)
            const Column*left=k&&cols[k-1].y==c.y&&cols[k-1].x==c.x-1?&cols[k-1]:nullptr;
            while(below<NC&&xyLess(cols[below].x,cols[below].y,c.x,c.y-1))++below;
            const Column*down=below<NC&&cols[below].x==c.x&&cols[below].y==c.y-1?&cols[below]:nullptr;
            connect(c,left);connect(c,down);
            if(stop)return;}
        flush();});
    const std::uint64_t uw=steps.load();
    if(abort){abort->unionSteps=uw;}
    if(crossed.load()||uw+N>room){if(abort){abort->crossed=true;abort->phase=crossed.load()?"union":"label";}return false;}
    // every span's parent made its root (roots are final once the unions are done), the roots in span order, ids by that order
    const std::size_t rc=std::max<std::size_t>(1,gfy_live::sep::chunkCount(N,1u<<15,threads));
    std::vector<std::vector<std::uint32_t>> rootParts(rc);
    gfy_live::sep::parallel(N,1u<<15,threads,[&](std::size_t a,std::size_t b,std::size_t chunk){auto&R=rootParts[chunk];
        for(std::size_t block=a;block<b;block+=8192){gfy_live::sep::throwIfCanceled(); // split2: per block, the walk itself is unchanged
         for(std::size_t i=block,blockEnd=std::min(b,block+8192);i<blockEnd;++i){const auto r=find(std::uint32_t(i));par[i].store(r,std::memory_order_relaxed);if(r==i)R.push_back(r);}}});
    std::vector<std::uint32_t> roots;for(auto&R:rootParts){roots.insert(roots.end(),R.begin(),R.end());std::vector<std::uint32_t>().swap(R);}
    const std::size_t K=roots.size();
    out.components.assign(K+1,Component{});
    auto idOf=[&](std::uint32_t r){return std::uint32_t(std::lower_bound(roots.begin(),roots.end(),r)-roots.begin())+1;};
    const std::size_t sc=std::max<std::size_t>(1,gfy_live::sep::chunkCount(NC,4096,threads));
    const bool local=K<=4096;
    std::vector<std::vector<Component>> part(local?sc:0,std::vector<Component>(local?K+1:0));std::vector<std::uint64_t> cells(sc,0);
    gfy_live::sep::parallel(NC,4096,threads,[&](std::size_t k0,std::size_t k1,std::size_t chunk){std::uint64_t total=0;
        for(std::size_t k=k0;k<k1;++k){if((k&2047u)==0)gfy_live::sep::throwIfCanceled(); // split2
            const auto&c=out.columns[k];for(auto i=c.first;i<c.first+c.count;++i){auto&s=out.spans[i];s.component=idOf(par[i].load(std::memory_order_relaxed));
            if(!local)continue;auto&component=part[chunk][s.component];
            component.cells+=std::uint64_t(s.hi-s.lo);total+=std::uint64_t(s.hi-s.lo);
            component.lo[0]=std::min(component.lo[0],c.x);component.hi[0]=std::max(component.hi[0],c.x+1);
            component.lo[1]=std::min(component.lo[1],c.y);component.hi[1]=std::max(component.hi[1],c.y+1);
            component.lo[2]=std::min(component.lo[2],s.lo);component.hi[2]=std::max(component.hi[2],s.hi);}}
        cells[chunk]=total;});
    if(local){for(std::size_t c=0;c<sc;++c){out.cells+=cells[c];for(std::size_t k=1;k<=K;++k){auto&d=out.components[k];const auto&q=part[c][k];d.cells+=q.cells;
        for(int a=0;a<3;++a){d.lo[a]=std::min(d.lo[a],q.lo[a]);d.hi[a]=std::max(d.hi[a],q.hi[a]);}}}}
    else for(const auto&c:out.columns)for(auto i=c.first;i<c.first+c.count;++i){const auto&s=out.spans[i];auto&component=out.components[s.component];
        component.cells+=std::uint64_t(s.hi-s.lo);out.cells+=std::uint64_t(s.hi-s.lo);
        component.lo[0]=std::min(component.lo[0],c.x);component.hi[0]=std::max(component.hi[0],c.x+1);
        component.lo[1]=std::min(component.lo[1],c.y);component.hi[1]=std::max(component.hi[1],c.y+1);
        component.lo[2]=std::min(component.lo[2],s.lo);component.hi[2]=std::max(component.hi[2],s.hi);}
    labelWork=uw+N;out.work+=labelWork;
    return true;
}
// The bands of derive() side by side, written in band order, then labelSideBySide. False: a cap was crossed (nothing is kept;
// the caller runs the serial code).
template<class Grid> bool deriveSideBySide(const Topology&parent,const Grid&g,std::vector<std::uint32_t>*parentSpans,const Limits&limits,std::uint64_t*unionLabel,Topology&out,DeriveAbort*abort=nullptr){
    constexpr unsigned threads=3;
#ifdef GFY_RX_PROF
    auto rxT=std::chrono::steady_clock::now();auto rxMark=[&](const char*n){auto now=std::chrono::steady_clock::now();std::cerr<<"rxprof derive."<<n<<' '<<std::chrono::duration<double,std::milli>(now-rxT).count()<<std::endl;rxT=now;};
#else
    auto rxMark=[](const char*){};
#endif
    out=Topology{};out.size=parent.size;out.origin=parent.origin;out.pitch=parent.pitch;
    std::vector<TileRef> tiles;tiles.reserve(g.tiles.size());
    for(const auto&t:g.tiles){const auto&w=t.second.removed;
        if(t.first.x<0||t.first.y<0||t.first.z<0||!(w[0]|w[1]|w[2]|w[3]|w[4]|w[5]|w[6]|w[7]))continue;
        tiles.push_back({t.first.y,t.first.x,t.first.z,w.data()});}
    std::sort(tiles.begin(),tiles.end(),tileRefLess);
    rxMark("tiles");
    const auto bandOf=[](int y){return y>=0?y/8:-1-(-1-y)/8;};
    const auto&columns=parent.columns;
    struct Band {std::size_t k0,k1,t0,t1;};std::vector<Band> bands;
    {std::size_t tileAt=0;
     for(std::size_t k=0;k<columns.size();){const int ty=bandOf(columns[k].y);std::size_t e=k+1;while(e<columns.size()&&bandOf(columns[e].y)==ty)++e;
      while(tileAt<tiles.size()&&tiles[tileAt].ty<ty)++tileAt;auto bandEnd=tileAt;while(bandEnd<tiles.size()&&tiles[bandEnd].ty==ty)++bandEnd;
      bands.push_back({k,e,tileAt,bandEnd});tileAt=bandEnd;k=e;}}
    // pass 1 (caps shared by the threads, checked every 32 columns and at every band's end)
    DeriveBudget budget(limits);
    std::vector<BandOut> outs(bands.size());std::atomic<std::size_t> nextBand{0};
    gfy_live::sep::parallel(threads,1,threads,[&](std::size_t,std::size_t,std::size_t){std::vector<Removed> band;
        for(std::size_t b;(b=nextBand.fetch_add(1,std::memory_order_relaxed))<bands.size();){gfy_live::sep::throwIfCanceled();
            if(budget.crossed.load(std::memory_order_relaxed))return;
            const auto&B=bands[b];auto&o=outs[b];const bool touched=B.t1>B.t0;
            const std::uint64_t firstSpan=columns[B.k0].first,lastSpan=std::uint64_t(columns[B.k1-1].first)+columns[B.k1-1].count;
            bool contiguous=true;for(auto j=B.k0+1;j<B.k1&&contiguous;++j)contiguous=columns[j].first==std::uint64_t(columns[j-1].first)+columns[j-1].count;
            if(!touched&&contiguous){o.copied=true;o.work=lastSpan-firstSpan;o.spanCount=std::size_t(lastSpan-firstSpan);o.columnCount=B.k1-B.k0;
                if(!budget.add(o.work,o.spanCount,0))return;continue;} // admitted by its counts: nothing is copied before pass 2
            if(touched)bandRuns(tiles.data()+B.t0,tiles.data()+B.t1,band);
            // every unit of work, span and removal run is counted as it happens and reported in quanta of DeriveBudget::quantum
            // units (summed over the three counters): a thread does at most one quantum past the crossing before it stops
            std::uint64_t fw=0,fr=0,fs=0;bool stop=false;
            auto unit=[&](std::uint64_t&counter){++counter;if(fw+fs+fr>=DeriveBudget::quantum){if(!budget.add(fw,fs,fr))stop=true;fw=fr=fs=0;}return !stop;};
            std::size_t r=0;
            for(std::size_t k=B.k0;k<B.k1&&!stop;++k){const auto&c=columns[k];
                if(!touched)band.clear();
                while(r<band.size()&&xyLess(band[r].x,band[r].y,c.x,c.y))++r;
                auto end=r;while(end<band.size()&&band[end].x==c.x&&band[end].y==c.y)++end;
                Column column{c.x,c.y,std::uint32_t(o.spans.size()),0};
                auto append=[&](int lo,int hi,std::uint32_t from){if(lo>=hi||stop)return;
                    if(o.spans.size()>column.first&&lo==o.spans.back().hi){o.spans.back().hi=hi;return;}
                    if(!unit(fs))return;
                    o.spans.push_back({lo,hi,0});if(parentSpans)o.parentSpans.push_back(from);};
                if(r==end){
                    for(auto i=c.first;i<c.first+c.count&&!stop;++i){++o.work;if(!unit(fw))break;append(parent.spans[i].lo,parent.spans[i].hi,i);}
                }else{auto q=r;std::size_t counted=SIZE_MAX;
                    for(auto i=c.first;i<c.first+c.count&&!stop;++i){++o.work;if(!unit(fw))break;int lo=parent.spans[i].lo;const int hi=parent.spans[i].hi;
                        while(q<end&&band[q].hi<=lo)++q;
                        while(q<end&&band[q].lo<hi&&!stop){++o.work;if(!unit(fw))break;
                            if(counted!=q){counted=q;++o.removalRuns;if(!unit(fr))break;}
                            append(lo,std::min(hi,band[q].lo),i);
                            lo=std::max(lo,band[q].hi);if(band[q].hi>=hi)break;++q;}
                        append(lo,hi,i);}
                    r=end;}
                column.count=std::uint32_t(o.spans.size()-column.first);
                if(column.count)o.columns.push_back(column);}
            if(stop||!budget.add(fw,fs,fr))return;
            o.spanCount=o.spans.size();o.columnCount=o.columns.size();}});
    if(abort){abort->work=budget.work.load();abort->spans=budget.spans.load();abort->runs=budget.runs.load();}
    if(budget.crossed.load()){if(abort){abort->crossed=true;abort->phase="subtract";}return false;}
    std::uint64_t work=0;std::vector<std::size_t> spanAt(bands.size()+1,0),colAt(bands.size()+1,0);
    for(std::size_t b=0;b<bands.size();++b){work+=outs[b].work;spanAt[b+1]=spanAt[b]+outs[b].spanCount;colAt[b+1]=colAt[b]+outs[b].columnCount;}
    const std::size_t totalSpans=spanAt.back();
    // pass 2: every band at its place (spans keep the parent's labels until the labelling rewrites all of them, as in the serial copy)
    out.spans.resize(totalSpans);out.columns.resize(colAt.back());if(parentSpans)parentSpans->resize(totalSpans);
    gfy_live::sep::parallel(bands.size(),1,threads,[&](std::size_t b0,std::size_t b1,std::size_t){
        for(std::size_t b=b0;b<b1;++b){gfy_live::sep::throwIfCanceled(); // split2
            auto&o=outs[b];const auto&B=bands[b];const std::size_t at=spanAt[b];
            if(o.copied){const std::uint64_t firstSpan=columns[B.k0].first;
                std::copy(parent.spans.begin()+std::ptrdiff_t(firstSpan),parent.spans.begin()+std::ptrdiff_t(firstSpan+o.spanCount),out.spans.begin()+std::ptrdiff_t(at));
                if(parentSpans)for(std::size_t i=0;i<o.spanCount;++i)(*parentSpans)[at+i]=std::uint32_t(firstSpan+i);
                for(std::size_t j=0;j<o.columnCount;++j){const auto&c=columns[B.k0+j];out.columns[colAt[b]+j]={c.x,c.y,std::uint32_t(at+(c.first-firstSpan)),c.count};}}
            else{std::copy(o.spans.begin(),o.spans.end(),out.spans.begin()+std::ptrdiff_t(at));
                if(parentSpans)std::copy(o.parentSpans.begin(),o.parentSpans.end(),parentSpans->begin()+std::ptrdiff_t(at));
                for(std::size_t j=0;j<o.columns.size();++j){auto c=o.columns[j];c.first=std::uint32_t(c.first+at);out.columns[colAt[b]+j]=c;}
                std::vector<Column>().swap(o.columns);std::vector<Span>().swap(o.spans);std::vector<std::uint32_t>().swap(o.parentSpans);}}});
    out.work=work;
    rxMark("subtract");gfy_live::sep::throwIfCanceled();
    std::uint64_t labelWork=0;
    if(!labelSideBySide(out,limits,labelWork,threads,abort))return false;
    if(unionLabel)*unionLabel=labelWork;
    rxMark("label");
#ifdef GFY_RX_PROF
    std::cerr<<"rxprof derive.sizes tiles="<<tiles.size()<<" parent_spans="<<parent.spans.size()<<" spans="<<out.spans.size()<<" removal_runs="<<budget.runs.load()<<" components="<<out.components.size()<<" work="<<out.work<<std::endl;
#endif
    return true;
}
inline bool sameDerived(const Topology&a,const Topology&b){
    if(a.size!=b.size||a.origin!=b.origin||a.pitch!=b.pitch||a.work!=b.work||a.cells!=b.cells||a.columns.size()!=b.columns.size()||a.spans.size()!=b.spans.size()||a.components.size()!=b.components.size())return false;
    for(std::size_t i=0;i<a.columns.size();++i){const auto&x=a.columns[i];const auto&y=b.columns[i];if(x.x!=y.x||x.y!=y.y||x.first!=y.first||x.count!=y.count)return false;}
    for(std::size_t i=0;i<a.spans.size();++i){const auto&x=a.spans[i];const auto&y=b.spans[i];if(x.lo!=y.lo||x.hi!=y.hi||x.component!=y.component)return false;}
    for(std::size_t i=0;i<a.components.size();++i){const auto&x=a.components[i];const auto&y=b.components[i];if(x.cells!=y.cells||x.lo!=y.lo||x.hi!=y.hi)return false;}
    return true;
}
}
// parentSpans (optional) receives, per output span, the index of the parent
// span that contains it (for compareDerived and ancestor maps).
// unionLabel (optional) receives the union+label work units (for referenceBuildBudget).
namespace detail {template<class Grid> Topology deriveSerial(const Topology&parent,const Grid&g,std::vector<std::uint32_t>*parentSpans,Limits limits,std::uint64_t*unionLabel);
#ifdef GFY_VOXEL_WORKER_TESTS
template<class Grid> void deriveLimitTest(const Topology&parent,const Grid&g,const Limits&limits,const Topology&full,std::uint64_t unionLabel);
#endif
}
template<class Grid> Topology derive(const Topology&parent,const Grid&g,std::vector<std::uint32_t>*parentSpans=nullptr,Limits limits={},
    std::uint64_t*unionLabel=nullptr){
    // R67: side by side when the caller may fan out (an exact copy of the result below; see detail::deriveSideBySide)
    static const bool serialDerive=std::getenv("GFY_SERIAL_DERIVE")!=nullptr;
    if(!serialDerive&&!gfy_live::sep::nested()&&parent.columns.size()>=16384&&parent.pitch==g.pitch&&parent.size[0]==g.size.x&&parent.size[1]==g.size.y&&parent.size[2]==g.size.z&&
       parent.origin[0]==g.origin[0]&&parent.origin[1]==g.origin[1]&&parent.origin[2]==g.origin[2]){
        // A crossed cap returns false (the partial scratch is released below, before the serial code runs and refuses where it
        // did); cancellation and every other error propagate unchanged.
        Topology fast;std::vector<std::uint32_t> fastSpans;std::uint64_t fastUnion=0;
        if(detail::deriveSideBySide(parent,g,parentSpans?&fastSpans:nullptr,limits,unionLabel?&fastUnion:nullptr,fast)){
#ifdef GFY_VOXEL_WORKER_TESTS
            static const bool verifyDerive=std::getenv("GFY_DERIVE_VERIFY")!=nullptr,limitTest=std::getenv("GFY_DERIVE_LIMIT_TEST")!=nullptr;
            if(verifyDerive){std::vector<std::uint32_t> serialSpans;std::uint64_t serialUnion=0;
                const auto serial=detail::deriveSerial(parent,g,parentSpans?&serialSpans:nullptr,limits,unionLabel?&serialUnion:nullptr);
                require(detail::sameDerived(serial,fast)&&serialSpans==fastSpans&&serialUnion==fastUnion,"derive_side_by_side_mismatch");
                std::cerr<<"derive_verify ok spans="<<fast.spans.size()<<" components="<<fast.components.size()<<std::endl;}
            if(limitTest)detail::deriveLimitTest(parent,g,limits,fast,fastUnion);
#endif
            if(parentSpans)*parentSpans=std::move(fastSpans);if(unionLabel)*unionLabel=fastUnion;return fast;}
        fast=Topology{};std::vector<std::uint32_t>().swap(fastSpans);
    }
    return detail::deriveSerial(parent,g,parentSpans,limits,unionLabel);
}
namespace detail {
template<class Grid> Topology deriveSerial(const Topology&parent,const Grid&g,std::vector<std::uint32_t>*parentSpans,Limits limits,std::uint64_t*unionLabel){
    Topology out;out.size=parent.size;out.origin=parent.origin;out.pitch=parent.pitch;
#ifdef GFY_RX_PROF
    auto rxT=std::chrono::steady_clock::now();auto rxMark=[&](const char*n){auto now=std::chrono::steady_clock::now();std::cerr<<"rxprof derive."<<n<<' '<<std::chrono::duration<double,std::milli>(now-rxT).count()<<std::endl;rxT=now;};
#else
    auto rxMark=[](const char*){};
#endif
    require(parent.pitch==g.pitch&&parent.size[0]==g.size.x&&parent.size[1]==g.size.y&&parent.size[2]==g.size.z&&
            parent.origin[0]==g.origin[0]&&parent.origin[1]==g.origin[1]&&parent.origin[2]==g.origin[2],"connectivity_derivation_domain");
    auto spend=[&](std::uint64_t n=1){require(n<=limits.work-out.work,"connectivity_work_budget");out.work+=n;};
    std::vector<TileRef> tiles;tiles.reserve(g.tiles.size());
    for(const auto&t:g.tiles){const auto&w=t.second.removed;
        if(t.first.x<0||t.first.y<0||t.first.z<0||!(w[0]|w[1]|w[2]|w[3]|w[4]|w[5]|w[6]|w[7]))continue;
        tiles.push_back({t.first.y,t.first.x,t.first.z,w.data()});}
    std::sort(tiles.begin(),tiles.end(),tileRefLess);
    rxMark("tiles");
    out.columns.reserve(parent.columns.size());out.spans.reserve(parent.spans.size()+parent.spans.size()/8);
    if(parentSpans){parentSpans->clear();parentSpans->reserve(out.spans.capacity());}
    std::vector<Removed> band;std::size_t tileAt=0;std::uint64_t removalRuns=0;
    const auto bandOf=[](int y){return y>=0?y/8:-1-(-1-y)/8;};
    const auto&columns=parent.columns;
    std::size_t nextCancelCheck=0;
    for(std::size_t k=0;k<columns.size();){
        // Cooperative cancel of a separation job (P1.5): only ever aborts.
        if(k>=nextCancelCheck){gfy_live::sep::throwIfCanceled();nextCancelCheck=k+8192;}
        // Parent columns of one band of 8 grid rows, and the band's tiles.
        const int ty=bandOf(columns[k].y);std::size_t e=k+1;while(e<columns.size()&&bandOf(columns[e].y)==ty)++e;
        while(tileAt<tiles.size()&&tiles[tileAt].ty<ty)++tileAt;
        auto bandEnd=tileAt;while(bandEnd<tiles.size()&&tiles[bandEnd].ty==ty)++bandEnd;
        const bool touched=bandEnd>tileAt;
        if(touched)bandRuns(tiles.data()+tileAt,tiles.data()+bandEnd,band);
        tileAt=bandEnd;
        const std::uint64_t firstSpan=columns[k].first,lastSpan=std::uint64_t(columns[e-1].first)+columns[e-1].count;
        bool contiguous=true;for(auto j=k+1;j<e&&contiguous;++j)contiguous=columns[j].first==std::uint64_t(columns[j-1].first)+columns[j-1].count;
        if(!touched&&contiguous){
            // No removed bit in this band: its columns and spans are copied
            // unchanged (the labels are rewritten by labelComponents). Same
            // spans, budget checks and work units as the per-column path.
            const auto n=lastSpan-firstSpan;spend(n);
            require(out.spans.size()+n<=limits.spans&&out.spans.size()+n<=UINT32_MAX,"connectivity_span_budget");
            const auto base=std::uint32_t(out.spans.size());
            out.spans.insert(out.spans.end(),parent.spans.begin()+std::ptrdiff_t(firstSpan),parent.spans.begin()+std::ptrdiff_t(lastSpan));
            if(parentSpans)for(auto i=firstSpan;i<lastSpan;++i)parentSpans->push_back(std::uint32_t(i));
            for(auto j=k;j<e;++j)out.columns.push_back({columns[j].x,columns[j].y,std::uint32_t(base+(columns[j].first-firstSpan)),columns[j].count});
            k=e;continue;
        }
        std::size_t r=0;
        for(;k<e;++k){const auto&c=columns[k];
        if(!touched)band.clear();
        while(r<band.size()&&xyLess(band[r].x,band[r].y,c.x,c.y))++r;
        auto end=r;while(end<band.size()&&band[end].x==c.x&&band[end].y==c.y)++end;
        Column column{c.x,c.y,std::uint32_t(out.spans.size()),0};
        auto append=[&](int lo,int hi,std::uint32_t from){if(lo>=hi)return;
            if(out.spans.size()>column.first&&lo==out.spans.back().hi){out.spans.back().hi=hi;return;}
            require(out.spans.size()<limits.spans&&out.spans.size()<UINT32_MAX,"connectivity_span_budget");
            out.spans.push_back({lo,hi,0});if(parentSpans)parentSpans->push_back(from);};
        if(r==end){spend(c.count);
            for(auto i=c.first;i<c.first+c.count;++i)append(parent.spans[i].lo,parent.spans[i].hi,i);
        }else{auto q=r;std::size_t counted=SIZE_MAX;
            for(auto i=c.first;i<c.first+c.count;++i){spend();int lo=parent.spans[i].lo;const int hi=parent.spans[i].hi;
                while(q<end&&band[q].hi<=lo)++q;
                while(q<end&&band[q].lo<hi){spend();
                    if(counted!=q){counted=q;require(++removalRuns<=limits.removal_runs,"connectivity_removal_budget");}
                    append(lo,std::min(hi,band[q].lo),i);
                    lo=std::max(lo,band[q].hi);if(band[q].hi>=hi)break;++q;}
                append(lo,hi,i);}
            r=end;}
        column.count=std::uint32_t(out.spans.size()-column.first);
        if(column.count)out.columns.push_back(column);
        }
    }
    rxMark("subtract");gfy_live::sep::throwIfCanceled();
    const auto beforeLabel=out.work;
    labelComponents(out,spend);
    if(unionLabel)*unionLabel=out.work-beforeLabel;
    rxMark("label");
#ifdef GFY_RX_PROF
    std::cerr<<"rxprof derive.sizes tiles="<<tiles.size()<<" parent_spans="<<parent.spans.size()<<" spans="<<out.spans.size()<<" removal_runs="<<removalRuns<<" components="<<out.components.size()<<" work="<<out.work<<std::endl;
#endif
    return out;
}
#ifdef GFY_VOXEL_WORKER_TESTS
// Probe evidence (GFY_DERIVE_LIMIT_TEST): the serial and the side-by-side derive of the same input under reduced limits, crossing in
// every phase (subtract, union, label) and exactly at the last unit; both must refuse with the same message or succeed with the
// same topology; the side-by-side work done before its stop is printed with the limit.
template<class Grid> void deriveLimitTest(const Topology&parent,const Grid&g,const Limits&limits,const Topology&full,std::uint64_t unionLabel){
    const std::uint64_t W=full.work,N=full.spans.size(),labelUnits=N,unionSteps=unionLabel>=labelUnits?unionLabel-labelUnits:0,sub=W-unionLabel;
    struct Trial {const char*kind;std::uint64_t value;};std::vector<Trial> trials;
    for(auto v:{sub/2,sub+unionSteps/2,sub+unionSteps+labelUnits/2,W-1,W})trials.push_back({"work",v});
    for(auto v:{N/2,N-1,N})trials.push_back({"spans",v});
    std::uint64_t runs=0;{std::vector<std::uint32_t> ps;Topology t;DeriveAbort a;if(deriveSideBySide(parent,g,&ps,limits,nullptr,t,&a))runs=a.runs;}
    if(runs)for(auto v:{runs/2,runs-1,runs})trials.push_back({"runs",v});
    for(const auto&t:trials){Limits L=limits;
        if(!std::strcmp(t.kind,"work"))L.work=t.value;else if(!std::strcmp(t.kind,"spans"))L.spans=std::size_t(t.value);else L.removal_runs=std::size_t(t.value);
        std::string serialOutcome,fastOutcome;Topology serialTop,fastTop;std::vector<std::uint32_t> sps,fps;std::uint64_t su=0,fu=0;
        try{serialTop=deriveSerial(parent,g,&sps,L,&su);serialOutcome="ok";}catch(const std::exception&e){serialOutcome=e.what();}
        DeriveAbort a;bool fastOk=false;
        try{fastOk=deriveSideBySide(parent,g,&fps,L,&fu,fastTop,&a);fastOutcome=fastOk?"ok":"";}catch(const std::exception&e){fastOutcome=e.what();}
        if(!fastOk&&fastOutcome.empty()){try{fastTop=deriveSerial(parent,g,&fps,L,&fu);fastOutcome="ok";}catch(const std::exception&e){fastOutcome=e.what();}}
        const bool same=serialOutcome==fastOutcome&&(serialOutcome!="ok"||(sameDerived(serialTop,fastTop)&&sps==fps&&su==fu));
        std::cerr<<"derive_limit "<<(same?"ok":"MISMATCH")<<" kind="<<t.kind<<" limit="<<t.value<<" serial="<<serialOutcome<<" side_by_side="<<fastOutcome
                 <<" stopped="<<(a.crossed?a.phase:"no")<<" work_done="<<a.work<<" spans_done="<<a.spans<<" runs_done="<<a.runs<<" union_steps="<<a.unionSteps<<std::endl;
        require(same,"derive_limit_mismatch");}
}
#endif
}
// compare(parent,derived) for derived = derive(parent,g,&parentSpans): the same
// Splits (ascending original, then ascending piece) without column searches.
// A derived component lies inside ONE parent component (derivation only
// removes cells), so all of its cells belong to that family entry.
inline std::vector<std::uint32_t> derivedAncestors(const Topology&before,const Topology&after,const std::vector<std::uint32_t>&parentSpans){
    require(parentSpans.size()==after.spans.size(),"connectivity_derivation_parents");
    std::vector<std::uint32_t> origin(after.components.size(),0);
    for(std::size_t i=0;i<after.spans.size();++i){const auto c=after.spans[i].component;
        require(parentSpans[i]<before.spans.size(),"connectivity_derivation_parents");
        const auto o=before.spans[parentSpans[i]].component;
        require(o&&(!origin[c]||origin[c]==o),"connectivity_material_regrowth");origin[c]=o;}
    return origin;
}
inline std::vector<Split> compareDerived(const Topology&before,const Topology&after,const std::vector<std::uint32_t>&parentSpans){
    require(before.size==after.size&&before.origin==after.origin&&before.pitch==after.pitch,"connectivity_snapshot_domain");
    const auto origin=derivedAncestors(before,after,parentSpans);
    std::vector<std::pair<std::uint32_t,std::uint32_t>> family;family.reserve(after.components.size());
    for(std::uint32_t c=1;c<after.components.size();++c)family.push_back({origin[c],c});
    std::sort(family.begin(),family.end());
    std::vector<Split> result;
    for(std::size_t i=0;i<family.size();){auto j=i;while(j<family.size()&&family[j].first==family[i].first)++j;
        if(j-i>1){Split split;split.original=family[i].first;
            for(auto k=i;k<j;++k)split.pieces.push_back({family[k].second,after.components[family[k].second].cells});
            result.push_back(std::move(split));}
        i=j;}
    return result;
}
// ---- Exact budget acceptance of build(g) (separation-track P1.4) -----------
// derive() returns build(g)'s output whenever build(g) succeeds. To keep base
// FAILURE behaviour too (a grid that build(g) rejects must still be rejected,
// with build(g)'s own message), callers ask referenceBuildAccepts() whether
// build(g) would pass its budgets and, when it would not, call build(g).
// For a grid with anatomy and no ownership, build(g) spends (RemainingConnectivity.h:84-229):
//   64 per tile of g.tiles (all tiles, :86)
// + per tile-row z-run in range with base: 1 (:109) + one per anatomy run iterated (:118, the
//   breaking one included)
// + the column scan (:163, :168, :182): a constant of the anatomy and lattice, equal to the
//   pristine build's work minus its union and label work ("scan work")
// + one per subtraction step (:198) = one per removed piece, since every piece lies inside one
//   base interval and the loop visits each piece once
// + union and label steps (:208, :222) of the output spans, which derive() spends identically.
// The removal-run budget counts removed pieces (:82). The column budget equals the pristine
// build's (same scan) and the span budget applies to the same output spans.
// A cheap upper bound (every z-run of every tile row, pieces bounded by the number of distinct
// anatomy cells its centres sample) settles most grids; otherwise the exact counts are computed.
inline std::uint64_t unionLabelWork(const Topology&t){
    std::uint64_t work=t.spans.size();
    auto steps=[&](const Column&a,const Column*b){if(!b)return;
        auto i=a.first,j=b->first;const auto ie=i+a.count,je=j+b->count;
        while(i<ie&&j<je){++work;const auto&u=t.spans[i];const auto&v=t.spans[j];if(u.hi<v.hi)++i;else ++j;}};
    std::size_t below=0;
    for(std::size_t k=0;k<t.columns.size();++k){const auto&c=t.columns[k];
        const Column*left=k&&t.columns[k-1].y==c.y&&t.columns[k-1].x==c.x-1?&t.columns[k-1]:nullptr;
        while(below<t.columns.size()&&xyLess(t.columns[below].x,t.columns[below].y,c.x,c.y-1))++below;
        const Column*down=below<t.columns.size()&&t.columns[below].x==c.x&&t.columns[below].y==c.y-1?&t.columns[below]:nullptr;
        steps(c,left);steps(c,down);}
    return work;
}
struct ReferenceBudget {bool accepted{},exact{};std::uint64_t work{},pieces{};};
// result = derive(pristine,g) and resultUnionLabelWork its union+label work; pristineScanWork =
// pristine.work - unionLabelWork(pristine). forceExact skips the cheap bound (tests).
template<class Grid> ReferenceBudget referenceBuildBudget(const Grid&g,std::uint64_t pristineScanWork,const Topology&result,
    std::uint64_t resultUnionLabelWork,bool forceExact=false,Limits limits={}){
    require(bool(g.anatomy)&&!g.ownership,"connectivity_reference_budget_grid");
    ReferenceBudget out;
    if(result.spans.size()>limits.spans||result.spans.size()>=UINT32_MAX)return out;
    const auto&a=*g.anatomy;
    const std::uint64_t fixed=64ull*g.tiles.size()+pristineScanWork+resultUnionLabelWork;
    if(fixed>limits.work)return out;
    // Cheap bound: per z-run 1 + (pieces+1) + pieces, pieces <= distinct anatomy samples.
    const double ratio=g.pitch/a.pitch;
    std::array<std::uint32_t,256>boundWork{},boundPieces{};
    for(unsigned b=0;b<256;++b)for(int z=0;z<8;){if(!(b&(1u<<z))){++z;continue;}int lo=z++;while(z<8&&(b&(1u<<z)))++z;
        const auto pieces=std::uint32_t(std::floor((z-lo-1)*ratio+1e-6))+4u;boundWork[b]+=2+2*pieces;boundPieces[b]+=pieces;}
    std::vector<const std::array<std::uint64_t,8>*>tiles;tiles.reserve(g.tiles.size());
    std::vector<std::array<int,3>>keys;keys.reserve(g.tiles.size());
    std::uint64_t upperWork=fixed,upperPieces=0;
    for(const auto&t:g.tiles){tiles.push_back(&t.second.removed);keys.push_back({{t.first.x,t.first.y,t.first.z}});
        for(const auto w:t.second.removed)for(int y=0;y<8;++y){const auto row=unsigned((w>>(y*8))&255u);upperWork+=boundWork[row];upperPieces+=boundPieces[row];}}
    if(!forceExact&&upperWork<=limits.work&&upperPieces<=limits.removal_runs){out.accepted=true;out.work=upperWork;out.pieces=upperPieces;return out;}
    // Exact counts (the collection loop of build(), with table lookups for the lattice maps).
    auto firstAnatomyCell=[&](int boundaryCell,int axis){ // verbatim from build()
        const double boundary=a.origin[axis]+boundaryCell*a.pitch;
        int c=std::clamp(int(std::ceil((boundary-g.origin[axis])/g.pitch-.5)),0,g.size[axis]);
        auto sample=[&](int i){return int(std::floor((g.origin[axis]+(i+.5)*g.pitch-a.origin[axis])/a.pitch));};
        while(c>0&&sample(c-1)>=boundaryCell)--c;
        while(c<g.size[axis]&&sample(c)<boundaryCell)++c;return c;
    };
    std::vector<int>fz(std::size_t(a.size[2])+1),ax(std::size_t(g.size.x)),ay(std::size_t(g.size.y)),az(std::size_t(g.size.z));
    for(std::size_t i=0;i<fz.size();++i)fz[i]=firstAnatomyCell(int(i),2);
    for(int i=0;i<g.size.x;++i)ax[std::size_t(i)]=int(std::floor((g.origin[0]+(i+.5)*g.pitch-a.origin[0])/a.pitch));
    for(int i=0;i<g.size.y;++i)ay[std::size_t(i)]=int(std::floor((g.origin[1]+(i+.5)*g.pitch-a.origin[1])/a.pitch));
    for(int i=0;i<g.size.z;++i)az[std::size_t(i)]=int(std::floor((g.origin[2]+(i+.5)*g.pitch-a.origin[2])/a.pitch));
    std::array<std::uint64_t,4>partWork{},partPieces{};
    gfy_live::sep::parallel(tiles.size(),1024,4,[&](std::size_t begin,std::size_t end,std::size_t chunk){std::uint64_t work=0,pieces=0;
        for(std::size_t n=begin;n<end;++n){const auto&w=*tiles[n];const auto&key=keys[n];
            for(int x=0;x<8;++x)for(int y=0;y<8;++y){unsigned bits=unsigned((w[std::size_t(x)]>>(y*8))&255u);if(!bits)continue;
                const int gx=key[0]*8+x,gy=key[1]*8+y;if(gx<0||gy<0||gx>=g.size.x||gy>=g.size.y)continue;
                const int cx=ax[std::size_t(gx)],cy=ay[std::size_t(gy)];
                if(cx<0||cy<0||cx>=int(a.size[0])||cy>=int(a.size[1]))continue;
                const std::size_t column=std::size_t(cy)*a.size[0]+std::size_t(cx);
                if(a.offsets[column]==a.offsets[column+1])continue;
                const auto first=a.runs.begin()+a.offsets[column],last=a.runs.begin()+a.offsets[column+1];
                for(int z=0;z<8;){if(!(bits&(1u<<z))){++z;continue;}int lo=z++;while(z<8&&(bits&(1u<<z)))++z;
                    ++work;lo+=key[2]*8;const int hi=key[2]*8+z;
                    if(lo<0||lo>=g.size.z){ // outside the lattice: evaluate the sample exactly as build() does
                        const int cell=int(std::floor((g.origin[2]+(lo+.5)*g.pitch-a.origin[2])/a.pitch));
                        auto s=std::lower_bound(first,last,cell,[](const auto&run,int c){return run.hi<=c;});
                        for(;s!=last;++s){++work;const int start=fz[s->lo];if(start>=hi)break;if(std::max(lo,start)<std::min(hi,fz[s->hi]))++pieces;}
                        continue;}
                    auto s=std::lower_bound(first,last,az[std::size_t(lo)],[](const auto&run,int c){return run.hi<=c;});
                    for(;s!=last;++s){++work;const int start=fz[s->lo];if(start>=hi)break;if(std::max(lo,start)<std::min(hi,fz[s->hi]))++pieces;}
                }}}
        partWork[chunk]=work;partPieces[chunk]=pieces;});
    std::uint64_t pieces=0,work=fixed;for(std::size_t c=0;c<4;++c){pieces+=partPieces[c];work+=partWork[c];}
    work+=pieces; // one subtraction step per piece
    out.exact=true;out.work=work;out.pieces=pieces;out.accepted=work<=limits.work&&pieces<=limits.removal_runs;
    return out;
}
// Limits that never trip (derive() under referenceBuildBudget(): base's own budgets decide).
inline Limits unlimitedLimits(){Limits l;l.columns=l.spans=l.removal_runs=std::size_t(1)<<40;l.work=std::uint64_t(1)<<62;return l;}
// Owned grids (g.ownership set; hit_rest, run_candidate): build(g) spends
//   64 per tile + per tile-row z-run in range whose column is an owner column: 1 (:109) and one
//   per owner span overlapping it (:113, each appends one piece)
// + per visited owner column 1 and per owner span 1 (:168, :176), columns with y in [y0,y1)
// + one subtraction step per piece + union and label of the output.
// resultUnionLabelWork may be unknownUnionLabelWork: the cheap check then bounds the
// union and label steps by 5 per result span (each connect() step advances one of
// the two columns' spans; every column is compared with its left and lower
// neighbour) and the scan by the owner's columns plus spans, and the exact value
// (unionLabelWork) is computed only when that bound does not settle acceptance.
constexpr std::uint64_t unknownUnionLabelWork=~std::uint64_t(0);
// ownerExcludesRemoved: the caller guarantees that no owner body span contains a
// removed cell of g, because the owner is a component of build() of a grid with
// g's tiles (ComponentSession make(): the owner comes from build(*accepted) and g
// is *accepted with that owner). build(g) then appends no removal piece: a z-run
// in an owner column costs its one unit (:109) and no span step (:113), so
//   work <= 64 per tile + every tile-row z-run + owner columns + owner spans
//           + union and label (<= 5 per result span, see above),
// and 0 removal runs. When that bound settles acceptance nothing is scanned;
// otherwise the exact pass below runs (it does not use the guarantee).
template<class Grid> ReferenceBudget referenceOwnedBuildBudget(const Grid&g,const Topology&result,std::uint64_t resultUnionLabelWork,
    bool forceExact=false,Limits limits={},bool ownerExcludesRemoved=false){
    require(bool(g.ownership),"connectivity_reference_budget_grid");
    ReferenceBudget out;
    if(result.spans.size()>limits.spans||result.spans.size()>=UINT32_MAX)return out;
    const auto&owner=*g.ownership;
    if(ownerExcludesRemoved&&!forceExact&&owner.body_columns.size()<=limits.columns){
        const std::uint64_t unionLabel=resultUnionLabelWork==unknownUnionLabelWork?5ull*result.spans.size():resultUnionLabelWork;
        const std::uint64_t fixed=64ull*g.tiles.size()+owner.body_columns.size()+owner.body_spans.size()+unionLabel;
        // At most 4 z-runs per tile row (8 cells) without scanning the tiles; else count them.
        if(fixed+256ull*g.tiles.size()<=limits.work){out.accepted=true;out.work=fixed+256ull*g.tiles.size();return out;}
        std::array<std::uint8_t,256>zRuns{};
        for(unsigned b=0;b<256;++b)for(int z=0;z<8;++z)zRuns[b]+=std::uint8_t(((b>>z)&1u)&&(z==0||!((b>>(z-1))&1u)));
        std::uint64_t runs=0;
        for(const auto&t:g.tiles)for(const auto w:t.second.removed)for(int y=0;y<8;++y)runs+=zRuns[unsigned((w>>(y*8))&255u)];
        if(fixed+runs<=limits.work){out.accepted=true;out.work=fixed+runs;return out;}
    }
    // Cheap bound: owner spans never touch, so a z-run of l cells overlaps at most (l+1)/2 of them.
    std::array<std::uint32_t,256>boundWork{},boundPieces{};
    for(unsigned b=0;b<256;++b)for(int z=0;z<8;){if(!(b&(1u<<z))){++z;continue;}int lo=z++;while(z<8&&(b&(1u<<z)))++z;
        const auto pieces=std::uint32_t((z-lo+1)/2);boundWork[b]+=1+2*pieces;boundPieces[b]+=pieces;}
    std::vector<std::pair<const std::array<std::uint64_t,8>*,std::array<int,3>>>tiles;tiles.reserve(g.tiles.size());
    std::uint64_t tileWork=0,upperPieces=0;
    for(const auto&t:g.tiles){tiles.push_back({&t.second.removed,{{t.first.x,t.first.y,t.first.z}}});
        for(const auto w:t.second.removed)for(int y=0;y<8;++y){const auto row=unsigned((w>>(y*8))&255u);tileWork+=boundWork[row];upperPieces+=boundPieces[row];}}
    if(resultUnionLabelWork==unknownUnionLabelWork){
        if(!forceExact&&owner.body_columns.size()<=limits.columns){
            const std::uint64_t upper=64ull*g.tiles.size()+owner.body_columns.size()+owner.body_spans.size()+5ull*result.spans.size()+tileWork;
            if(upper<=limits.work&&upperPieces<=limits.removal_runs){out.accepted=true;out.work=upper;out.pieces=upperPieces;return out;}
        }
        resultUnionLabelWork=unionLabelWork(result);
    }
    int y0=0,y1=g.size.y;
    if(g.anatomy){const auto&a=*g.anatomy;
        auto firstAnatomyCell=[&](int boundaryCell,int axis){ // verbatim from build()
            const double boundary=a.origin[axis]+boundaryCell*a.pitch;
            int c=std::clamp(int(std::ceil((boundary-g.origin[axis])/g.pitch-.5)),0,g.size[axis]);
            auto sample=[&](int i){return int(std::floor((g.origin[axis]+(i+.5)*g.pitch-a.origin[axis])/a.pitch));};
            while(c>0&&sample(c-1)>=boundaryCell)--c;
            while(c<g.size[axis]&&sample(c)<boundaryCell)++c;return c;
        };
        y0=firstAnatomyCell(0,1);y1=firstAnatomyCell(int(a.size[1]),1);}
    std::uint64_t scan=0,visited=0;
    for(const auto&c:owner.body_columns)if(c.y>=y0&&c.y<y1){++visited;scan+=1+c.count;}
    if(visited>limits.columns)return out;
    const std::uint64_t fixed=64ull*g.tiles.size()+scan+resultUnionLabelWork;
    if(fixed>limits.work)return out;
    const std::uint64_t upperWork=fixed+tileWork;
    if(!forceExact&&upperWork<=limits.work&&upperPieces<=limits.removal_runs){out.accepted=true;out.work=upperWork;out.pieces=upperPieces;return out;}
    // Pieces in columns the scan never visits (y outside [y0,y1)) are never subtracted.
    std::array<std::uint64_t,4>partWork{},partPieces{},partSubtracted{};
    gfy_live::sep::parallel(tiles.size(),1024,4,[&](std::size_t begin,std::size_t end,std::size_t chunk){std::uint64_t work=0,pieces=0,subtracted=0;
        for(std::size_t n=begin;n<end;++n){const auto&w=*tiles[n].first;const auto&key=tiles[n].second;
            for(int x=0;x<8;++x)for(int y=0;y<8;++y){unsigned bits=unsigned((w[std::size_t(x)]>>(y*8))&255u);if(!bits)continue;
                const int gx=key[0]*8+x,gy=key[1]*8+y;if(gx<0||gy<0||gx>=g.size.x||gy>=g.size.y)continue;
                const auto&columns=owner.body_columns;
                auto c=std::lower_bound(columns.begin(),columns.end(),std::array<int,2>{{gx,gy}},
                    [](const auto&column,const std::array<int,2>&p){return xyLess(column.x,column.y,p[0],p[1]);});
                if(c==columns.end()||c->x!=gx||c->y!=gy)continue;
                const auto first=owner.body_spans.begin()+c->first,last=first+c->count;
                for(int z=0;z<8;){if(!(bits&(1u<<z))){++z;continue;}int lo=z++;while(z<8&&(bits&(1u<<z)))++z;
                    ++work;lo+=key[2]*8;const int hi=key[2]*8+z;
                    auto s=std::lower_bound(first,last,lo,[](const auto&span,int cell){return span.hi<=cell;});
                    for(;s!=last&&s->lo<hi;++s){++work;if(std::max(lo,s->lo)<std::min(hi,s->hi)){++pieces;subtracted+=gy>=y0&&gy<y1;}}
                }}}
        partWork[chunk]=work;partPieces[chunk]=pieces;partSubtracted[chunk]=subtracted;});
    std::uint64_t pieces=0,work=fixed;for(std::size_t c=0;c<4;++c){pieces+=partPieces[c];work+=partWork[c]+partSubtracted[c];}
    out.exact=true;out.work=work;out.pieces=pieces;out.accepted=work<=limits.work&&pieces<=limits.removal_runs;
    return out;
}
// ---- Exact fast pristine build (separation-track P1.3) ---------------------
// build(g) for a grid with anatomy, no ownership and no tiles (the asset's base
// volume): the same columns, spans, components, cells and the same `work`
// (scan work feeds referenceBuildBudget). The column scan (:154-204) runs in
// bands of rows with table lookups for firstAnatomyCell and the lattice samples
// (same expressions, evaluated once per index); bands are concatenated in row
// order; union and labels are build()'s code. Budgets are checked on the totals;
// any budget build(g) would fail makes build(g) run, so failures are build(g)'s
// own. PristineBuild is the band form (bands may run on any threads, in any
// order: the separation cache's joinable task); buildPristine runs it on <= 4
// threads.
template<class Grid> struct PristineBuild {
 static constexpr int rows=32;
 Grid g; // a copy sharing the immutable anatomy (the task owns what it reads)
 Limits limits;
 bool applicable{};
 std::vector<int>fx,fz,axOf;int y0{},y1{};std::size_t bands{};
 struct Band{std::vector<Column>columns;std::vector<Span>spans;std::uint64_t work{},visited{};};
 std::vector<Band>parts;
 explicit PristineBuild(const Grid&grid,Limits l={}):g(grid),limits(l){
  applicable=bool(g.anatomy)&&!g.ownership&&g.tiles.empty();
  if(!applicable)return;
  require(std::isfinite(g.pitch)&&g.pitch>0,"connectivity_pitch");
  for(int a=0;a<3;++a)require(std::isfinite(g.origin[a])&&g.size[a]>0&&g.size[a]<=65535,"connectivity_domain");
  const auto&a=*g.anatomy;
  fx.resize(std::size_t(a.size[0])+1);fz.resize(std::size_t(a.size[2])+1);axOf.resize(std::size_t(g.size.x));
  for(std::size_t i=0;i<fx.size();++i)fx[i]=firstAnatomyCell(int(i),0);
  for(std::size_t i=0;i<fz.size();++i)fz[i]=firstAnatomyCell(int(i),2);
  for(int x=0;x<g.size.x;++x)axOf[std::size_t(x)]=int(std::floor((g.origin[0]+(x+.5)*g.pitch-a.origin[0])/a.pitch));
  y0=firstAnatomyCell(0,1);y1=firstAnatomyCell(int(a.size[1]),1);
  bands=y1>y0?std::size_t((y1-y0+rows-1)/rows):0;parts.resize(bands);
 }
 int firstAnatomyCell(int boundaryCell,int axis)const{ // verbatim from build()
  const auto&a=*g.anatomy;const double boundary=a.origin[axis]+boundaryCell*a.pitch;
  int c=std::clamp(int(std::ceil((boundary-g.origin[axis])/g.pitch-.5)),0,g.size[axis]);
  auto sample=[&](int i){return int(std::floor((g.origin[axis]+(i+.5)*g.pitch-a.origin[axis])/a.pitch));};
  while(c>0&&sample(c-1)>=boundaryCell)--c;
  while(c<g.size[axis]&&sample(c)<boundaryCell)++c;return c;
 }
 void band(std::size_t b){
  const auto&a=*g.anatomy;auto&part=parts[b];
  std::vector<int>activeXs;std::vector<std::array<int,2>>base;
  const int yb=y0+int(b)*rows,ye=std::min(y1,yb+rows);
  for(int y=yb;y<ye;++y){
   const int ay=int(std::floor((g.origin[1]+(y+.5)*g.pitch-a.origin[1])/a.pitch));
   if(ay<0||ay>=int(a.size[1]))continue;
   part.work+=a.size[0];activeXs.clear();
   for(std::size_t ax=0;ax<a.size[0];++ax){const auto c=std::size_t(ay)*a.size[0]+ax;
    if(a.offsets[c]==a.offsets[c+1])continue;
    for(int x=fx[ax];x<fx[ax+1];++x)activeXs.push_back(x);}
   for(int x:activeXs){++part.work;++part.visited;base.clear();
    const int ax=axOf[std::size_t(x)];
    if(ax<0||ax>=int(a.size[0]))continue;
    const auto column=std::size_t(ay)*a.size[0]+std::size_t(ax);
    for(auto r=a.offsets[column];r<a.offsets[column+1];++r){++part.work;const auto&s=a.runs[r];
     const int lo=fz[s.lo],hi=fz[s.hi];if(lo>=hi)continue;
     if(!base.empty()&&lo<=base.back()[1])base.back()[1]=std::max(base.back()[1],hi);else base.push_back({{lo,hi}});}
    if(base.empty())continue;
    Column col{x,y,std::uint32_t(part.spans.size()),0};
    for(const auto&iv:base){if(part.spans.size()>col.first&&iv[0]==part.spans.back().hi){part.spans.back().hi=iv[1];continue;}
     part.spans.push_back({iv[0],iv[1],0});}
    col.count=std::uint32_t(part.spans.size()-col.first);
    if(col.count)part.columns.push_back(col);}
  }
 }
 Topology finish(std::uint64_t*scanWork=nullptr){
  // Preserve build()'s failure behavior; only publish scan work after success.
  auto fallback=[&](){Topology result=build(g,limits);if(scanWork)*scanWork=result.work-unionLabelWork(result);return result;};
  if(!applicable)return fallback();
  Topology out;out.pitch=g.pitch;
  for(int a=0;a<3;++a){out.origin[a]=g.origin[a];out.size[a]=g.size[a];}
  std::uint64_t work=0,visited=0,spans=0,columns=0;
  for(const auto&p:parts){work+=p.work;visited+=p.visited;spans+=p.spans.size();columns+=p.columns.size();}
  if(visited>limits.columns||spans>limits.spans||spans>=UINT32_MAX||work>limits.work)return fallback();
  out.columns.reserve(columns);out.spans.reserve(spans);
  for(auto&p:parts){const auto offset=std::uint32_t(out.spans.size());
   for(auto c:p.columns){c.first+=offset;out.columns.push_back(c);}
   out.spans.insert(out.spans.end(),p.spans.begin(),p.spans.end());
   std::vector<Column>().swap(p.columns);std::vector<Span>().swap(p.spans);}
  out.work=work;bool over=false;
  detail::labelComponents(out,[&](std::uint64_t n=1){out.work+=n;over=over||out.work>limits.work;});
  if(over)return fallback();
  if(scanWork)*scanWork=work; // exact work before labelComponents, already budget-accepted
  return out;
 }
};
template<class Grid> Topology buildPristine(const Grid&g,Limits limits={}){
 if(!g.anatomy||g.ownership||!g.tiles.empty())return build(g,limits);
 PristineBuild<Grid> job(g,limits);std::atomic<std::size_t>cursor{0};
 gfy_live::sep::parallel(job.bands>=2?4:1,1,4,[&](std::size_t,std::size_t,std::size_t){
  for(std::size_t b;(b=cursor.fetch_add(1))<job.bands;)job.band(b);});
 return job.finish();
}
namespace tiles {
using Mask=std::array<std::uint64_t,8>;
using Key=std::array<int,3>;
constexpr std::uint64_t Z0=0x0101010101010101ull,Z7=Z0<<7;
inline bool empty(const Mask&m){return !(m[0]|m[1]|m[2]|m[3]|m[4]|m[5]|m[6]|m[7]);}
inline bool full(const Mask&m){return (m[0]&m[1]&m[2]&m[3]&m[4]&m[5]&m[6]&m[7])==~std::uint64_t(0);}
inline unsigned popcount64(std::uint64_t v){
#if defined(_MSC_VER)
 return unsigned(__popcnt64(v));
#else
 return unsigned(__builtin_popcountll(v));
#endif
}
inline unsigned popcount(const Mask&m){unsigned n=0;for(auto w:m)n+=popcount64(w);return n;}
// 6-neighbour dilation inside the tile (bits never cross a tile face).
inline Mask dilate(const Mask&m){Mask d;
 for(int x=0;x<8;++x){const std::uint64_t v=m[std::size_t(x)];
  d[std::size_t(x)]=v|((v<<1)&~Z0)|((v>>1)&~Z7)|(v<<8)|(v>>8)|(x?m[std::size_t(x-1)]:0)|(x<7?m[std::size_t(x+1)]:0);}
 return d;}
// Face masks: x faces are the end words (y,z bits); y faces gather the end
// byte of every word (byte x = z bits); z faces gather the end bit of every
// byte (byte x, bit y). Two nodes of adjacent tiles touch when the facing
// masks overlap.
inline std::uint64_t faceX(const Mask&m,bool hi){return m[hi?7:0];}
inline std::uint64_t faceY(const Mask&m,bool hi){std::uint64_t r=0;for(int x=0;x<8;++x)r|=((m[std::size_t(x)]>>(hi?56:0))&0xFFull)<<(8*x);return r;}
inline std::uint64_t faceZ(const Mask&m,bool hi){std::uint64_t r=0;for(int x=0;x<8;++x){const std::uint64_t bits=hi?((m[std::size_t(x)]&Z7)>>7):(m[std::size_t(x)]&Z0);
 // bits contains only bit zero of each byte; unsigned multiplication gathers them in y order.
 const std::uint64_t byte=(bits*0x0102040810204080ull)>>56;r|=byte<<(8*x);}return r;}
struct Faces {std::array<std::uint64_t,6> f;}; // -x,+x,-y,+y,-z,+z
inline Faces faces(const Mask&m){return {{faceX(m,false),faceX(m,true),faceY(m,false),faceY(m,true),faceZ(m,false),faceZ(m,true)}};}
// Appends the 6-connected components of m; returns their count.
inline std::size_t flood(const Mask&m,std::vector<Mask>&out){
 Mask rest=m;std::size_t count=0;
 while(!empty(rest)){Mask c{};int x=0;while(!rest[std::size_t(x)])++x;
  c[std::size_t(x)]=rest[std::size_t(x)]&(~rest[std::size_t(x)]+1); // lowest set bit
  for(;;){Mask n=dilate(c);bool grew=false;for(int w=0;w<8;++w){n[std::size_t(w)]&=rest[std::size_t(w)];grew=grew||n[std::size_t(w)]!=c[std::size_t(w)];}
   c=n;if(!grew)break;}
  for(int w=0;w<8;++w)rest[std::size_t(w)]&=~c[std::size_t(w)];
  out.push_back(c);++count;}
 return count;}
struct Node {Mask mask;Faces face;std::uint32_t tile{},cells{};};
// The pristine tile graph of one (asset, lattice): every tile with solid cells,
// its face neighbours, its nodes (partial tiles only; a full tile is one implicit
// node) and each node's pristine component (first appearance in node order).
struct Base {
 std::array<int,3> size{};std::array<double,3> origin{};double pitch{};
 std::vector<Key> keys;                 // sorted (x,y,z)
 std::vector<std::uint32_t> neighbour;  // 3 per tile: +x,+y,+z tile index or ~0u
 std::vector<std::uint8_t> isFull;      // per tile
 std::vector<std::uint32_t> nodeFirst;  // per tile (+1): nodes of partial tiles
 std::vector<Node> nodes;
 std::vector<std::uint32_t> pristine;   // per node: pristine component id (1-based)
 std::vector<std::uint32_t> pristineOfFull; // per tile: pristine component id of a full tile (0 for partial)
 std::uint32_t pristineComponents{};std::uint64_t cells{};
 std::size_t bytes()const{return keys.size()*(sizeof(Key)+13+4+4)+nodes.size()*sizeof(Node)+pristine.size()*4;}
 // The keys' lexicographic order, compared inline (std::array's < and == go through MSVC's out-of-line vectorized
 // mismatch/equal: a function call per comparison of this hot lookup).
 static bool keyLess(const Key&a,const Key&b)noexcept{return a[0]!=b[0]?a[0]<b[0]:(a[1]!=b[1]?a[1]<b[1]:a[2]<b[2]);}
 std::uint32_t find(const Key&k)const{const auto it=std::lower_bound(keys.begin(),keys.end(),k,keyLess);
  return it!=keys.end()&&(*it)[0]==k[0]&&(*it)[1]==k[1]&&(*it)[2]==k[2]?std::uint32_t(it-keys.begin()):~0u;}
 std::uint32_t nodeCount(std::uint32_t tile)const{return nodeFirst[tile+1]-nodeFirst[tile];}
};
namespace detail {
struct UnionFind {
 std::vector<std::uint32_t> parent;
 explicit UnionFind(std::size_t n):parent(n){for(std::size_t i=0;i<n;++i)parent[i]=std::uint32_t(i);}
 std::uint32_t root(std::uint32_t i){while(parent[i]!=i){parent[i]=parent[parent[i]];i=parent[i];}return i;}
 void join(std::uint32_t a,std::uint32_t b){a=root(a);b=root(b);if(a!=b){if(a<b)parent[b]=a;else parent[a]=b;}}
};
}
// Rasterizes the pristine spans into tiles, floods the partial tiles (up to
// maxThreads threads) and labels the pristine graph.
inline Base buildBase(const Topology&pristine,unsigned maxThreads=4){
 Base base;base.size=pristine.size;base.origin=pristine.origin;base.pitch=pristine.pitch;
 require(pristine.size[0]>0&&pristine.size[1]>0&&pristine.size[2]>0,"tile_base_domain");
 const int nx=(pristine.size[0]+7)/8,ny=(pristine.size[1]+7)/8,nz=(pristine.size[2]+7)/8;
 // Dense tile index over the lattice (temporary), then compact in (x,y,z) order.
 std::vector<std::int32_t> dense(std::size_t(nx)*std::size_t(ny)*std::size_t(nz),-1);
 auto denseAt=[&](int tx,int ty,int tz){return (std::size_t(tx)*std::size_t(ny)+std::size_t(ty))*std::size_t(nz)+std::size_t(tz);};
 std::vector<Mask> solid;
 for(const auto&c:pristine.columns){require(c.x>=0&&c.y>=0&&c.x<pristine.size[0]&&c.y<pristine.size[1],"tile_base_column");
  const int tx=c.x/8,ty=c.y/8,wx=c.x&7,by=c.y&7;
  for(auto i=c.first;i<c.first+c.count;++i){const auto&s=pristine.spans[i];require(s.lo>=0&&s.hi<=pristine.size[2]&&s.lo<s.hi,"tile_base_span");
   for(int tz=s.lo/8;tz<=(s.hi-1)/8;++tz){const int zl=std::max(s.lo,tz*8)-tz*8,zh=std::min(s.hi,tz*8+8)-tz*8;
    auto&slot=dense[denseAt(tx,ty,tz)];if(slot<0){slot=std::int32_t(solid.size());solid.push_back(Mask{});}
    const std::uint64_t bits=((zh-zl)==8?0xFFull:((1ull<<(zh-zl))-1ull)<<zl)<<(8*by);solid[std::size_t(slot)][std::size_t(wx)]|=bits;}}}
 // Compact in lattice order (the dense scan is (x,y,z) ascending).
 std::vector<std::int32_t> remap(solid.size(),-1);base.keys.reserve(solid.size());
 for(int tx=0;tx<nx;++tx)for(int ty=0;ty<ny;++ty)for(int tz=0;tz<nz;++tz){const auto slot=dense[denseAt(tx,ty,tz)];if(slot<0)continue;
  remap[std::size_t(slot)]=std::int32_t(base.keys.size());base.keys.push_back({{tx,ty,tz}});}
 {std::vector<Mask> ordered(solid.size());for(std::size_t i=0;i<solid.size();++i)ordered[std::size_t(remap[i])]=solid[i];solid.swap(ordered);}
 const std::size_t T=base.keys.size();
 base.neighbour.assign(T*3,~0u);base.isFull.assign(T,0);
 for(std::size_t t=0;t<T;++t){const auto&k=base.keys[t];base.isFull[t]=full(solid[t])?1:0;
  for(int d=0;d<3;++d){Key n=k;++n[std::size_t(d)];if(n[std::size_t(d)]>=(d==0?nx:d==1?ny:nz))continue;
   const auto slot=dense[denseAt(n[0],n[1],n[2])];if(slot>=0)base.neighbour[t*3+std::size_t(d)]=std::uint32_t(remap[std::size_t(slot)]);}}
 std::vector<std::int32_t>().swap(dense);
 // Flood the partial tiles: chunks on up to maxThreads threads, concatenated in tile order.
 const std::size_t chunks=gfy_live::sep::chunkCount(T,256,maxThreads);
 std::vector<std::vector<Node>> partNodes(chunks);std::vector<std::vector<std::uint32_t>> partFirst(chunks);
 gfy_live::sep::parallel(T,256,maxThreads,[&](std::size_t first,std::size_t last,std::size_t chunk){
  auto&nodes=partNodes[chunk];auto&starts=partFirst[chunk];starts.reserve(last-first+1);std::vector<Mask> masks;
  for(std::size_t t=first;t<last;++t){starts.push_back(std::uint32_t(nodes.size()));if(base.isFull[t])continue;
   masks.clear();flood(solid[t],masks);
   for(const auto&m:masks){Node n;n.mask=m;n.face=faces(m);n.tile=std::uint32_t(t);n.cells=popcount(m);nodes.push_back(n);}}
  starts.push_back(std::uint32_t(nodes.size()));});
 base.nodeFirst.assign(T+1,0);
 std::size_t totalNodes=0;for(const auto&part:partNodes)totalNodes+=part.size();base.nodes.reserve(totalNodes);
 {std::uint32_t offset=0;std::size_t t=0;
  for(std::size_t c=0;c<chunks;++c){const auto&starts=partFirst[c];for(std::size_t i=0;i+1<starts.size();++i)base.nodeFirst[t++]=offset+starts[i];
   offset+=std::uint32_t(partNodes[c].size());base.nodes.insert(base.nodes.end(),partNodes[c].begin(),partNodes[c].end());}
  base.nodeFirst[T]=offset;require(t==T,"tile_base_chunks");}
 std::vector<Mask>().swap(solid);
 // Pristine labels: ids [0,P) partial nodes, [P,P+T) full tiles.
 const std::uint32_t P=std::uint32_t(base.nodes.size());
 detail::UnionFind sets(P+T);
 auto nodeIds=[&](std::uint32_t t,std::uint32_t&first,std::uint32_t&count,bool&isFull){isFull=base.isFull[t]!=0;if(isFull){first=P+t;count=1;}else{first=base.nodeFirst[t];count=base.nodeCount(t);}};
 for(std::uint32_t t=0;t<T;++t)for(int d=0;d<3;++d){const auto n=base.neighbour[t*3+std::size_t(d)];if(n==~0u)continue;
  std::uint32_t af,ac,bf,bc;bool aFull,bFull;nodeIds(t,af,ac,aFull);nodeIds(n,bf,bc,bFull);
  if(aFull&&bFull){sets.join(af,bf);continue;}
  for(std::uint32_t a=0;a<ac;++a){const std::uint64_t fa=aFull?~std::uint64_t(0):base.nodes[af+a].face.f[std::size_t(2*d+1)];if(!fa)continue;
   for(std::uint32_t b=0;b<bc;++b){const std::uint64_t fb=bFull?~std::uint64_t(0):base.nodes[bf+b].face.f[std::size_t(2*d)];
    if(fa&fb)sets.join(af+a,bf+b);}}}
 base.pristine.assign(P,0);base.pristineOfFull.assign(T,0);std::vector<std::uint32_t> ids(P+T,0);std::uint32_t next=1;
 for(std::uint32_t id=0;id<P+T;++id){const bool isFull=id>=P;const auto t=isFull?id-P:base.nodes[id].tile;
  if(isFull&&!base.isFull[t])continue;const auto r=sets.root(id);if(!ids[r])ids[r]=next++;
  if(isFull)base.pristineOfFull[t]=ids[r];else base.pristine[id]=ids[r];}
 base.pristineComponents=next-1;
 base.cells=0;for(const auto&n:base.nodes)base.cells+=n.cells;for(std::uint32_t t=0;t<T;++t)if(base.isFull[t])base.cells+=512;
 require(base.cells==pristine.cells,"tile_base_cells");
 require(base.pristineComponents+1==pristine.components.size(),"tile_base_components");
 return base;
}
struct Verdict {
 bool separated{};std::uint32_t remaining_components{},largest_substantial_split{},components{},debris_components{};
 std::uint64_t debris_cells{};std::size_t touched_tiles{},touched_nodes{},air_tiles{};
 double ms_touch{},ms_union{},ms_verdict{};
 // Debris to erase (per touched tile: the removed-or-debris mask update): tile key and the cells to erase.
 std::vector<std::pair<Key,Mask>> debris;
 // Verification support: component of any cell (0: air), pristine ancestor per component, cells per component.
 std::vector<std::uint64_t> componentCells;std::vector<std::uint32_t> componentPristine;std::vector<std::uint8_t> componentDebris;
};
// Per-verdict lookup state (kept for GFY_SEP_VERIFY_TILES and the debris erase).
template<class Grid> struct Labeling {
 const Base*base{};const Grid*grid{};
 std::vector<std::uint32_t> touchedTile;      // base tile index of every touched base tile (sorted)
 std::vector<Mask> removed;                   // removed mask of every touched base tile
 std::vector<std::uint32_t> touchedNodeFirst; // CSR into touchedNodes
 std::vector<Node> touchedNodes;
 std::vector<std::uint32_t> component;        // per id (partial nodes, full tiles, touched nodes): component or 0
 std::vector<std::int32_t> tileTouched;       // per base tile: index into the touched arrays or -1
 std::uint32_t P{},T{};
 std::int32_t touchedIndex(std::uint32_t tile)const{return tileTouched[tile];}
 // Component of a lattice cell (0: air or removed).
 std::uint32_t at(int x,int y,int z)const{
  if(x<0||y<0||z<0)return 0;const auto tile=base->find({{x/8,y/8,z/8}});if(tile==~0u)return 0;
  const std::size_t w=std::size_t(x&7);const std::uint64_t bit=1ull<<((y&7)*8+(z&7));
  const auto ti=touchedIndex(tile);
  if(ti>=0){for(auto n=touchedNodeFirst[std::size_t(ti)];n<touchedNodeFirst[std::size_t(ti)+1];++n)if(touchedNodes[n].mask[w]&bit)return component[P+T+n];return 0;}
  if(base->isFull[tile])return component[P+tile];
  for(auto n=base->nodeFirst[tile];n<base->nodeFirst[tile+1];++n)if(base->nodes[n].mask[w]&bit)return component[n];return 0;}
};
// The verdict of `g` against `base` (same lattice, same anatomy). minimumPieceM3: the
// piece minimum; substantial <=> cells*pitch^3 >= minimum, exactly as the span path.
template<class Grid> Verdict verdict(const Base&base,const Grid&g,double minimumPieceM3,Labeling<Grid>*keep=nullptr){
 using Clock=std::chrono::steady_clock;const auto t0=Clock::now();
 Verdict out;Labeling<Grid> local;Labeling<Grid>&L=keep?*keep:local;L.base=&base;L.grid=&g;
 const auto T=std::uint32_t(base.keys.size()),P=std::uint32_t(base.nodes.size());L.P=P;L.T=T;
 // Touched tiles: grid tiles with removed bits inside a base tile (air tiles only feed the cut test).
 struct Air {Key key;Mask removed;};std::vector<Air> air;
 {std::vector<std::pair<std::uint32_t,const Mask*>> touched;touched.reserve(g.tiles.size());
  for(const auto&t:g.tiles){const auto&w=t.second.removed;if(empty(w))continue;
   if(t.first.x<0||t.first.y<0||t.first.z<0)continue;
   const auto tile=base.find({{t.first.x,t.first.y,t.first.z}});
   if(tile==~0u){air.push_back({{{t.first.x,t.first.y,t.first.z}},w});continue;}
   touched.push_back({tile,&w});}
  std::sort(touched.begin(),touched.end(),[](const auto&a,const auto&b){return a.first<b.first;});
  L.touchedTile.reserve(touched.size());L.removed.reserve(touched.size());L.tileTouched.assign(T,-1);
  for(const auto&t:touched){L.tileTouched[t.first]=std::int32_t(L.touchedTile.size());L.touchedTile.push_back(t.first);L.removed.push_back(*t.second);}}
 out.touched_tiles=L.touchedTile.size();out.air_tiles=air.size();
 // Re-flood the touched tiles: remaining = solid & ~removed.
 L.touchedNodeFirst.assign(L.touchedTile.size()+1,0);std::vector<Mask> masks;
 for(std::size_t i=0;i<L.touchedTile.size();++i){L.touchedNodeFirst[i]=std::uint32_t(L.touchedNodes.size());
  const auto tile=L.touchedTile[i];Mask solid;
  if(base.isFull[tile])solid.fill(~std::uint64_t(0));else{solid.fill(0);for(auto n=base.nodeFirst[tile];n<base.nodeFirst[tile+1];++n)for(int w=0;w<8;++w)solid[std::size_t(w)]|=base.nodes[n].mask[std::size_t(w)];}
  Mask remaining;for(int w=0;w<8;++w)remaining[std::size_t(w)]=solid[std::size_t(w)]&~L.removed[i][std::size_t(w)];
  masks.clear();flood(remaining,masks);
  for(const auto&m:masks){Node n;n.mask=m;n.face=faces(m);n.tile=tile;n.cells=popcount(m);L.touchedNodes.push_back(n);}}
 L.touchedNodeFirst[L.touchedTile.size()]=std::uint32_t(L.touchedNodes.size());
 out.touched_nodes=L.touchedNodes.size();
 const auto t1=Clock::now();out.ms_touch=std::chrono::duration<double,std::milli>(t1-t0).count();
 // Union over the tile graph. Ids: [0,P) partial base nodes, [P,P+T) full tiles, [P+T,...) touched nodes.
 const std::uint32_t N=P+T+std::uint32_t(L.touchedNodes.size());
 detail::UnionFind sets(N);
 auto nodeOf=[&](std::uint32_t id)->const Node&{return id<P?base.nodes[id]:L.touchedNodes[id-P-T];};
 auto ids=[&](std::uint32_t t,std::uint32_t&first,std::uint32_t&count,bool&isFull){
  const auto ti=L.touchedIndex(t);
  if(ti>=0){isFull=false;first=P+T+L.touchedNodeFirst[std::size_t(ti)];count=L.touchedNodeFirst[std::size_t(ti)+1]-L.touchedNodeFirst[std::size_t(ti)];return;}
  isFull=base.isFull[t]!=0;if(isFull){first=P+t;count=1;}else{first=base.nodeFirst[t];count=base.nodeCount(t);}};
 for(std::uint32_t t=0;t<T;++t)for(int d=0;d<3;++d){const auto n=base.neighbour[t*3+std::size_t(d)];if(n==~0u)continue;
  std::uint32_t af,ac,bf,bc;bool aFull,bFull;ids(t,af,ac,aFull);ids(n,bf,bc,bFull);
  if(aFull&&bFull){sets.join(af,bf);continue;}
  for(std::uint32_t a=0;a<ac;++a){const std::uint64_t fa=aFull?~std::uint64_t(0):nodeOf(af+a).face.f[std::size_t(2*d+1)];if(!fa)continue;
   for(std::uint32_t b=0;b<bc;++b){const std::uint64_t fb=bFull?~std::uint64_t(0):nodeOf(bf+b).face.f[std::size_t(2*d)];
    if(fa&fb)sets.join(af+a,bf+b);}}}
 const auto t2=Clock::now();out.ms_union=std::chrono::duration<double,std::milli>(t2-t1).count();
 // Components by first appearance; cells; pristine ancestor.
 // All unions are complete; their minimum live id is the root. Its output slot
 // also holds the root label, so no second N-entry root-to-component array is needed.
 L.component.assign(N,0);
 out.componentCells.assign(1,0);out.componentPristine.assign(1,0);
 auto pristineOf=[&](std::uint32_t id)->std::uint32_t{
  if(id<P)return base.pristine[id];if(id<P+T)return base.pristineOfFull[id-P];
  const auto&n=L.touchedNodes[id-P-T];const auto t=n.tile;if(base.isFull[t])return base.pristineOfFull[t];
  for(auto b=base.nodeFirst[t];b<base.nodeFirst[t+1];++b){const auto&bm=base.nodes[b].mask;for(int w=0;w<8;++w)if(bm[std::size_t(w)]&n.mask[std::size_t(w)])return base.pristine[b];}
  return 0;};
 auto live=[&](std::uint32_t id){if(id<P)return L.touchedIndex(base.nodes[id].tile)<0;if(id<P+T)return base.isFull[id-P]!=0&&L.touchedIndex(id-P)<0;return true;};
 for(std::uint32_t id=0;id<N;++id){if(!live(id))continue;const auto r=sets.root(id);auto&c=L.component[r];
  if(!c){c=std::uint32_t(out.componentCells.size());out.componentCells.push_back(0);out.componentPristine.push_back(pristineOf(id));}
  L.component[id]=c;out.componentCells[c]+=id<P||id>=P+T?nodeOf(id).cells:512;}
 out.components=std::uint32_t(out.componentCells.size()-1);
 // Debris: components below the minimum whose cells touch a removed cell (in-tile dilation, or a face
 // against a neighbour's removed face, including neighbours that hold no solid cell at all).
 const double pitch3=base.pitch*base.pitch*base.pitch;
 auto minimal=[&](std::uint64_t cells){return cells*pitch3<minimumPieceM3;};
 auto substantial=[&](std::uint64_t cells){return cells*pitch3>=minimumPieceM3;};
 std::vector<std::uint8_t> touchesCut(out.componentCells.size(),0);
 auto markCut=[&](std::uint32_t id){const auto c=L.component[id];if(c)touchesCut[c]=1;};
 auto removedOf=[&](const Key&k,Mask&m)->bool{ // removed mask of any grid tile by key
  const auto tile=base.find(k);if(tile!=~0u){const auto ti=L.touchedIndex(tile);if(ti<0)return false;m=L.removed[std::size_t(ti)];return true;}
  for(const auto&a:air)if(a.key[0]==k[0]&&a.key[1]==k[1]&&a.key[2]==k[2]){m=a.removed;return true;}return false;};
 static const int delta[6][3]={{-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1}};
 auto cutNeighbours=[&](const Key&k,const Mask&removed,std::uint32_t sourceTile){ // nodes in the six neighbours of a tile with removed cells
  for(int d=0;d<6;++d){
   const std::uint64_t rf=d<2?faceX(removed,d==1):d<4?faceY(removed,d==3):faceZ(removed,d==5); // face of the removed mask towards n
   if(!rf)continue; // a removed mask with no cells on this face cannot touch its neighbour
   std::uint32_t tile;
   if(sourceTile!=~0u&&(d&1))tile=base.neighbour[std::size_t(sourceTile)*3+std::size_t(d/2)];
   else{Key n=k;for(int a=0;a<3;++a)n[std::size_t(a)]+=delta[d][a];tile=base.find(n);}
   if(tile==~0u)continue;
   std::uint32_t f,c;bool isFull;ids(tile,f,c,isFull);
   for(std::uint32_t i=0;i<c;++i){const std::uint64_t nf=isFull?~std::uint64_t(0):nodeOf(f+i).face.f[std::size_t(d^1)];if(rf&nf)markCut(f+i);}}};
 for(std::size_t i=0;i<L.touchedTile.size();++i){const auto tile=L.touchedTile[i];const auto&removed=L.removed[i];
  for(auto n=P+T+L.touchedNodeFirst[i];n<P+T+L.touchedNodeFirst[i+1];++n){const Mask d=dilate(nodeOf(n).mask);bool cut=false;for(int w=0;w<8&&!cut;++w)cut=(d[std::size_t(w)]&removed[std::size_t(w)])!=0;if(cut)markCut(n);}
  cutNeighbours(base.keys[tile],removed,tile);}
 for(const auto&a:air)cutNeighbours(a.key,a.removed,~0u);
 out.componentDebris.assign(out.componentCells.size(),0);
 for(std::uint32_t c=1;c<out.componentCells.size();++c)if(touchesCut[c]&&minimal(out.componentCells[c])){out.componentDebris[c]=1;++out.debris_components;out.debris_cells+=out.componentCells[c];}
 // The debris cells to erase, per tile (any node of a debris component).
 if(out.debris_components){std::vector<std::pair<std::uint32_t,Mask>> perTile;
  for(std::uint32_t id=0;id<N;++id){if(!live(id))continue;const auto c=L.component[id];if(!c||!out.componentDebris[c])continue;
   if(id<P)perTile.push_back({base.nodes[id].tile,base.nodes[id].mask});
   else if(id<P+T){Mask all;all.fill(~std::uint64_t(0));perTile.push_back({id-P,all});}
   else perTile.push_back({L.touchedNodes[id-P-T].tile,L.touchedNodes[id-P-T].mask});}
  std::sort(perTile.begin(),perTile.end(),[](const auto&a,const auto&b){return a.first<b.first;});
  for(std::size_t i=0;i<perTile.size();){Mask m=perTile[i].second;std::size_t j=i+1;
   for(;j<perTile.size()&&perTile[j].first==perTile[i].first;++j)for(int w=0;w<8;++w)m[std::size_t(w)]|=perTile[j].second[std::size_t(w)];
   out.debris.push_back({base.keys[perTile[i].first],m});i=j;}}
 // compare(): per pristine component, the remaining (post-debris) pieces; substantial ones counted.
 std::vector<std::uint32_t> pieces(base.pristineComponents+1,0),substantialPieces(base.pristineComponents+1,0);
 for(std::uint32_t c=1;c<out.componentCells.size();++c){if(out.componentDebris[c])continue;const auto o=out.componentPristine[c];require(o&&o<pieces.size(),"tile_verdict_ancestor");
  ++pieces[o];if(substantial(out.componentCells[c]))++substantialPieces[o];}
 for(std::uint32_t o=1;o<pieces.size();++o)if(pieces[o]>1){out.largest_substantial_split=std::max(out.largest_substantial_split,substantialPieces[o]);if(substantialPieces[o]>=2)out.separated=true;}
 out.remaining_components=out.components-out.debris_components;
 out.ms_verdict=std::chrono::duration<double,std::milli>(Clock::now()-t2).count();
 return out;
}
struct OwnedVerdict {
 bool usable{},separated{};std::uint32_t remaining_components{},largest_substantial_split{},components{},debris_components{};
 std::size_t touched_tiles{},touched_nodes{};double ms_touch{},ms_union{},ms_verdict{};const char*refusal{""};
};
template<class Grid> OwnedVerdict verdictOwned(const Base&base,const Grid&g,double minimumPieceM3,double scrapBelowM3,unsigned maxThreads=4){
 using Clock=std::chrono::steady_clock;const auto t0=Clock::now();OwnedVerdict out;
 const auto T=std::uint32_t(base.keys.size()),P=std::uint32_t(base.nodes.size());
 // Touched tiles: grid tiles whose stored removed bits hit a birth cell of the piece, in base tile order.
 std::vector<std::pair<std::uint32_t,const Mask*>> touched;touched.reserve(g.tiles.size());
 for(const auto&t:g.tiles){const auto&w=t.second.removed;if(empty(w))continue;
  if(t.first.x<0||t.first.y<0||t.first.z<0)continue;
  const auto tile=base.find({{t.first.x,t.first.y,t.first.z}});if(tile==~0u)continue;
  if(!base.isFull[tile]){bool hit=false;
   for(auto n=base.nodeFirst[tile];n<base.nodeFirst[tile+1]&&!hit;++n)for(int k=0;k<8;++k)if(base.nodes[n].mask[std::size_t(k)]&w[std::size_t(k)]){hit=true;break;}
   if(!hit)continue;}
  touched.push_back({tile,&w});}
 std::sort(touched.begin(),touched.end(),[](const auto&a,const auto&b){return a.first<b.first;});
 const std::size_t count=touched.size();out.touched_tiles=count;
 std::vector<std::int32_t> tileTouched(T,-1);for(std::size_t i=0;i<count;++i)tileTouched[touched[i].first]=std::int32_t(i);
 // Re-flood the touched tiles (remaining = birth & ~removed): chunks side by side, concatenated in tile order.
 const std::size_t chunks=gfy_live::sep::chunkCount(count,256,maxThreads);
 std::vector<std::vector<Node>> partNodes(chunks);std::vector<std::vector<std::uint32_t>> partCount(chunks);
 if(count)gfy_live::sep::parallel(count,256,maxThreads,[&](std::size_t first,std::size_t last,std::size_t chunk){
  auto&nodes=partNodes[chunk];auto&counts=partCount[chunk];counts.reserve(last-first);std::vector<Mask> masks;
  for(std::size_t i=first;i<last;++i){const auto tile=touched[i].first;const Mask&removed=*touched[i].second;Mask remaining;
   if(base.isFull[tile])for(int w=0;w<8;++w)remaining[std::size_t(w)]=~removed[std::size_t(w)];
   else{remaining.fill(0);for(auto n=base.nodeFirst[tile];n<base.nodeFirst[tile+1];++n)for(int w=0;w<8;++w)remaining[std::size_t(w)]|=base.nodes[n].mask[std::size_t(w)];
    for(int w=0;w<8;++w)remaining[std::size_t(w)]&=~removed[std::size_t(w)];}
   masks.clear();flood(remaining,masks);counts.push_back(std::uint32_t(masks.size()));
   for(const auto&m:masks){Node node;node.mask=m;node.face=faces(m);node.tile=tile;node.cells=popcount(m);nodes.push_back(node);}}});
 std::vector<std::uint32_t> touchedNodeFirst(count+1,0);std::vector<Node> touchedNodes;
 {std::size_t total=0,i=0;for(const auto&part:partNodes)total+=part.size();touchedNodes.reserve(total);
  for(std::size_t c=0;c<chunks;++c){for(const auto n:partCount[c]){require(i<count,"owned_verdict_chunks");touchedNodeFirst[i+1]=touchedNodeFirst[i]+n;++i;}
   touchedNodes.insert(touchedNodes.end(),partNodes[c].begin(),partNodes[c].end());}
  require(i==count&&touchedNodeFirst[count]==touchedNodes.size(),"owned_verdict_chunks");}
 out.touched_nodes=touchedNodes.size();
 const auto t1=Clock::now();out.ms_touch=std::chrono::duration<double,std::milli>(t1-t0).count();
 // Union over the tile graph, as verdict(). Ids: [0,P) partial base nodes, [P,P+T) full tiles, [P+T,...) touched nodes.
 const std::uint32_t N=P+T+std::uint32_t(touchedNodes.size());
 detail::UnionFind sets(N);
 auto nodeOf=[&](std::uint32_t id)->const Node&{return id<P?base.nodes[id]:touchedNodes[id-P-T];};
 auto ids=[&](std::uint32_t t,std::uint32_t&first,std::uint32_t&number,bool&isFull){
  const auto ti=tileTouched[t];
  if(ti>=0){isFull=false;first=P+T+touchedNodeFirst[std::size_t(ti)];number=touchedNodeFirst[std::size_t(ti)+1]-touchedNodeFirst[std::size_t(ti)];return;}
  isFull=base.isFull[t]!=0;if(isFull){first=P+t;number=1;}else{first=base.nodeFirst[t];number=base.nodeCount(t);}};
 for(std::uint32_t t=0;t<T;++t)for(int d=0;d<3;++d){const auto n=base.neighbour[t*3+std::size_t(d)];if(n==~0u)continue;
  std::uint32_t af,ac,bf,bc;bool aFull,bFull;ids(t,af,ac,aFull);ids(n,bf,bc,bFull);
  if(aFull&&bFull){sets.join(af,bf);continue;}
  for(std::uint32_t a=0;a<ac;++a){const std::uint64_t fa=aFull?~std::uint64_t(0):nodeOf(af+a).face.f[std::size_t(2*d+1)];if(!fa)continue;
   for(std::uint32_t b=0;b<bc;++b){const std::uint64_t fb=bFull?~std::uint64_t(0):nodeOf(bf+b).face.f[std::size_t(2*d)];
    if(fa&fb)sets.join(af+a,bf+b);}}}
 const auto t2=Clock::now();out.ms_union=std::chrono::duration<double,std::milli>(t2-t1).count();
 // Components by first appearance; cells; birth component.
 std::vector<std::uint32_t> component(N,0);std::vector<std::uint64_t> componentCells(1,0);std::vector<std::uint32_t> componentBirth(1,0);
 auto birthOf=[&](std::uint32_t id)->std::uint32_t{
  if(id<P)return base.pristine[id];if(id<P+T)return base.pristineOfFull[id-P];
  const auto&n=touchedNodes[id-P-T];const auto t=n.tile;if(base.isFull[t])return base.pristineOfFull[t];
  for(auto b=base.nodeFirst[t];b<base.nodeFirst[t+1];++b){const auto&bm=base.nodes[b].mask;for(int w=0;w<8;++w)if(bm[std::size_t(w)]&n.mask[std::size_t(w)])return base.pristine[b];}
  return 0;};
 auto live=[&](std::uint32_t id){if(id<P)return tileTouched[base.nodes[id].tile]<0;if(id<P+T)return base.isFull[id-P]!=0&&tileTouched[id-P]<0;return true;};
 for(std::uint32_t id=0;id<N;++id){if(!live(id))continue;const auto root=sets.root(id);auto&c=component[root];
  if(!c){c=std::uint32_t(componentCells.size());componentCells.push_back(0);componentBirth.push_back(birthOf(id));}
  component[id]=c;componentCells[c]+=id<P||id>=P+T?nodeOf(id).cells:512;}
 out.components=std::uint32_t(componentCells.size()-1);
 // The span path's size tests, with its expressions (LiveSeparationWorker_impl.h prepare()).
 const double pitch=g.pitch,cell=pitch*pitch*pitch;
 const auto minimal=[&](std::uint64_t cells){return cells*pitch*pitch*pitch<minimumPieceM3;};
 const auto substantial=[&](std::uint64_t cells){return cells*pitch*pitch*pitch>=minimumPieceM3;};
 std::size_t pieceSized=0;
 for(std::uint32_t c=1;c<componentCells.size();++c){const double m3=double(componentCells[c])*cell;
  if(m3>=minimumPieceM3){++pieceSized;if(m3<scrapBelowM3){out.refusal="scrap_sized_component";return out;}}}
 if(pieceSized>12){out.refusal="more_than_12_pieces";return out;}
 // Debris: a component below the minimum with a cell beside a removed cell, tested cell by cell as the span path does.
 std::vector<std::uint8_t> small(componentCells.size(),0),debris(componentCells.size(),0);bool anySmall=false;
 for(std::uint32_t c=1;c<componentCells.size();++c)if(minimal(componentCells[c])){small[c]=1;anySmall=true;}
 if(anySmall){
  const auto beside=[&](int x,int y,int z){
   static const int delta[6][3]={{-1,0,0},{1,0,0},{0,-1,0},{0,1,0},{0,0,-1},{0,0,1}};
   for(const auto&d:delta){const int qx=x+d[0],qy=y+d[1],qz=z+d[2];if(g.valid({qx,qy,qz})&&g.removed({qx,qy,qz}))return true;}
   return false;};
  for(std::uint32_t id=0;id<N;++id){if(!live(id))continue;const auto c=component[id];if(!c||!small[c]||debris[c])continue;
   const bool whole=id>=P&&id<P+T;const auto tile=whole?id-P:nodeOf(id).tile;const auto&key=base.keys[tile];
   Mask m;if(whole)m.fill(~std::uint64_t(0));else m=nodeOf(id).mask;bool cut=false;
   for(int w=0;w<8&&!cut;++w)for(std::uint64_t bits=m[std::size_t(w)];bits&&!cut;bits&=bits-1){
    const unsigned b=popcount64((bits&(~bits+1))-1);cut=beside(key[0]*8+w,key[1]*8+int(b>>3),key[2]*8+int(b&7));}
   if(cut)debris[c]=1;}}
 // compare(): per birth component, the remaining (post-debris) pieces; substantial ones counted.
 std::vector<std::uint32_t> pieces(base.pristineComponents+1,0),substantialPieces(base.pristineComponents+1,0);
 for(std::uint32_t c=1;c<componentCells.size();++c){if(debris[c]){++out.debris_components;continue;}
  const auto o=componentBirth[c];require(o&&o<pieces.size(),"owned_verdict_ancestor");
  ++pieces[o];if(substantial(componentCells[c]))++substantialPieces[o];}
 for(std::uint32_t o=1;o<pieces.size();++o)if(pieces[o]>1){out.largest_substantial_split=std::max(out.largest_substantial_split,substantialPieces[o]);if(substantialPieces[o]>=2)out.separated=true;}
 out.remaining_components=out.components-out.debris_components;out.usable=true;
 out.ms_verdict=std::chrono::duration<double,std::milli>(Clock::now()-t2).count();
 return out;
}
}
}
