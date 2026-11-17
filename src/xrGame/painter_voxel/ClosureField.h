#pragma once
// Render-only closure material field for analytic channel walls.
//
// The legacy cavity mesher keeps every wall between a removed cell and a kept
// cell that is tissue OR air (e.g. the gap between a garment and the body) when
// it lies inside the CLOSED body envelope, and shades an air-side wall with
// renderMaterial()==1 (FastWound_impl.h wall emission). Channel walls sampling
// only original anatomy therefore opened background holes wherever that legacy
// closure lies in anatomical air. This field has the TissueField layout, so the
// existing painter_anatomy() lookup is unchanged: original anatomy slot where
// nonzero, air_slot for anatomical-air cells whose centres are inside the
// envelope, 0 (discard) outside. Damage/anatomy semantics are untouched; the
// field is never used for receipts, removal or material accounting.
//
// Inside test: identical to BodyBVH::inside() for a preserveCrossings envelope
// (fixed direction .312371,.179319,1; half-open projected-edge rule; t>1e-9;
// parity). Bricks no envelope triangle's bounding box reaches are uniform with
// respect to the envelope and take one test; others test every air cell.
#include "../../Include/xrRender/SparseTissueField.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <bitset>
#include <chrono>
#include <mutex>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <numeric>
#include <thread>
#include <unordered_map>
#include <vector>

namespace painter_detail {
struct EnvelopeParity {
    static constexpr double dx=.312371,dy=.179319,dz=1;
    std::vector<std::array<double,9>> tri;
    double sx=dx/dz,sy=dy/dz,u0{},v0{},cell{};int nu{},nv{};
    std::vector<std::uint32_t> first,items;
    std::array<double,3> lo{1e300,1e300,1e300},hi{-1e300,-1e300,-1e300};
    EnvelopeParity(const std::vector<std::array<float,3>>&positions,const std::vector<std::array<std::uint32_t,3>>&faces){
        tri.reserve(faces.size());
        for(const auto&f:faces){std::array<double,9> t;for(int k=0;k<3;++k)for(int a=0;a<3;++a){t[k*3+a]=double(positions[f[k]][a]);lo[a]=std::min(lo[a],t[k*3+a]);hi[a]=std::max(hi[a],t[k*3+a]);}tri.push_back(t);}
        constexpr double pad=1e-6;double ulo=1e30,uhi=-1e30,vlo=1e30,vhi=-1e30;std::vector<std::array<double,4>> box(tri.size());
        for(std::size_t i=0;i<tri.size();++i){auto&b=box[i];b={1e30,-1e30,1e30,-1e30};
            for(int k=0;k<3;++k){const double*q=&tri[i][k*3];const double u=q[0]-sx*q[2],v=q[1]-sy*q[2];b[0]=std::min(b[0],u);b[1]=std::max(b[1],u);b[2]=std::min(b[2],v);b[3]=std::max(b[3],v);}
            b[0]-=pad;b[1]+=pad;b[2]-=pad;b[3]+=pad;ulo=std::min(ulo,b[0]);uhi=std::max(uhi,b[1]);vlo=std::min(vlo,b[2]);vhi=std::max(vhi,b[3]);}
        if(tri.empty())return;
        cell=std::max({.005,(uhi-ulo)/1024,(vhi-vlo)/1024});u0=ulo;v0=vlo;nu=int((uhi-ulo)/cell)+1;nv=int((vhi-vlo)/cell)+1;
        auto range=[&](double a,double b,double o,int n,int&x,int&y){x=std::clamp(int(std::floor((a-o)/cell)),0,n-1);y=std::clamp(int(std::floor((b-o)/cell)),0,n-1);};
        first.assign(std::size_t(nu)*nv+1,0);
        for(int pass=0;pass<2;++pass){std::vector<std::uint32_t> cursor;
            if(pass){for(std::size_t c=1;c<first.size();++c)first[c]+=first[c-1];items.resize(first.back());cursor.assign(first.begin(),first.end()-1);}
            for(std::uint32_t f=0;f<tri.size();++f){int ua,ub,va,vb;range(box[f][0],box[f][1],u0,nu,ua,ub);range(box[f][2],box[f][3],v0,nv,va,vb);
                for(int u=ua;u<=ub;++u)for(int v=va;v<=vb;++v){const std::size_t c=std::size_t(u)*nv+v;if(pass)items[cursor[c]++]=f;else ++first[c+1];}}}
    }
    // BodyBVH::halfOpenHit with d=(dx,dy,dz): dominant axis z, x=0, y=1.
    bool hit(const double p[3],std::uint32_t face)const{
        const auto&t=tri[face];struct P{double x,y,z;};
        auto project=[&](int k){const double qx=t[k*3]-p[0],qy=t[k*3+1]-p[1],qz=t[k*3+2]-p[2];return P{qx-sx*qz,qy-sy*qz,qz/dz};};
        auto edge=[](P a,P b){return a.x*b.y-a.y*b.x;};
        auto inclusive=[](P a,P b){return b.y>a.y||(b.y==a.y&&b.x<a.x);};
        std::array<P,3> v{project(0),project(1),project(2)};
        double area2=edge(P{v[1].x-v[0].x,v[1].y-v[0].y,0},P{v[2].x-v[0].x,v[2].y-v[0].y,0});if(std::abs(area2)<1e-18)return false;
        if(area2<0){std::swap(v[1],v[2]);area2=-area2;}
        std::array<double,3> e;
        for(int k=0;k<3;++k){e[k]=edge(v[k],v[(k+1)%3]);if(e[k]<0||(e[k]==0&&!inclusive(v[k],v[(k+1)%3])))return false;}
        return (e[1]*v[0].z+e[2]*v[1].z+e[0]*v[2].z)/area2>1e-9;
    }
    bool inside(double x,double y,double z)const{
        if(tri.empty())return false;const double p[3]{x,y,z};
        const double fu=std::floor((x-sx*z-u0)/cell),fv=std::floor((y-sy*z-v0)/cell);
        if(!(fu>=0&&fv>=0&&fu<nu&&fv<nv))return false;
        const std::size_t c=std::size_t(fu)*nv+std::size_t(fv);std::size_t crossings=0;
        for(auto i=first[c];i<first[c+1];++i)crossings+=hit(p,items[i]);
        return crossings%2!=0;
    }
};

struct ClosureStats {std::size_t touched_bricks{},filled_cells{},inside_tests{},changed_bricks{};double build_ms{};};

// domainLo/domainHi: world bounds the field must cover (the damage grid, which the
// worker extends beyond the anatomical lattice to reach loose clothing). The
// field is extended by whole bricks so anatomy bricks map 1:1 (no resampling).
struct ClosureThreads {unsigned width=0;
    void operator()(std::size_t count,const std::function<void(std::size_t,std::size_t)>&body)const{
        const unsigned n=width?width:std::max(1u,std::thread::hardware_concurrency());std::atomic<std::size_t> next{0};
        auto run=[&]{for(std::size_t b;(b=next.fetch_add(64))<count;)body(b,std::min(count,b+64));};
        std::vector<std::thread> pool;for(unsigned t=1;t<n;++t)pool.emplace_back(run);run();for(auto&t:pool)t.join();}};
template<class Parallel=ClosureThreads>
inline std::shared_ptr<const TissueField> buildClosureField(const TissueField&anatomy,const EnvelopeParity&env,std::array<double,3> domainLo,std::array<double,3> domainHi,
    unsigned airSlot=1,ClosureStats*stats=nullptr,Parallel parallel={},std::size_t limit=64u<<20){
    auto out=std::make_shared<TissueField>();
    if(!anatomy.available||airSlot==0||airSlot>63||!(anatomy.pitch>0))return out;
    const auto start=std::chrono::steady_clock::now();
    const double pitch=anatomy.pitch,brick=pitch*8;int k[3];unsigned np[3];std::array<float,3> origin;
    for(int a=0;a<3;++a){const double o=anatomy.origin[a];
        k[a]=std::max(0,int(std::ceil((o-domainLo[a])/brick-1e-9)));
        const double end=o+double(anatomy.pages[a])*brick;const int after=std::max(0,int(std::ceil((domainHi[a]-end)/brick-1e-9)));
        np[a]=anatomy.pages[a]+unsigned(k[a]+after);origin[a]=float(o-double(k[a])*brick);if(np[a]>4096)return out;}
    const unsigned px=np[0],py=np[1],pz=np[2];const std::size_t bricks=std::size_t(px)*py*pz;
    const std::size_t paddedPages=((bricks+TissueField::headerWords+TissueField::pageWidth-1)/TissueField::pageWidth)*TissueField::pageWidth;
    if(paddedPages*4>=limit)return out;
    auto anatomyBrick=[&](unsigned bx,unsigned by,unsigned bz)->std::int64_t{const int ax=int(bx)-k[0],ay=int(by)-k[1],az=int(bz)-k[2];
        if(ax<0||ay<0||az<0||unsigned(ax)>=anatomy.pages[0]||unsigned(ay)>=anatomy.pages[1]||unsigned(az)>=anatomy.pages[2])return -1;
        return (std::int64_t(az)*anatomy.pages[1]+ay)*anatomy.pages[0]+ax;};
    auto label=[&](unsigned x,unsigned y,unsigned z){return std::uint8_t(anatomy.at(int(x)-k[0]*8,int(y)-k[1]*8,int(z)-k[2]*8));};
    // Conservative envelope reach per brick (triangle bounding boxes).
    std::vector<std::uint8_t> touched(bricks,0);
    for(const auto&t:env.tri){int b0[3],b1[3];
        for(int a=0;a<3;++a){const double lo=std::min({t[a],t[3+a],t[6+a]}),hi=std::max({t[a],t[3+a],t[6+a]});
            b0[a]=std::max(int(std::floor((lo-origin[a])/brick)),0);b1[a]=std::min(int(std::floor((hi-origin[a])/brick)),int(np[a])-1);}
        if(b0[0]>b1[0]||b0[1]>b1[1]||b0[2]>b1[2])continue;
        for(int z=b0[2];z<=b1[2];++z)for(int y=b0[1];y<=b1[1];++y)for(int x=b0[0];x<=b1[0];++x)touched[(std::size_t(z)*py+unsigned(y))*px+unsigned(x)]=1;}
    std::vector<std::int32_t> changed(bricks,-1);std::vector<std::array<std::uint8_t,512>> content;std::vector<std::uint8_t> uniformFill(bricks,0);
    std::atomic<std::size_t> filled{0},tests{0};std::mutex merge;
    auto centre=[&](unsigned c,int a){return double(origin[a])+(double(c)+.5)*pitch;};
    auto work=[&](std::size_t first,std::size_t last){std::array<std::uint8_t,512> labels;std::vector<std::pair<std::size_t,std::array<std::uint8_t,512>>> local;std::size_t myFilled=0,myTests=0;
        for(std::size_t i=first;i<last;++i){
            const unsigned bx=unsigned(i%px),by=unsigned((i/px)%py),bz=unsigned(i/(std::size_t(px)*py));
            const auto ai=anatomyBrick(bx,by,bz);const std::uint32_t page=ai>=0?anatomy.page[std::size_t(ai)+TissueField::headerWords]:TissueField::uniformBit;
            if((page&TissueField::uniformBit)&&(page&63u))continue; // all tissue: unchanged
            if(!touched[i]){ // wholly on one side of the envelope
                ++myTests;if(!env.inside(centre(bx*8+4,0),centre(by*8+4,1),centre(bz*8+4,2)))continue;
                if(page&TissueField::uniformBit){uniformFill[i]=1;myFilled+=512;continue;}}
            bool any=false;
            for(unsigned lz=0;lz<8;++lz)for(unsigned ly=0;ly<8;++ly)for(unsigned lx=0;lx<8;++lx){
                const unsigned x=bx*8+lx,y=by*8+ly,z=bz*8+lz;auto&l=labels[(lz*8+ly)*8+lx];l=label(x,y,z);if(l)continue;
                bool in=true;if(touched[i]){++myTests;in=env.inside(centre(x,0),centre(y,1),centre(z,2));}
                if(in){l=std::uint8_t(airSlot);any=true;++myFilled;}}
            if(any)local.emplace_back(i,labels);}
        filled+=myFilled;tests+=myTests;std::lock_guard<std::mutex> g(merge);for(auto&x:local){changed[x.first]=std::int32_t(content.size());content.push_back(x.second);}};
    parallel(bricks,work);
    // Deterministic content order regardless of executor scheduling.
    {std::vector<std::pair<std::size_t,std::int32_t>> order;for(std::size_t i=0;i<bricks;++i)if(changed[i]>=0)order.emplace_back(i,changed[i]);
     std::vector<std::array<std::uint8_t,512>> sorted;sorted.reserve(order.size());for(auto&o:order){changed[o.first]=std::int32_t(sorted.size());sorted.push_back(content[std::size_t(o.second)]);}content.swap(sorted);}
    // Serial deterministic re-encode in page order (layout identical to buildTissueField).
    auto&v=*out;v.origin=origin;v.pitch=anatomy.pitch;for(int a=0;a<3;++a){v.pages[a]=np[a];v.size[a]=np[a]*8;}
    v.page.assign(paddedPages,TissueField::uniformBit);v.data.clear();
    auto encode=[&](const std::uint8_t*first,std::size_t p)->bool{const std::uint8_t value=first[0];
        if(std::all_of(first,first+512,[value](std::uint8_t x){return x==value;})){v.page[p]=TissueField::uniformBit|value;value?++v.uniform:++v.empty;return true;}
        std::array<std::uint8_t,64> present{},index{};unsigned count=0;for(unsigned q=0;q<512;++q)present[first[q]]=1;
        for(unsigned s=0;s<64;++s)if(present[s])index[s]=std::uint8_t(count++);
        const unsigned mode=count<=2?0:count<=4?1:count<=16?2:3,bits=1u<<mode,palette=bits==8?0:1u<<bits;
        const std::size_t bytes=palette+512*bits/8,offset=v.data.size();
        if(v.page.size()*4+offset+bytes+TissueField::dataWidth>limit)return false;
        v.page[p]=(mode<<28)|std::uint32_t(offset);++v.mixed;v.data.resize(offset+bytes,0);
        if(bits==8)std::copy(first,first+512,v.data.begin()+std::ptrdiff_t(offset));
        else{for(unsigned s=0;s<64;++s)if(present[s])v.data[offset+index[s]]=std::uint8_t(s);
            for(unsigned q=0;q<512;++q){const unsigned bit=q*bits;v.data[offset+palette+(bit>>3)]|=std::uint8_t(index[first[q]]<<(bit&7));}}
        return true;};
    std::array<std::uint8_t,512> copy;std::size_t changedBricks=0;
    for(std::size_t i=0;i<bricks;++i){const std::size_t p=i+TissueField::headerWords;
        const unsigned bx=unsigned(i%px),by=unsigned((i/px)%py),bz=unsigned(i/(std::size_t(px)*py));
        const auto ai=anatomyBrick(bx,by,bz);const std::uint32_t page=ai>=0?anatomy.page[std::size_t(ai)+TissueField::headerWords]:TissueField::uniformBit;
        if(uniformFill[i]){v.page[p]=TissueField::uniformBit|airSlot;++v.uniform;++changedBricks;continue;}
        if(changed[i]>=0){++changedBricks;if(!encode(content[std::size_t(changed[i])].data(),p))return std::make_shared<TissueField>();continue;}
        if(page&TissueField::uniformBit){v.page[p]=page;(page&63u)?++v.uniform:++v.empty;continue;}
        for(unsigned lz=0;lz<8;++lz)for(unsigned ly=0;ly<8;++ly)for(unsigned lx=0;lx<8;++lx)copy[(lz*8+ly)*8+lx]=label(bx*8+lx,by*8+ly,bz*8+lz);
        if(!encode(copy.data(),p))return std::make_shared<TissueField>();}
    v.data.resize(std::max<std::size_t>(TissueField::dataWidth,((v.data.size()+TissueField::dataWidth-1)/TissueField::dataWidth)*TissueField::dataWidth),0);
    auto fbits=[](float f){std::uint32_t u;std::memcpy(&u,&f,4);return u;};
    for(int a=0;a<3;++a){v.page[a]=fbits(v.origin[a]);v.page[4+a]=v.size[a];v.page[8+a]=v.pages[a];}
    v.page[3]=anatomy.page[3];v.page[7]=TissueField::brickShift;v.page[11]=TissueField::dataWidth;v.page[12]=TissueField::pageWidth;v.page[13]=TissueField::headerWords;
    v.available=true;
    if(stats){stats->touched_bricks=std::size_t(std::count(touched.begin(),touched.end(),1));stats->filled_cells=filled;stats->inside_tests=tests;stats->changed_bricks=changedBricks;
        stats->build_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();}
    return out;
}

// Fast builder (runtime): identical output layout and domain, but the closed-envelope
// parity is evaluated along lattice columns (+z) instead of per-cell oblique rays:
// every column's envelope crossings are found once (half-open projected edges, the
// same per-triangle rule as halfOpenHit), then each cell's parity is the number of
// crossings strictly above its centre. For a closed envelope this is the same inside
// set; it can differ only where the envelope itself is open. Validated cell-by-cell
// against buildClosureField plus the coverage harness before use.
struct EnvelopeColumns {
    std::vector<std::uint32_t> first; std::vector<float> z; std::vector<std::uint8_t> shell; unsigned nx{},ny{}; std::size_t hits{};
};
// R54: the shipped envelopes are unions of overlapping closed shells (torso with arms, each leg, head, hands). One
// parity over all crossings reads the overlap - neck, shoulders, waist, an original GFY upper arm - as OUTSIDE, and the
// shader discards the cap there (a ring you see through). Each face carries its shell (welded connectivity, 64 bits,
// ids folded beyond 64), each shell keeps its own parity, and a cell is inside when it is inside ANY shell.
inline std::vector<std::uint8_t> envelopeShells(const std::vector<std::array<float,3>>&positions,const std::vector<std::array<std::uint32_t,3>>&faces){
    std::vector<std::uint32_t> weld(positions.size()),parent(positions.size());std::iota(parent.begin(),parent.end(),0u);
    {std::unordered_map<std::uint64_t,std::uint32_t> seen;seen.reserve(positions.size()*2+1);
     auto q=[](float v){return std::uint64_t((std::llround(double(v)*1e5)+(1ll<<20))&0x1FFFFFll);};
     for(std::uint32_t i=0;i<positions.size();++i)weld[i]=seen.emplace((q(positions[i][0])<<42)|(q(positions[i][1])<<21)|q(positions[i][2]),i).first->second;}
    auto find=[&](std::uint32_t x){while(parent[x]!=x){parent[x]=parent[parent[x]];x=parent[x];}return x;};
    for(const auto&f:faces){if(f[0]>=weld.size()||f[1]>=weld.size()||f[2]>=weld.size())continue;
        const auto a=find(weld[f[0]]);auto b=find(weld[f[1]]);if(a!=b)parent[b]=a;b=find(weld[f[2]]);if(a!=b)parent[b]=a;}
    std::unordered_map<std::uint32_t,std::uint8_t> id;std::vector<std::uint8_t> out(faces.size(),0);
    for(std::size_t i=0;i<faces.size();++i){if(faces[i][0]>=weld.size())continue;const auto r=find(weld[faces[i][0]]);
        auto it=id.find(r);if(it==id.end())it=id.emplace(r,std::uint8_t(id.size()%64)).first;out[i]=it->second;}
    return out;
}
template<class Parallel>
inline EnvelopeColumns envelopeColumns(const std::vector<std::array<float,3>>&positions,const std::vector<std::array<std::uint32_t,3>>&faces,
    const std::vector<std::uint8_t>&shellOf,const std::array<float,3>&origin,double pitch,unsigned nx,unsigned ny,Parallel&&parallel){
    EnvelopeColumns c;c.nx=nx;c.ny=ny;const std::size_t columns=std::size_t(nx)*ny;
    std::vector<std::atomic<std::uint32_t>> count(columns);for(auto&a:count)a.store(0,std::memory_order_relaxed);
    // visit(face, column, zHit) for every half-open crossing of that column's line.
    auto scan=[&](std::size_t f,auto&&visit){
        const auto&ids=faces[f];double X[3],Y[3],Z[3];
        for(int k=0;k<3;++k){X[k]=positions[ids[k]][0];Y[k]=positions[ids[k]][1];Z[k]=positions[ids[k]][2];}
        const double lx=std::min({X[0],X[1],X[2]}),hx=std::max({X[0],X[1],X[2]}),ly=std::min({Y[0],Y[1],Y[2]}),hy=std::max({Y[0],Y[1],Y[2]});
        const long i0=std::max(0L,long(std::ceil((lx-origin[0])/pitch-.5))),i1=std::min(long(nx)-1,long(std::floor((hx-origin[0])/pitch-.5)));
        const long j0=std::max(0L,long(std::ceil((ly-origin[1])/pitch-.5))),j1=std::min(long(ny)-1,long(std::floor((hy-origin[1])/pitch-.5)));
        for(long j=j0;j<=j1;++j)for(long i=i0;i<=i1;++i){
            const double px=double(origin[0])+(double(i)+.5)*pitch,py=double(origin[1])+(double(j)+.5)*pitch;
            struct P{double x,y,z;};std::array<P,3> v{P{X[0]-px,Y[0]-py,Z[0]},P{X[1]-px,Y[1]-py,Z[1]},P{X[2]-px,Y[2]-py,Z[2]}};
            auto edge=[](P a,P b){return a.x*b.y-a.y*b.x;};auto inclusive=[](P a,P b){return b.y>a.y||(b.y==a.y&&b.x<a.x);};
            double area2=edge(P{v[1].x-v[0].x,v[1].y-v[0].y,0},P{v[2].x-v[0].x,v[2].y-v[0].y,0});if(std::abs(area2)<1e-18)continue;
            if(area2<0){std::swap(v[1],v[2]);area2=-area2;}
            double e[3];bool in=true;for(int k=0;k<3&&in;++k){e[k]=edge(v[k],v[(k+1)%3]);if(e[k]<0||(e[k]==0&&!inclusive(v[k],v[(k+1)%3])))in=false;}
            if(!in)continue;visit(std::size_t(j)*nx+std::size_t(i),(e[1]*v[0].z+e[2]*v[1].z+e[0]*v[2].z)/area2);
        }};
    parallel(faces.size(),[&](std::size_t a,std::size_t b){for(std::size_t f=a;f<b;++f)scan(f,[&](std::size_t col,double){count[col].fetch_add(1,std::memory_order_relaxed);});});
    c.first.resize(columns+1);c.first[0]=0;for(std::size_t k=0;k<columns;++k)c.first[k+1]=c.first[k]+count[k].load(std::memory_order_relaxed);
    c.hits=c.first[columns];c.z.resize(c.hits);c.shell.resize(c.hits);
    for(std::size_t k=0;k<columns;++k)count[k].store(c.first[k],std::memory_order_relaxed);
    parallel(faces.size(),[&](std::size_t a,std::size_t b){for(std::size_t f=a;f<b;++f)scan(f,[&](std::size_t col,double zHit){
        const auto slot=count[col].fetch_add(1,std::memory_order_relaxed);c.z[slot]=float(zHit);c.shell[slot]=f<shellOf.size()?shellOf[f]:std::uint8_t(0);});});
    parallel(columns,[&](std::size_t a,std::size_t b){std::vector<std::pair<float,std::uint8_t>> hits;
        for(std::size_t k=a;k<b;++k){const auto h0=c.first[k],h1=c.first[k+1];if(h1-h0<2)continue;hits.clear();
            for(auto h=h0;h<h1;++h)hits.emplace_back(c.z[h],c.shell[h]);std::sort(hits.begin(),hits.end());
            for(auto h=h0;h<h1;++h){c.z[h]=hits[h-h0].first;c.shell[h]=hits[h-h0].second;}}});
    return c;
}
// R54 skin shadow: where a garment stands OUTSIDE the envelope (loose sleeves, trouser cuffs, a coat over the hips), the
// cells between that garment and the body it hangs from are closed too, so a cut's cap reaches the cloth instead of
// stopping at the envelope and leaving a see-through slot. From points on every body skin face (1.5 mm apart, 1.5 mm OUTSIDE
// the surface) the fill marches inward along the face's outward normal; the march is kept only if it meets anatomy or
// an envelope shell within 5 cm (cloth hanging farther off, e.g. a coat tail in the air, stays open).
struct SkinShadowInput {const std::vector<std::array<float,3>>*positions=nullptr;const std::vector<std::array<float,3>>*normals=nullptr;
    const std::vector<std::array<std::uint32_t,3>>*faces=nullptr;double reach=.05;};
template<class Parallel=ClosureThreads>
inline std::shared_ptr<const TissueField> buildClosureFieldColumns(const TissueField&anatomy,const std::vector<std::array<float,3>>&positions,
    const std::vector<std::array<std::uint32_t,3>>&faces,std::array<double,3> domainLo,std::array<double,3> domainHi,
    unsigned airSlot=1,ClosureStats*stats=nullptr,Parallel parallel={},std::size_t limit=64u<<20,const SkinShadowInput&skin={}){
    auto out=std::make_shared<TissueField>();
    if(!anatomy.available||airSlot==0||airSlot>63||!(anatomy.pitch>0)||faces.empty())return out;
    const auto start=std::chrono::steady_clock::now();
    const double pitch=anatomy.pitch,brick=pitch*8;int k[3];unsigned np[3];std::array<float,3> origin;
    for(int a=0;a<3;++a){const double o=anatomy.origin[a];
        k[a]=std::max(0,int(std::ceil((o-domainLo[a])/brick-1e-9)));
        const double end=o+double(anatomy.pages[a])*brick;const int after=std::max(0,int(std::ceil((domainHi[a]-end)/brick-1e-9)));
        np[a]=anatomy.pages[a]+unsigned(k[a]+after);origin[a]=float(o-double(k[a])*brick);if(np[a]>4096)return out;}
    const unsigned px=np[0],py=np[1],pz=np[2],nx=px*8,ny=py*8,nz=pz*8;const std::size_t bricks=std::size_t(px)*py*pz;
    const std::size_t paddedPages=((bricks+TissueField::headerWords+TissueField::pageWidth-1)/TissueField::pageWidth)*TissueField::pageWidth;
    if(paddedPages*4>=limit)return out;
    const auto shells=envelopeShells(positions,faces);
    const auto cols=envelopeColumns(positions,faces,shells,origin,pitch,nx,ny,parallel);
    auto anatomyBrick=[&](unsigned bx,unsigned by,unsigned bz)->std::int64_t{const int ax=int(bx)-k[0],ay=int(by)-k[1],az=int(bz)-k[2];
        if(ax<0||ay<0||az<0||unsigned(ax)>=anatomy.pages[0]||unsigned(ay)>=anatomy.pages[1]||unsigned(az)>=anatomy.pages[2])return -1;
        return (std::int64_t(az)*anatomy.pages[1]+ay)*anatomy.pages[0]+ax;};
    auto label=[&](unsigned x,unsigned y,unsigned z){return std::uint8_t(anatomy.at(int(x)-k[0]*8,int(y)-k[1]*8,int(z)-k[2]*8));};
    std::vector<std::int32_t> changed(bricks,-1);std::vector<std::array<std::uint8_t,512>> content;std::vector<std::uint8_t> uniformFill(bricks,0);
    std::atomic<std::size_t> filled{0};std::mutex merge;
    const std::size_t words=(nz+63)/64;
    std::unordered_map<std::size_t,std::array<std::uint64_t,8>> shadow;std::vector<char> shadowColumn(std::size_t(px)*py,0);
    if(skin.positions&&skin.faces&&!skin.faces->empty()){
        auto insideCell=[&](long x,long y,long z)->bool{
            if(x<0||y<0||z<0||x>=long(nx)||y>=long(ny)||z>=long(nz))return false;
            if(label(unsigned(x),unsigned(y),unsigned(z)))return true;
            const std::size_t col=std::size_t(y)*nx+std::size_t(x);const auto h0=cols.first[col],h1=cols.first[col+1];
            const double zc=double(origin[2])+(double(z)+.5)*pitch;std::uint64_t m=0;
            for(auto h=h1;h>h0;--h){if(double(cols.z[h-1])-zc>1e-9)m^=std::uint64_t(1)<<cols.shell[h-1];else break;}
            return m!=0;};
        auto cellOf=[&](const double q[3],long c[3]){for(int a=0;a<3;++a)c[a]=long(std::floor((q[a]-double(origin[a]))/pitch));};
        const int macro=std::max(1,int(std::lround(.002/pitch)));const double step=std::max(pitch,.0015),lip=.0015; // lip: the cap tucks 1.5 mm past the cloth, no seam
        std::vector<std::uint64_t> keys;
        parallel(skin.faces->size(),[&](std::size_t a,std::size_t b){std::vector<std::uint64_t> local,path;
            for(std::size_t i=a;i<b;++i){const auto&f=(*skin.faces)[i];const auto&P=*skin.positions;
                if(f[0]>=P.size()||f[1]>=P.size()||f[2]>=P.size())continue;
                double v[3][3],n[3]={0,0,0};for(int k=0;k<3;++k)for(int c=0;c<3;++c)v[k][c]=P[f[k]][c];
                if(skin.normals&&skin.normals->size()==P.size())for(int k=0;k<3;++k)for(int c=0;c<3;++c)n[c]+=(*skin.normals)[f[k]][c];
                const double len=std::sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);if(!(len>1e-9))continue;for(double&c:n)c/=len;
                bool outside=false;for(int k=0;k<3&&!outside;++k){const double q[3]={v[k][0]+n[0]*lip,v[k][1]+n[1]*lip,v[k][2]+n[2]*lip};long c[3];cellOf(q,c);outside=!insideCell(c[0],c[1],c[2]);}
                if(!outside)continue; // the whole face lies on or inside the closed body already
                double edge=0;for(int k=0;k<3;++k){const double*p=v[k],*q=v[(k+1)%3];edge=std::max(edge,std::sqrt((p[0]-q[0])*(p[0]-q[0])+(p[1]-q[1])*(p[1]-q[1])+(p[2]-q[2])*(p[2]-q[2])));}
                const int steps=std::max(1,int(std::ceil(edge/.0015)));
                for(int s=0;s<=steps;++s)for(int t=0;t<=steps-s;++t){const double u=double(s)/steps,w=double(t)/steps;
                    double q0[3];for(int c=0;c<3;++c)q0[c]=v[0][c]+(v[1][c]-v[0][c])*u+(v[2][c]-v[0][c])*w+n[c]*lip;
                    long c0[3];cellOf(q0,c0);if(insideCell(c0[0],c0[1],c0[2]))continue;
                    path.clear();bool met=false;
                    for(double d=0;d<=skin.reach;d+=step){double q[3];for(int c=0;c<3;++c)q[c]=q0[c]-n[c]*d;long cc[3];cellOf(q,cc);
                        if(cc[0]<0||cc[1]<0||cc[2]<0||cc[0]>=long(nx)||cc[1]>=long(ny)||cc[2]>=long(nz))break;
                        if(insideCell(cc[0],cc[1],cc[2])){met=true;break;}
                        path.push_back((std::uint64_t(cc[0]/macro)<<42)|(std::uint64_t(cc[1]/macro)<<21)|std::uint64_t(cc[2]/macro));}
                    if(met)local.insert(local.end(),path.begin(),path.end());}}
            std::sort(local.begin(),local.end());local.erase(std::unique(local.begin(),local.end()),local.end());
            std::lock_guard<std::mutex> g(merge);keys.insert(keys.end(),local.begin(),local.end());});
        std::sort(keys.begin(),keys.end());keys.erase(std::unique(keys.begin(),keys.end()),keys.end());
        for(const auto key:keys){const long mx=long(key>>42),my=long((key>>21)&0x1FFFFF),mz=long(key&0x1FFFFF);
            for(int dz=0;dz<macro;++dz)for(int dy=0;dy<macro;++dy)for(int dx=0;dx<macro;++dx){const long x=mx*macro+dx,y=my*macro+dy,z=mz*macro+dz;
                if(x>=long(nx)||y>=long(ny)||z>=long(nz))continue;const std::size_t i=(std::size_t(z>>3)*py+std::size_t(y>>3))*px+std::size_t(x>>3);
                const unsigned bit=unsigned(((z&7)*8+(y&7))*8+(x&7));shadow[i][bit>>6]|=std::uint64_t(1)<<(bit&63);shadowColumn[std::size_t(y>>3)*px+std::size_t(x>>3)]=1;}}
    }
    // One brick column (bx,by) at a time: 64 cell columns x full height of inside bits.
    parallel(std::size_t(px)*py,[&](std::size_t a,std::size_t b){
        std::vector<std::uint64_t> bits(64*words);std::array<std::uint8_t,512> labels;std::vector<std::pair<std::size_t,std::array<std::uint8_t,512>>> local;std::size_t myFilled=0;
        for(std::size_t bc=a;bc<b;++bc){const unsigned bx=unsigned(bc%px),by=unsigned(bc/px);bool any=false;
            std::fill(bits.begin(),bits.end(),0);
            for(unsigned ly=0;ly<8;++ly)for(unsigned lx=0;lx<8;++lx){const std::size_t col=std::size_t(by*8+ly)*nx+(bx*8+lx);
                const auto h0=cols.first[col],h1=cols.first[col+1];if(h0==h1)continue;any=true;
                auto*row=&bits[(ly*8+lx)*words];std::int64_t h=std::int64_t(h1)-1;std::uint64_t inside=0;
                for(std::int64_t z=std::int64_t(nz)-1;z>=0;--z){const double zc=double(origin[2])+(double(z)+.5)*pitch;
                    while(h>=std::int64_t(h0)&&double(cols.z[std::size_t(h)])-zc>1e-9){inside^=std::uint64_t(1)<<cols.shell[std::size_t(h)];--h;}
                    if(inside)row[std::size_t(z)>>6]|=std::uint64_t(1)<<(z&63);}}
            if(!any&&!shadowColumn[bc])continue; // no crossing and no skin shadow in this brick column: all outside
            for(unsigned bz=0;bz<pz;++bz){const std::size_t i=(std::size_t(bz)*py+by)*px+bx;
                const auto ai=anatomyBrick(bx,by,bz);const std::uint32_t page=ai>=0?anatomy.page[std::size_t(ai)+TissueField::headerWords]:TissueField::uniformBit;
                if((page&TissueField::uniformBit)&&(page&63u))continue;
                unsigned inside=0;for(unsigned c=0;c<64;++c)inside+=unsigned(std::bitset<8>(unsigned((bits[c*words+((bz*8)>>6)]>>((bz*8)&63))&0xFFu)).count());
                const auto found=shadow.find(i);const std::array<std::uint64_t,8>*sm=found==shadow.end()?nullptr:&found->second;
                if(!inside&&!sm)continue;
                if((page&TissueField::uniformBit)&&inside==512){uniformFill[i]=1;myFilled+=512;continue;}
                bool fill=false;
                for(unsigned lz=0;lz<8;++lz)for(unsigned ly=0;ly<8;++ly)for(unsigned lx=0;lx<8;++lx){const unsigned q=(lz*8+ly)*8+lx;
                    auto&l=labels[q];l=label(bx*8+lx,by*8+ly,bz*8+lz);if(l)continue;
                    const unsigned z=bz*8+lz;if(((bits[(ly*8+lx)*words+(z>>6)]>>(z&63))&1u)||(sm&&(((*sm)[q>>6]>>(q&63))&1u))){l=std::uint8_t(airSlot);fill=true;++myFilled;}}
                if(fill)local.emplace_back(i,labels);}}
        filled+=myFilled;std::lock_guard<std::mutex> g(merge);for(auto&x:local){changed[x.first]=std::int32_t(content.size());content.push_back(x.second);}});
    {std::vector<std::pair<std::size_t,std::int32_t>> order;for(std::size_t i=0;i<bricks;++i)if(changed[i]>=0)order.emplace_back(i,changed[i]);
     std::vector<std::array<std::uint8_t,512>> sorted;sorted.reserve(order.size());for(auto&o:order){changed[o.first]=std::int32_t(sorted.size());sorted.push_back(content[std::size_t(o.second)]);}content.swap(sorted);}
    auto&v=*out;v.origin=origin;v.pitch=anatomy.pitch;for(int a=0;a<3;++a){v.pages[a]=np[a];v.size[a]=np[a]*8;}
    v.page.assign(paddedPages,TissueField::uniformBit);v.data.clear();
    auto encode=[&](const std::uint8_t*first,std::size_t p)->bool{const std::uint8_t value=first[0];
        if(std::all_of(first,first+512,[value](std::uint8_t x){return x==value;})){v.page[p]=TissueField::uniformBit|value;value?++v.uniform:++v.empty;return true;}
        std::array<std::uint8_t,64> present{},index{};unsigned count=0;for(unsigned q=0;q<512;++q)present[first[q]]=1;
        for(unsigned s=0;s<64;++s)if(present[s])index[s]=std::uint8_t(count++);
        const unsigned mode=count<=2?0:count<=4?1:count<=16?2:3,bits=1u<<mode,palette=bits==8?0:1u<<bits;
        const std::size_t bytes=palette+512*bits/8,offset=v.data.size();
        if(v.page.size()*4+offset+bytes+TissueField::dataWidth>limit)return false;
        v.page[p]=(mode<<28)|std::uint32_t(offset);++v.mixed;v.data.resize(offset+bytes,0);
        if(bits==8)std::copy(first,first+512,v.data.begin()+std::ptrdiff_t(offset));
        else{for(unsigned s=0;s<64;++s)if(present[s])v.data[offset+index[s]]=std::uint8_t(s);
            for(unsigned q=0;q<512;++q){const unsigned bit=q*bits;v.data[offset+palette+(bit>>3)]|=std::uint8_t(index[first[q]]<<(bit&7));}}
        return true;};
    std::array<std::uint8_t,512> copy;std::size_t changedBricks=0;
    for(std::size_t i=0;i<bricks;++i){const std::size_t p=i+TissueField::headerWords;
        const unsigned bx=unsigned(i%px),by=unsigned((i/px)%py),bz=unsigned(i/(std::size_t(px)*py));
        const auto ai=anatomyBrick(bx,by,bz);const std::uint32_t page=ai>=0?anatomy.page[std::size_t(ai)+TissueField::headerWords]:TissueField::uniformBit;
        if(uniformFill[i]){v.page[p]=TissueField::uniformBit|airSlot;++v.uniform;++changedBricks;continue;}
        if(changed[i]>=0){++changedBricks;if(!encode(content[std::size_t(changed[i])].data(),p))return std::make_shared<TissueField>();continue;}
        if(page&TissueField::uniformBit){v.page[p]=page;(page&63u)?++v.uniform:++v.empty;continue;}
        for(unsigned lz=0;lz<8;++lz)for(unsigned ly=0;ly<8;++ly)for(unsigned lx=0;lx<8;++lx)copy[(lz*8+ly)*8+lx]=label(bx*8+lx,by*8+ly,bz*8+lz);
        if(!encode(copy.data(),p))return std::make_shared<TissueField>();}
    v.data.resize(std::max<std::size_t>(TissueField::dataWidth,((v.data.size()+TissueField::dataWidth-1)/TissueField::dataWidth)*TissueField::dataWidth),0);
    auto fbits=[](float f){std::uint32_t u;std::memcpy(&u,&f,4);return u;};
    for(int a=0;a<3;++a){v.page[a]=fbits(v.origin[a]);v.page[4+a]=v.size[a];v.page[8+a]=v.pages[a];}
    v.page[3]=anatomy.page[3];v.page[7]=TissueField::brickShift;v.page[11]=TissueField::dataWidth;v.page[12]=TissueField::pageWidth;v.page[13]=TissueField::headerWords;
    v.available=true;
    if(stats){stats->touched_bricks=0;stats->filled_cells=filled;stats->inside_tests=cols.hits;stats->changed_bricks=changedBricks;
        stats->build_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();}
    return out;
}
} // namespace painter_detail
