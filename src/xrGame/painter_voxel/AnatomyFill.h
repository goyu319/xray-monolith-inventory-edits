#pragma once
// Anatomy fill: the atlas's connective filler label (1) covers everything between its modelled structures (67 % of a body:
// the lungs are only a bronchial tree, the cerebrum and many muscles are missing, and the suit adds a thick shell), so every
// wound wall showed one filler texture. When the material table carries per-label classes (VXMATS02), the filler is relabelled
// once per asset, per axial slice (x-z plane, columns run along z):
//   skull slices  : filler inside the hull of the cranial vault bones (eroded 3 mm)                -> brain
//   thorax slices : filler inside the hull of ribs/cartilage/sternum/thoracic vertebrae (eroded 10 mm) -> lung, or viscera
//                   when abdominal organs cover >= 12 % of that hull (below the diaphragm); only up to the lung apex
//                   (top of the manubrium + 25 mm)
//   lumbar/pelvis : filler inside the hip-bone hull (eroded 10 mm), and torso filler deeper than 25 mm in front of the
//                   vertebral body                                                                   -> viscera
//   elsewhere     : depth below the outer surface <= 10 mm (4 mm on skull slices) -> subcutaneous fat, deeper -> muscle
// Depth = min(exact 2-D distance to the outside in the slice, 3-D distance on a 4 mm lattice). Deterministic; tables
// without classes (VXMATS01) never reach this code.
// Included inside gfy_live::core after core/native_voxel_probe.cpp (material_volume::Sparse).
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>
namespace anatomy_fill {
enum Class : std::uint8_t {cage=1,spine=2,vault=4,pelvis=8,abdominal=16,manubrium=32};
struct Labels {std::uint16_t fat,muscle,lung,viscera,brain;};
namespace detail {
// Felzenszwalb-Huttenlocher 1-D squared distance transform of a strided float line in place (computed in double;
// 0 = site, 1e12 = none).
inline void dt1(float* f,int n,int stride,std::vector<double>&d,std::vector<int>&v,std::vector<double>&z){
    d.resize(std::size_t(n)*2);v.resize(n);z.resize(std::size_t(n)+1);double* g=d.data()+n;
    for(int q=0;q<n;++q)g[q]=f[std::size_t(q)*stride];
    int k=0;v[0]=0;z[0]=-1e300;z[1]=1e300;
    for(int q=1;q<n;++q){
        double s=((g[q]+double(q)*q)-(g[v[k]]+double(v[k])*v[k]))/(2.0*(q-v[k]));
        while(s<=z[k]){--k;s=((g[q]+double(q)*q)-(g[v[k]]+double(v[k])*v[k]))/(2.0*(q-v[k]));}
        ++k;v[k]=q;z[k]=s;z[k+1]=1e300;}
    k=0;for(int q=0;q<n;++q){while(z[k+1]<q)++k;const double dq=double(q-v[k]);d[q]=dq*dq+g[v[k]];}
    for(int q=0;q<n;++q)f[std::size_t(q)*stride]=float(std::min(d[q],1e12));
}
struct P2 {int x,z;};
inline long long cross(const P2&o,const P2&a,const P2&b){return (long long)(a.x-o.x)*(b.z-o.z)-(long long)(a.z-o.z)*(b.x-o.x);}
// Andrew's monotone chain; counter-clockwise, no collinear points.
inline std::vector<P2> hull(std::vector<P2> p){
    std::sort(p.begin(),p.end(),[](const P2&a,const P2&b){return a.x<b.x||(a.x==b.x&&a.z<b.z);});
    p.erase(std::unique(p.begin(),p.end(),[](const P2&a,const P2&b){return a.x==b.x&&a.z==b.z;}),p.end());
    if(p.size()<3)return {};
    std::vector<P2> h(2*p.size());std::size_t k=0;
    for(std::size_t i=0;i<p.size();++i){while(k>=2&&cross(h[k-2],h[k-1],p[i])<=0)--k;h[k++]=p[i];}
    for(std::size_t i=p.size()-1,t=k+1;i-->0;){while(k>=t&&cross(h[k-2],h[k-1],p[i])<=0)--k;h[k++]=p[i];}
    h.resize(k-1);return h;
}
// Inside the hull shrunk by `erode` cells (signed distance to every edge >= erode).
struct Hull {
    std::vector<std::array<double,3>> e; // a*x+b*z+c >= 0 inside, (a,b) unit
    int x0=0,x1=-1,z0=0,z1=-1;
    bool valid()const{return !e.empty();}
    void build(const std::vector<P2>&pts,double erode){
        e.clear();if(pts.size()<20)return;auto h=hull(pts);if(h.size()<3)return;
        for(std::size_t i=0;i<h.size();++i){const auto&a=h[i];const auto&b=h[(i+1)%h.size()];
            const double dx=b.x-a.x,dz=b.z-a.z,l=std::sqrt(dx*dx+dz*dz);if(l<=0)continue;
            const double nx=-dz/l,nz=dx/l;e.push_back({nx,nz,-(nx*a.x+nz*a.z)-erode});}
        x0=h[0].x;x1=h[0].x;z0=h[0].z;z1=h[0].z;for(const auto&q:h){x0=std::min(x0,q.x);x1=std::max(x1,q.x);z0=std::min(z0,q.z);z1=std::max(z1,q.z);}
    }
    bool inside(int x,int z)const{if(x<x0||x>x1||z<z0||z>z1)return false;for(const auto&q:e)if(q[0]*x+q[1]*z+q[2]<0)return false;return true;}
};
}
// Returns a relabelled copy; `slot` maps label -> slot (filler label 1 -> slot 1), `cls` label -> Class bits.
template<class Parallel>
std::shared_ptr<const material_volume::Sparse> fill(const material_volume::Sparse&a,const std::vector<std::uint16_t>&slot,
    const std::vector<std::uint8_t>&cls,const Labels&out,Parallel&&parallel){
    const int X=int(a.size[0]),Y=int(a.size[1]),Z=int(a.size[2]);const double mm=a.pitch*1000.;
    auto cellClass=[&](std::uint16_t l)->std::uint8_t{return l<cls.size()?cls[l]:0;};
    auto column=[&](int x,int y){const auto c=std::size_t(y)*X+x;return std::make_pair(a.runs.data()+a.offsets[c],a.runs.data()+a.offsets[c+1]);};
    // Lung apex: top of the manubrium + 25 mm.
    int manubriumTop=-1;
    for(int y=Y-1;y>=0&&manubriumTop<0;--y)for(int x=0;x<X;++x){auto r=column(x,y);for(auto*p=r.first;p<r.second;++p)if(cellClass(p->label)&manubrium){manubriumTop=y;break;}if(manubriumTop>=0)break;}
    const int lungTop=manubriumTop>=0?manubriumTop+int(std::lround(25/mm)):std::numeric_limits<int>::max();
    // Coarse 3-D depth: 4 mm lattice, occupancy = each column's outer extent; exact separable squared EDT.
    const int C=std::max(1,int(std::lround(4/mm)));const int CX=(X+C-1)/C,CY=(Y+C-1)/C,CZ=(Z+C-1)/C;
    std::vector<float> d3(std::size_t(CX)*CY*CZ,0.f);const float big=1e12f;
    for(int y=0;y<Y;++y)for(int x=0;x<X;++x){auto r=column(x,y);if(r.first==r.second)continue;
        const int lo=r.first->lo,hi=(r.second-1)->hi;for(int z=lo/C;z<=(hi-1)/C;++z)d3[(std::size_t(z)*CY+y/C)*CX+x/C]=big;}
    // Distance to the nearest OUTSIDE coarse cell: sites are the zero cells.
    parallel(std::size_t(CY)*CZ,[&](std::size_t i0,std::size_t i1){std::vector<double>d;std::vector<int>v;std::vector<double>z;
        for(std::size_t i=i0;i<i1;++i)detail::dt1(d3.data()+i*CX,CX,1,d,v,z);});
    parallel(std::size_t(CX)*CZ,[&](std::size_t i0,std::size_t i1){std::vector<double>d;std::vector<int>v;std::vector<double>z;
        for(std::size_t i=i0;i<i1;++i){const std::size_t x=i%CX,zz=i/CX;detail::dt1(d3.data()+zz*std::size_t(CY)*CX+x,CY,CX,d,v,z);}});
    parallel(std::size_t(CX)*CY,[&](std::size_t i0,std::size_t i1){std::vector<double>d;std::vector<int>v;std::vector<double>z;
        for(std::size_t i=i0;i<i1;++i)detail::dt1(d3.data()+i,CZ,CX*CY,d,v,z);});
    // Per-slice relabel into per-slice run lists.
    std::vector<std::vector<material_volume::Run>> sliceRuns(Y);std::vector<std::vector<std::uint32_t>> sliceCounts(Y);
    parallel(std::size_t(Y),[&](std::size_t y0,std::size_t y1){
        std::vector<std::uint8_t> S,K;std::vector<float> D;
        std::vector<std::uint16_t> L;std::vector<double>d;std::vector<int>v;std::vector<double>zs;
        std::vector<detail::P2> vaultP,cageP,pelvisP;std::vector<int> spineX,spineZ;
        for(std::size_t yy=y0;yy<y1;++yy){const int y=int(yy);
            auto& outRuns=sliceRuns[y];auto& counts=sliceCounts[y];counts.assign(X,0);
            bool anyFiller=false;
            for(int x=0;x<X;++x){auto r=column(x,y);for(auto*p=r.first;p<r.second;++p)if(p->label==1){anyFiller=true;break;}if(anyFiller)break;}
            if(!anyFiller){for(int x=0;x<X;++x){auto r=column(x,y);outRuns.insert(outRuns.end(),r.first,r.second);counts[x]=std::uint32_t(r.second-r.first);}continue;}
            // Work only inside this slice's occupied box (+1 empty border = outside for the distance transform).
            int xa=X,xb=-1,za=Z,zb=-1;
            for(int x=0;x<X;++x){auto r=column(x,y);if(r.first==r.second)continue;xa=std::min(xa,x);xb=std::max(xb,x);za=std::min(za,int(r.first->lo));zb=std::max(zb,int((r.second-1)->hi)-1);}
            const int BX=xb-xa+3,BZ=zb-za+3;auto at=[&](int x,int z){return std::size_t(x-xa+1)*BZ+std::size_t(z-za+1);};
            const std::size_t cells=std::size_t(BX)*BZ;if(S.size()<cells){S.resize(cells);K.resize(cells);D.resize(cells);L.resize(cells);}
            std::fill(S.begin(),S.begin()+cells,0);std::fill(K.begin(),K.begin()+cells,0);
            vaultP.clear();cageP.clear();pelvisP.clear();spineX.clear();spineZ.clear();std::size_t vaultCells=0,cageCells=0,pelvisCells=0;
            for(int x=xa;x<=xb;++x){auto r=column(x,y);for(auto*p=r.first;p<r.second;++p){
                const std::uint8_t s=p->label==1?std::uint8_t(1):std::uint8_t(200),k=cellClass(p->label);
                for(int z=p->lo;z<p->hi;++z){S[at(x,z)]=s;K[at(x,z)]=k;}
                if(k&vault){vaultP.push_back({x,p->lo});vaultP.push_back({x,p->hi-1});vaultCells+=p->hi-p->lo;}
                if(k&cage){cageP.push_back({x,p->lo});cageP.push_back({x,p->hi-1});cageCells+=p->hi-p->lo;}
                if(k&pelvis){pelvisP.push_back({x,p->lo});pelvisP.push_back({x,p->hi-1});pelvisCells+=p->hi-p->lo;}
                if(k&spine)for(int z=p->lo;z<p->hi;++z){spineX.push_back(x);spineZ.push_back(z);}}}
            // Exact 2-D squared distance to the outside (in cells): sites = empty cells.
            for(std::size_t i=0;i<cells;++i)D[i]=S[i]?big:0.f;
            for(int x=0;x<BX;++x)detail::dt1(D.data()+std::size_t(x)*BZ,BZ,1,d,v,zs);
            for(int z=0;z<BZ;++z)detail::dt1(D.data()+z,BX,BZ,d,v,zs);
            const bool headSlice=vaultCells>200;
            detail::Hull hv,hc,hp;
            if(headSlice)hv.build(vaultP,3/mm);
            bool lungSlice=false,abdomenCage=false;
            if(!headSlice&&cageCells>150&&y<=lungTop){hc.build(cageP,10/mm);
                if(hc.valid()){std::size_t in=0,ab=0;for(int x=std::max(xa,hc.x0);x<=std::min(xb,hc.x1);++x)for(int z=std::max(za,hc.z0);z<=std::min(zb,hc.z1);++z)if(hc.inside(x,z)){++in;if(K[at(x,z)]&abdominal)++ab;}
                    abdomenCage=in&&double(ab)/double(in)>=.12;lungSlice=!abdomenCage;}}
            const bool lumbar=!headSlice&&!hc.valid()&&spineX.size()>50&&y<=lungTop;
            if((lumbar||!hc.valid())&&!headSlice&&pelvisCells>150)hp.build(pelvisP,10/mm);
            double spineCX=0;int spineFront=0;if(lumbar){for(int q:spineX)spineCX+=q;spineCX/=spineX.size();spineFront=*std::max_element(spineZ.begin(),spineZ.end());}
            const double fatCells=(headSlice?4:10)/mm,viscDepth=25/mm,torsoHalf=250/mm;
            for(int x=xa;x<=xb;++x)for(int z=za;z<=zb;++z){const std::size_t i=at(x,z);if(S[i]!=1)continue;
                std::uint16_t lab;
                if(headSlice&&hv.valid()&&hv.inside(x,z))lab=out.brain;
                else if(hc.valid()&&hc.inside(x,z))lab=lungSlice?out.lung:out.viscera;
                else {
                    const float c3=d3[(std::size_t(z/C)*CY+y/C)*CX+x/C];const double depth=std::min(double(std::sqrt(D[i])),std::max(0.,std::sqrt(double(c3))*C-C*.5));
                    if(hp.valid()&&hp.inside(x,z))lab=out.viscera;
                    else if(lumbar&&std::abs(x-spineCX)<torsoHalf&&z>spineFront&&depth>viscDepth)lab=out.viscera;
                    else lab=depth<=fatCells?out.fat:out.muscle;}
                L[i]=lab;}
            for(int x=0;x<X;++x){auto r=column(x,y);const std::size_t before=outRuns.size();
                for(auto*p=r.first;p<r.second;++p){
                    if(p->label!=1){outRuns.push_back(*p);continue;}
                    int z=p->lo;while(z<p->hi){const std::uint16_t lab=L[at(x,z)];int e=z+1;while(e<p->hi&&L[at(x,e)]==lab)++e;
                        if(outRuns.size()>before&&outRuns.back().hi==z&&outRuns.back().label==lab)outRuns.back().hi=std::uint16_t(e);
                        else outRuns.push_back({std::uint16_t(z),std::uint16_t(e),lab});z=e;}}
                counts[x]=std::uint32_t(outRuns.size()-before);}
        }});
    auto o=std::make_shared<material_volume::Sparse>();o->size=a.size;o->origin=a.origin;o->pitch=a.pitch;
    std::size_t total=0;for(const auto&s:sliceRuns)total+=s.size();o->runs.reserve(total);o->offsets.assign(a.offsets.size(),0);
    std::size_t c=0;for(int y=0;y<Y;++y){std::size_t at=0;for(int x=0;x<X;++x){o->offsets[c++]=std::uint32_t(o->runs.size());
            const auto n=sliceCounts[y].empty()?0u:sliceCounts[y][x];
            o->runs.insert(o->runs.end(),sliceRuns[y].begin()+at,sliceRuns[y].begin()+at+n);at+=n;}
        std::vector<material_volume::Run>().swap(sliceRuns[y]);}
    o->offsets[c]=std::uint32_t(o->runs.size());
    return o;
}
}
