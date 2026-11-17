#pragma once
// Sever splashes (maskdecal step, 2026-11-12): where the skin of a cut end ENDS, for painter_sever_effects (Lua) to place its
// splashes relative to the skin edge instead of the cut point. Every value of painter_voxel_mask_only (0-3): the exterior of a
// cut end is hidden where its removal field marks the cell (values 0 and 2 and the cut of value 1, which takes today's path) and,
// on a value-3 piece, where one of its decide-once tears hides it (the exterior rule of painter_vmask_tear.h,
// painter_vt_exterior_hidden, with the skin's class: the same index, region, map offset / flags, rip radius and the skin's torn
// depth E; tearfn2's renderer judges the exterior with that class too).
// measure(): the cut bone's skin of one cut end (exterior children: no anatomy / wall / cap material, not the coverage box;
// triangles whose three corners hold at least 0.9 of the bone's weight together; within 1.3 x the limb's radius at the cut of the
// axis), sampled about every 3 mm (barycentric grid) from 5 cm beyond to 25 cm behind the cut along the limb axis; per angular
// sector around the axis the visible sample nearest the cut (the skin edge there). Triangles are visited nearest the cut first and
// a sample is tested only when it is nearer than its sector's edge so far; the walk stops when no triangle left can come nearer.
// Result: the edge points (rest model space, the space of the mesh and the cut) and their distance from the cut point along the
// axis toward this end's own side (the side holding most of its bone's skin).
#include "RuntimePackedMesh.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>
namespace skin_edge {
struct Result {
    std::vector<std::array<float,3>> points; // one per sector that has visible skin, rest model space
    std::vector<float> back;                 // its distance from the cut point toward this end's own side (m)
    unsigned samples=0,tested=0,hidden=0;double ms=0;
};
inline float bits(std::uint32_t w){float f;std::memcpy(&f,&w,4);return f;}
// painter_vt_exterior_hidden (painter_vmask_tear.h) on the CPU, label 1 (the skin's class): every tear the index cell at p lists
// (cutfix layout, TearAtlas.h: cell table + lists, plane table, the piece's depth factor and partner tear)
inline bool tear_hidden(const gfy_packed::TearAtlas& a,const double p[3]){
    if(!a.K||a.params.size()<2*104||!a.mapW||(a.map.empty()&&a.blocks.size()!=a.K))return false;
    auto Wl=[&](std::size_t i)->std::uint32_t{return i<a.params.size()?a.params[i]:0u;};
    auto W=[&](std::size_t row,unsigned texel,unsigned c)->std::uint32_t{return Wl(row*104+std::size_t(texel)*4+c);};
    const double f[4]={bits(W(0,0,0)),bits(W(0,0,1)),bits(W(0,0,2)),bits(W(0,0,3))};
    const std::uint32_t tableRow=W(0,3,0),listRow=W(0,3,1),planeRow=W(0,3,2);
    const std::uint32_t dx=W(1,0,0),dy=W(1,0,1),dz=W(1,0,2),K=W(1,0,3); // row 1: the index dims and K
    if(!(f[3]>0)||!K)return false;
    const double ix=std::floor((p[0]-f[0])/f[3]),iy=std::floor((p[1]-f[1])/f[3]),iz=std::floor((p[2]-f[2])/f[3]);
    if(ix<0||iy<0||iz<0||ix>=double(dx)||iy>=double(dy)||iz>=double(dz))return false;
    const std::uint32_t li=(std::uint32_t(iz)*dy+std::uint32_t(iy))*dx+std::uint32_t(ix);
    const std::uint32_t e=Wl(std::size_t(tableRow)*104+li),first=e>>10,count=e&1023u;
    auto plane=[&](std::uint32_t t,unsigned c){return Wl(std::size_t(planeRow)*104+std::size_t(t)*4+c);};
    struct T {double C[3],u[3],w[3],n[3],a2,b2,h2,ripR,H,Rl,scale;std::uint32_t side,mw,mh,mapY,mapX,partner;const gfy_packed::TearBlock*block;};
    auto load=[&](std::uint32_t id){T t;const std::size_t y=std::size_t(id)+2;
        for(unsigned k=0;k<3;++k){t.C[k]=bits(W(y,0,k));t.u[k]=bits(W(y,1,k));t.w[k]=bits(W(y,2,k));t.n[k]=bits(W(y,3,k));}
        t.side=W(y,4,3);t.mw=W(y,6,0);t.mh=W(y,6,1);t.mapY=W(y,6,3);t.a2=bits(W(y,5,0));t.b2=bits(W(y,5,1));t.h2=bits(W(y,5,2));t.ripR=bits(W(y,5,3));
        t.H=bits(W(y,7,0));t.Rl=bits(W(y,7,1));t.partner=W(y,11,3);t.mapX=plane(2*id+1,2)&65535u;t.scale=double(plane(2*id+1,3)>>16)/65535.;
        t.block=id<a.blocks.size()&&a.blocks[id]?a.blocks[id].get():nullptr;return t;};
    auto mapAt=[&](const T&t,double pa,double pb,unsigned ch)->double{if(!t.mw||!t.mh||!(t.h2>0))return 0;
        const int i=std::min(std::max(int(std::floor((pa-t.a2)/t.h2)),0),int(t.mw)-1),j=std::min(std::max(int(std::floor((pb-t.b2)/t.h2)),0),int(t.mh)-1);
        if(t.block){const std::size_t q=(std::size_t(j)*t.block->map_dim[0]+std::size_t(i))*20+ch;return q<t.block->map.size()?t.block->map[q]:0.;} // by reference
        const std::size_t q=((std::size_t(j)+t.mapY)*a.mapW+t.mapX+std::size_t(i)*5+(ch>>2))*4+(ch&3u);return q<a.map.size()?a.map[q]:0.;};
    // the distance of p from tear j's surface on the side the piece keeps
    auto kept=[&](std::uint32_t j){const T t=load(j);const double d[3]={p[0]-t.C[0],p[1]-t.C[1],p[2]-t.C[2]};
        const double s=d[0]*t.n[0]+d[1]*t.n[1]+d[2]*t.n[2]-mapAt(t,d[0]*t.u[0]+d[1]*t.u[1]+d[2]*t.u[2],d[0]*t.w[0]+d[1]*t.w[1]+d[2]*t.w[2],0);return t.side==1u?s:-s;};
    double m=.004;
    for(std::uint32_t q=0;q<count;++q){
        const std::uint32_t at=first+q,wd=Wl(std::size_t(listRow)*104+at/2),ent=(at&1u)?(wd>>16):(wd&65535u),id=ent&32767u;
        if(ent>=32768u)return true; // the cell lies wholly on that tear's far side
        if(id>=K)continue;
        {const double nC=bits(plane(2*id,3)),n0=bits(plane(2*id,0)),n1=bits(plane(2*id,1)),n2=bits(plane(2*id,2));
         const double sp=n0*p[0]+n1*p[1]+n2*p[2]-nC,sLo=bits(plane(2*id+1,0)),sHi=bits(plane(2*id+1,1));const std::uint32_t sd=(plane(2*id+1,3)>>12)&3u;
         if(!((sp>=sLo&&sp<=sHi)||(sd==0u&&sp>sHi)||(sd==1u&&sp<sLo)))continue;}
        const T t=load(id);
        const double d[3]={p[0]-t.C[0],p[1]-t.C[1],p[2]-t.C[2]};
        const double s0=d[0]*t.n[0]+d[1]*t.n[1]+d[2]*t.n[2],pa=d[0]*t.u[0]+d[1]*t.u[1]+d[2]*t.u[2],pb=d[0]*t.w[0]+d[1]*t.w[1]+d[2]*t.w[2];
        double own=.004; // outside the tear's region: kept
        if(!(std::abs(s0)>t.H+.024||pa*pa+pb*pb>t.Rl*t.Rl)&&(unsigned(mapAt(t,pa,pb,1)+.5)&1u)){
            const double s=s0-mapAt(t,pa,pb,0);const unsigned sd=s>=0?1u:0u;
            double fs=t.scale;
            if(t.partner&&t.side<=1u){const double tk=t.side==1u?s:-s;if(tk>0){const double tj=kept(t.partner-1);
                if(tj>0)fs=std::min(fs,.3*(tk+tj)/std::max(t.ripR+std::max(0.,t.H-.03),1e-4));}}
            const double mm=std::max(0.,std::abs(s)-t.ripR*fs)-mapAt(t,pa,pb,2+sd)*fs; // the skin's torn depth E on that side
            const unsigned st=mm>=0?sd:2u;
            own=(st==t.side||(t.side==2u&&st<2u))?mm:-std::max(std::abs(mm),.0005);
        }
        m=std::min(m,own);
    }
    return m<0;
}
inline bool hidden(const gfy_packed::Mesh& m,const double p[3]){
    if(m.removal_field&&m.removal_field->sample(p[0],p[1],p[2]))return true;
    return m.mask_piece&&m.tear_atlas&&tear_hidden(*m.tear_atlas,p);
}
inline Result measure(const gfy_packed::Mesh& m,std::uint16_t bone,const double cut[3],const double axisIn[3],unsigned sectors=32,unsigned grid=5){
    const auto t0=std::chrono::steady_clock::now();
    Result r;
    const double al=std::sqrt(axisIn[0]*axisIn[0]+axisIn[1]*axisIn[1]+axisIn[2]*axisIn[2]);
    if(!(al>1e-9)||!sectors||!grid)return r;
    const double a[3]={axisIn[0]/al,axisIn[1]/al,axisIn[2]/al};
    double u[3]={std::abs(a[0])<.8?1.:0.,std::abs(a[0])<.8?0.:1.,0.};
    {const double k=u[0]*a[0]+u[1]*a[1]+u[2]*a[2];for(int q=0;q<3;++q)u[q]-=k*a[q];const double l=std::sqrt(u[0]*u[0]+u[1]*u[1]+u[2]*u[2]);for(int q=0;q<3;++q)u[q]/=l;}
    const double w[3]={a[1]*u[2]-a[2]*u[1],a[2]*u[0]-a[0]*u[2],a[0]*u[1]-a[1]*u[0]};
    struct Tri {std::array<double,3> p[3];};
    std::vector<Tri> tris;double sideSum=0;
    for(const auto& c:m.children){
        if(c.material||c.indices.size()<3||c.vertices.empty())continue;
        bool box=true;for(const auto& v:c.vertices)if(!(v.surface[12]< -500.f)){box=false;break;}
        if(box)continue;
        for(std::size_t k=0;k+2<c.indices.size();k+=3){
            Tri t;double wsum=0;bool ok=true;
            for(unsigned q=0;q<3&&ok;++q){
                const std::size_t i=c.indices[k+q];if(i>=c.vertices.size()){ok=false;break;}
                const auto& v=c.vertices[i];t.p[q]={double(v.surface[0]),double(v.surface[1]),double(v.surface[2])};
                for(unsigned s=0;s<4;++s)if(v.bone[s]==bone)wsum+=double(v.weight[s]);
            }
            if(!ok||wsum<.9)continue;
            double tc=0;for(unsigned q=0;q<3;++q)tc+=((t.p[q][0]-cut[0])*a[0]+(t.p[q][1]-cut[1])*a[1]+(t.p[q][2]-cut[2])*a[2])/3;
            if(std::abs(tc)>.3)continue;
            const double e1[3]={t.p[1][0]-t.p[0][0],t.p[1][1]-t.p[0][1],t.p[1][2]-t.p[0][2]},e2[3]={t.p[2][0]-t.p[0][0],t.p[2][1]-t.p[0][1],t.p[2][2]-t.p[0][2]};
            const double cx=e1[1]*e2[2]-e1[2]*e2[1],cy=e1[2]*e2[0]-e1[0]*e2[2],cz=e1[0]*e2[1]-e1[1]*e2[0];
            sideSum+=tc*.5*std::sqrt(cx*cx+cy*cy+cz*cz);
            tris.push_back(t);
        }
    }
    if(tris.empty()||sideSum==0){r.ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();return r;}
    const double sg=sideSum>0?1.:-1.; // this end's own side along the axis
    const double twoPi=6.283185307179586;
    auto backOf=[&](const double* p){return sg*((p[0]-cut[0])*a[0]+(p[1]-cut[1])*a[1]+(p[2]-cut[2])*a[2]);};
    auto radialOf=[&](const double* p){const double d[3]={p[0]-cut[0],p[1]-cut[1],p[2]-cut[2]};
        const double du=d[0]*u[0]+d[1]*u[1]+d[2]*u[2],dw=d[0]*w[0]+d[1]*w[1]+d[2]*w[2];return std::sqrt(du*du+dw*dw);};
    // the limb's own surface: within 1.3 times its radius at the cut (the median distance from the axis of its skin's corners within
    // 2 cm of the cut, the ring the cut went through) - skin of the same bone farther out (groin, buttock) is not where a splash lands
    std::vector<float> ring;struct Order {double lo;std::size_t i;};std::vector<Order> order;order.reserve(tris.size()); // (not 'near': a Windows macro)
    for(std::size_t i=0;i<tris.size();++i){
        double lo=1e9;
        for(unsigned q=0;q<3;++q){const double b=backOf(tris[i].p[q].data());lo=std::min(lo,b);if(std::abs(b)<=.02)ring.push_back(float(radialOf(tris[i].p[q].data())));}
        order.push_back({lo,i});
    }
    float limit=1e9f;
    if(ring.size()>=8){std::nth_element(ring.begin(),ring.begin()+ring.size()/2,ring.end());limit=1.3f*ring[ring.size()/2]+.005f;}
    // triangles nearest the cut first; a sector keeps its visible sample nearest the cut; stop once no triangle left can come nearer
    std::sort(order.begin(),order.end(),[](const Order& x,const Order& y){return x.lo<y.lo;});
    std::vector<float> best(sectors,1e9f);std::vector<std::array<float,3>> at(sectors);float worst=1e9f;
    for(const auto& o:order){
        if(o.lo>.25||o.lo>=worst)break;
        const auto& t=tris[o.i];
        // samples about every 3 mm (the cell is 2 mm): the grid follows the triangle's longest edge (at least 'grid', at most 24)
        double longest=0;for(unsigned q=0;q<3;++q){const auto& x=t.p[q];const auto& y=t.p[(q+1)%3];
            longest=std::max(longest,std::sqrt((x[0]-y[0])*(x[0]-y[0])+(x[1]-y[1])*(x[1]-y[1])+(x[2]-y[2])*(x[2]-y[2])));}
        const unsigned g=std::min(24u,std::max(grid,unsigned(std::ceil(longest/.003))));
        for(unsigned i=0;i<=g;++i)for(unsigned j=0;i+j<=g;++j){
            const double bi=double(i)/g,bj=double(j)/g;double p[3];
            for(unsigned q=0;q<3;++q)p[q]=t.p[0][q]+(t.p[1][q]-t.p[0][q])*bi+(t.p[2][q]-t.p[0][q])*bj;
            const double back=backOf(p);
            if(back< -.05||back>.25)continue;
            if(radialOf(p)>limit)continue;
            const double d[3]={p[0]-cut[0],p[1]-cut[1],p[2]-cut[2]};
            double ang=std::atan2(d[0]*w[0]+d[1]*w[1]+d[2]*w[2],d[0]*u[0]+d[1]*u[1]+d[2]*u[2]);if(ang<0)ang+=twoPi;
            const unsigned sec=std::min(unsigned(ang/twoPi*sectors),sectors-1);
            ++r.samples;
            if(back>=best[sec])continue;
            ++r.tested;
            if(hidden(m,p)){++r.hidden;continue;}
            best[sec]=float(back);at[sec]={float(p[0]),float(p[1]),float(p[2])};
        }
        worst=*std::max_element(best.begin(),best.end());
    }
    for(unsigned k=0;k<sectors;++k)if(best[k]<1e8f){r.points.push_back(at[k]);r.back.push_back(best[k]);}
    r.ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-t0).count();
    return r;
}
inline double median(std::vector<float> v){if(v.empty())return 0;std::sort(v.begin(),v.end());const std::size_t h=v.size()/2;return v.size()%2?v[h]:.5*(double(v[h-1])+v[h]);}
} // namespace skin_edge
