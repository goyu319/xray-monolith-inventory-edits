#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include "BloodCellLayer.h"

namespace PainterStaticBlood {
namespace chart {
constexpr unsigned MaxReceivers=384,MaxMarkVertices=1536,MaxPending=1024;
struct Point {double x=0,y=0;};
struct Box {
    double a=1.e100,b=1.e100,c=-1.e100,d=-1.e100;
    void grow(Point p){a=std::min(a,p.x);b=std::min(b,p.y);c=std::max(c,p.x);d=std::max(d,p.y);}
    void add(const Box& q){grow({q.a,q.b});grow({q.c,q.d});}
};
inline V3 sub(V3 a,V3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
inline double dot(V3 a,V3 b){return double(a.x)*b.x+double(a.y)*b.y+double(a.z)*b.z;}
inline V3 cross(V3 a,V3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline V3 unit(V3 a){double k=std::sqrt(dot(a,a));if(k<1e-12)return {};return {float(a.x/k),float(a.y/k),float(a.z/k)};}
inline bool finite(V3 p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
inline Point project(const Frame& f,V3 p){const V3 q=sub(p,f.origin);return {dot(q,f.u),dot(q,f.v)};}
inline double edge(Point a,Point b,Point p){return (b.x-a.x)*(p.y-a.y)-(b.y-a.y)*(p.x-a.x);}
inline bool before(V3 a,V3 b){return a.x!=b.x?a.x<b.x:(a.y!=b.y?a.y<b.y:a.z<b.z);}
inline std::array<float,9> key(const Receiver& t){
    std::array<V3,3> p={t.p[0],t.p[1],t.p[2]};std::sort(p.begin(),p.end(),before);
    return {p[0].x,p[0].y,p[0].z,p[1].x,p[1].y,p[1].z,p[2].x,p[2].y,p[2].z};
}
inline bool valid(const Mark& m){
    if(!m.id||!m.image||!m.image->width||!m.image->height||m.clipped.empty()||
       m.clipped.size()%3||m.clipped.size()>MaxMarkVertices||m.receivers.empty()||m.receivers.size()>MaxReceivers||
       m.image->pixels.size()!=std::size_t(m.image->width)*m.image->height||!m.stamp.lifetimeMs||
       !std::isfinite(m.stamp.opacity)||m.stamp.opacity<=0||m.stamp.opacity>1.f)return false;
    for(const auto& v:m.clipped)if(!finite(v.p)||!std::isfinite(v.u)||!std::isfinite(v.v))return false;
    for(const auto& t:m.receivers)for(auto v:t.p)if(!finite(v))return false;
    return true;
}
inline bool seedFrame(const Mark& m,Frame& f) {
    double best=0;V3 n={};
    for(std::size_t i=0;i<m.clipped.size();i+=3){
        const auto a=m.clipped[i].p,b=m.clipped[i+1].p,c=m.clipped[i+2].p;
        V3 q=cross(sub(b,a),sub(c,a));double area=dot(q,q);
        if(area>best){best=area;n=q;f.origin=a;}
    }
    if(best<1e-19)return false;
    f.n=unit(n);f.u=unit(cross(std::abs(f.n.y)<.8f?V3{0,1,0}:V3{1,0,0},f.n));f.v=unit(cross(f.n,f.u));
    return true;
}
struct Fit {Box box;double density=0;};
inline bool fit(const Mark& m,const Frame& f,const Config& c,Fit& v){
    for(const auto& x:m.clipped){
        if(std::fabs(dot(sub(x.p,f.origin),f.n))>c.planeTolerance)return false;
        v.box.grow(project(f,x.p));
    }
    for(std::size_t i=0;i<m.clipped.size();i+=3){
        const Vertex* p=m.clipped.data()+i;
        V3 norm=cross(sub(p[1].p,p[0].p),sub(p[2].p,p[0].p));
        double mag=std::sqrt(dot(norm,norm));if(mag<=1e-10)continue;
        if(dot(norm,f.n)<mag*c.normalCos)return false;
        Point a=project(f,p[0].p),b=project(f,p[1].p),q=project(f,p[2].p);
        double det=edge(a,b,q);if(std::fabs(det)<1e-12)continue;
        double du1=(p[1].u-p[0].u)*m.image->width,du2=(p[2].u-p[0].u)*m.image->width;
        double dv1=(p[1].v-p[0].v)*m.image->height,dv2=(p[2].v-p[0].v)*m.image->height;
        double dx1=b.x-a.x,dy1=b.y-a.y,dx2=q.x-a.x,dy2=q.y-a.y;
        double ux=(du1*dy2-du2*dy1)/det,uy=(du2*dx1-du1*dx2)/det;
        double vx=(dv1*dy2-dv2*dy1)/det,vy=(dv2*dx1-dv1*dx2)/det;
        double xx=ux*ux+vx*vx,yy=uy*uy+vy*vy,xy=ux*uy+vx*vy;
        double largest=.5*(xx+yy+std::hypot(xx-yy,2.*xy));
        v.density=std::max(v.density,std::sqrt(largest));
    }
    return std::isfinite(v.density)&&v.density>0;
}
inline double distance(const Box& a,const Box& b){
    return std::hypot(std::max({0.,b.a-a.c,a.a-b.c}),std::max({0.,b.b-a.d,a.b-b.d}));
}
inline bool extent(Box b,double density,const Config& c,Frame& frame,unsigned& w,unsigned& h){
    const double pitch=1./density;
    // A margin allows later neighbouring splats to use this same image.
    const double pad=std::max(2*pitch,.08*std::max(b.c-b.a,b.d-b.b));
    frame.left=std::floor((b.a-pad)/pitch)*pitch;
    frame.bottom=std::floor((b.b-pad)/pitch)*pitch;frame.pitch=pitch;
    double dw=std::ceil((b.c+pad-frame.left)/pitch),dh=std::ceil((b.d+pad-frame.bottom)/pitch);
    if(!(dw>0&&dh>0)||dw>c.maxDimension||dh>c.maxDimension||dw*dh*6+512*sizeof(Stamp)>c.byteCap)return false;
    w=(unsigned)dw;h=(unsigned)dh;return true;
}
struct SurfacePoint {Point q;V3 p;};
using Polygon=std::vector<SurfacePoint>;
template<class SignedDistance> inline Polygon clip(const Polygon& in,SignedDistance side){
    Polygon out;if(in.empty())return out;out.reserve(in.size()+1);
    SurfacePoint a=in.back();double sa=side(a.q);
    for(const auto& b:in){
        double sb=side(b.q);
        if((sb>=0)!=(sa>=0)){
            double t=sa/(sa-sb);
            out.push_back({{a.q.x+t*(b.q.x-a.q.x),a.q.y+t*(b.q.y-a.q.y)},
                           {float(a.p.x+t*(b.p.x-a.p.x)),float(a.p.y+t*(b.p.y-a.p.y)),float(a.p.z+t*(b.p.z-a.p.z))}});
        }
        if(sb>=0)out.push_back(b);a=b;sa=sb;
    }
    return out;
}
inline double area(const Polygon& poly){
    double sum=0;if(poly.size()>2)for(std::size_t i=1;i+1<poly.size();++i)sum+=edge(poly[0].q,poly[i].q,poly[i+1].q);
    return .5*sum;
}
inline bool mesh(Patch& p,const std::vector<const Mark*>& group,const Config& cfg){
    const Frame& f=p.frame;const double maxX=f.left+p.width*f.pitch,maxY=f.bottom+p.height*f.pitch;
    std::set<std::array<float,9>> unique;
    struct Tile{Polygon poly;Box b;};
    std::vector<Tile> tiles;
    for(auto m:group)for(const auto& tri:m->receivers){
        if(!unique.insert(key(tri)).second)continue;
        if(unique.size()>MaxReceivers)return false;
        Polygon q;for(const auto& v:tri.p)q.push_back({project(f,v),v});
        q=clip(q,[&](Point a){return a.x-f.left;});q=clip(q,[&](Point a){return maxX-a.x;});
        q=clip(q,[&](Point a){return a.y-f.bottom;});q=clip(q,[&](Point a){return maxY-a.y;});
        double ownArea=area(q);if(std::abs(ownArea)<=1e-11)continue;
        if(ownArea<0)return false;
        Box bounds;for(const auto& v:q){if(std::abs(dot(sub(v.p,f.origin),f.n))>cfg.planeTolerance)return false;bounds.grow(v.q);}
        for(const auto& t:tiles){
            if(bounds.c<=t.b.a||t.b.c<=bounds.a||bounds.d<=t.b.b||t.b.d<=bounds.b)continue;
            Polygon intersect=q;
            for(std::size_t i=0;i<t.poly.size()&&!intersect.empty();++i){
                Point a=t.poly[i].q,b=t.poly[(i+1)%t.poly.size()].q;
                intersect=clip(intersect,[&](Point v){return edge(a,b,v);});
            }
            if(std::abs(area(intersect))>1e-10)return false;
        }
        const auto vertex=[&](const SurfacePoint& v){return Vertex{v.p,~0u,float((v.q.x-f.left)/(p.width*f.pitch)),float((v.q.y-f.bottom)/(p.height*f.pitch))};};
        for(std::size_t i=1;i+1<q.size();i++){
            if(edge(q[0].q,q[i].q,q[i+1].q)<=1e-12)continue;
            p.mesh.push_back(vertex(q[0]));p.mesh.push_back(vertex(q[i]));p.mesh.push_back(vertex(q[i+1]));
        }
        p.receivers.push_back(tri);tiles.push_back({std::move(q),bounds});
    }
    return !p.mesh.empty()&&p.textureBytes()<=cfg.byteCap;
}
inline float fade(const Stamp& s,std::uint32_t now){
    float elapsed=s.lifetimeMs==~0u?0.f:std::min(1.f,float(now-s.landedMs)/float(s.lifetimeMs));
    return 1.f-std::floor(BloodCellLayer::sat(1.f-s.opacity*(1.f-elapsed))*255.f)/255.f;
}
inline std::uint32_t byte(float x){return std::uint32_t(BloodCellLayer::sat(x)*255.f+.5f);}
inline unsigned layer(std::uint16_t code){return (code>>9)&3u;}
inline std::uint32_t composite(std::uint32_t old,std::uint16_t tag,const std::vector<Stamp>& stamps,
       const Stamp& s,const PainterBloodTextureCodec::Pixel& tex,unsigned& dstLayer){
    using namespace BloodCellLayer;
    float r=sat(tex[0]),g=sat(tex[1]),b=sat(tex[2]),a=sat(tex[3]*1.7f-.7f);
    unsigned id=tag&511u;dstLayer=0;
    if(id){
        float fNew=std::max(fade(s,s.landedMs),1.f/255.f),eNew=a*fNew;
        float coverOld=float(old>>24)/255.f;
        float eOld=coverOld*fade(stamps[id],s.landedMs),count=LayerCounts[layer(tag)];
        float through=std::pow(1.f-eOld,count)*(1.f-eNew),sum=count*eOld+eNew;
        float err=1e9f,best=0;
        for(unsigned k=0;k<4;k++){float one=1.f-std::pow(through,1.f/LayerCounts[k]);
            float e=std::fabs(LayerCounts[k]*one-sum);if(e<err){err=e;dstLayer=k;best=one;}}
        a=sat(best/fNew);float covered=1.f-through;
        if(covered>0){float w=eNew/covered,z=1.f-w;r=r*w+((old&255u)/255.f)*z;g=g*w+(((old>>8)&255u)/255.f)*z;b=b*w+(((old>>16)&255u)/255.f)*z;}
    }
    return byte(r)|(byte(g)<<8)|(byte(b)<<16)|(byte(a)<<24);
}
// Each clipped input triangle must be completely covered by the published non-overlapping collision footprint.
inline bool covered(const Patch& p,const Mark& m,double* incomingArea=nullptr){
    const auto& f=p.frame;double total=0;
    for(std::size_t t=0;t<m.clipped.size();t+=3){
        Polygon source;Box bounds;
        for(unsigned j=0;j<3;j++){auto v=m.clipped[t+j].p;Point xy=project(f,v);source.push_back({xy,v});bounds.grow(xy);}
        const double own=std::abs(area(source));if(own<=1e-11)continue;double matched=0;
        for(std::size_t j=0;j<p.mesh.size();j+=3){
            Point q[3];Box b;for(unsigned k=0;k<3;k++){q[k]=project(f,p.mesh[j+k].p);b.grow(q[k]);}
            if(bounds.c<=b.a||b.c<=bounds.a||bounds.d<=b.b||b.d<=bounds.b)continue;
            Polygon piece=source;
            for(unsigned k=0;k<3&&!piece.empty();k++){
                const Point a=q[k],z=q[(k+1)%3];piece=clip(piece,[&](Point v){return edge(a,z,v);});
            }
            matched+=std::abs(area(piece));
        }
        if(matched+std::max(own*1e-5,1e-9)<own)return false;
        total+=own;
    }
    if(incomingArea)*incomingArea+=total;
    return total>0;
}
inline bool pays(const Patch& p,const std::vector<const Mark*>& selected){
    double original=0,footprint=0;
    for(const auto* m:selected)if(!covered(p,*m,&original))return false;
    for(std::size_t j=0;j<p.mesh.size();j+=3)
        footprint+=.5*std::abs(edge(project(p.frame,p.mesh[j].p),project(p.frame,p.mesh[j+1].p),project(p.frame,p.mesh[j+2].p)));
    // The snapshot itself shades its full receiver footprint, including the empty texels.
    return footprint>0 && original>=1.1*footprint;
}
inline bool sameReceivers(const Patch& p,const Mark& m){
    for(const auto& r:m.receivers){
        auto k=key(r);bool found=false;
        for(const auto& r0:p.receivers)if(key(r0)==k){found=true;break;}
        if(!found)return false;
    }
    return true;
}
inline bool paint(const Patch& p,const Mark& m,const Fit& fit,Delta& d){
    const auto& f=p.frame;
    int x0=std::max(0,int(std::floor((fit.box.a-f.left)/f.pitch))),x1=std::min(int(p.width),int(std::ceil((fit.box.c-f.left)/f.pitch)));
    int y0=std::max(0,int(std::floor((fit.box.b-f.bottom)/f.pitch))),y1=std::min(int(p.height),int(std::ceil((fit.box.d-f.bottom)/f.pitch)));
    if(x0>=x1||y0>=y1)return false;
    d.id=m.id;d.stampIndex=(unsigned)p.stamps.size();d.stamp=m.stamp;
    d.rect={unsigned(x0),unsigned(y0),unsigned(x1-x0),unsigned(y1-y0)};
    d.rgba.resize(std::size_t(d.rect.w)*d.rect.h);d.owner.resize(d.rgba.size());
    for(unsigned y=0;y<d.rect.h;++y){
        auto first=(y+d.rect.y)*p.width+d.rect.x,at=y*d.rect.w;
        std::copy_n(p.rgba.data()+first,d.rect.w,d.rgba.data()+at);std::copy_n(p.owner.data()+first,d.rect.w,d.owner.data()+at);
    }
    bool any=false;int usedLeft=x1,usedRight=x0,usedBottom=y1,usedTop=y0;
    for(std::size_t i=0;i<m.clipped.size();i+=3){
        auto* vs=m.clipped.data()+i;Point q[3];Box box;
        for(unsigned j=0;j<3;++j){q[j]=project(f,vs[j].p);q[j].x=(q[j].x-f.left)/f.pitch;q[j].y=(q[j].y-f.bottom)/f.pitch;box.grow(q[j]);}
        const double det=edge(q[0],q[1],q[2]);if(det<=1e-12)continue;
        int left=std::max(x0,int(std::ceil(box.a-.5))),right=std::min(x1-1,int(std::floor(box.c-.5)));
        int bottom=std::max(y0,int(std::ceil(box.b-.5))),top=std::min(y1-1,int(std::floor(box.d-.5)));
        for(int y=bottom;y<=top;++y)for(int x=left;x<=right;++x){
            const Point at{double(x)+.5,double(y)+.5};
            double a=edge(q[1],q[2],at)/det,b=edge(q[2],q[0],at)/det,c=1.-a-b;
            if(a< -1e-9||b< -1e-9||c< -1e-9)continue;
            auto n=std::size_t(y-y0)*d.rect.w+(x-x0);if((d.owner[n]&511u)==d.stampIndex)continue;
            auto sample=m.image->sample(float(a*vs[0].u+b*vs[1].u+c*vs[2].u),float(a*vs[0].v+b*vs[1].v+c*vs[2].v));
            if(sample[3]*1.7f-.7f<.5f/255.f)continue;
            unsigned k;auto pixel=composite(d.rgba[n],d.owner[n],p.stamps,m.stamp,sample,k);
            if(!(pixel>>24))continue;
            d.rgba[n]=pixel;d.owner[n]=std::uint16_t(d.stampIndex|(k<<9));any=true;
            usedLeft=std::min(usedLeft,x);usedRight=std::max(usedRight,x);usedBottom=std::min(usedBottom,y);usedTop=std::max(usedTop,y);
        }
    }
    if(!any)return false;
    Rect used{unsigned(usedLeft),unsigned(usedBottom),unsigned(usedRight-usedLeft+1),unsigned(usedTop-usedBottom+1)};
    if(used.w<d.rect.w||used.h<d.rect.h){
        for(unsigned y=0;y<used.h;++y){
            auto begin=(used.y-d.rect.y+y)*d.rect.w+used.x-d.rect.x,at=y*used.w;
            std::memmove(d.rgba.data()+at,d.rgba.data()+begin,used.w*sizeof(d.rgba[0]));
            std::memmove(d.owner.data()+at,d.owner.data()+begin,used.w*sizeof(d.owner[0]));
        }
        d.rgba.resize(std::size_t(used.w)*used.h);d.owner.resize(d.rgba.size());d.rect=used;
    }
    return true;
}
} // namespace chart

inline bool prepareAppend(const Patch& p,const Mark& m,std::uint32_t now,const Config& c,Delta& out){
    if(!chart::valid(m)||!p.width||m.sector!=p.sector||m.material!=p.material||p.stamps.size()>=512||
       (!p.ids.empty()&&m.id<=p.ids.back())||!chart::fade(m.stamp,now))return false;
    chart::Fit fit;if(!chart::fit(m,p.frame,c,fit)||fit.density*p.frame.pitch>1.005)return false;
    const Frame& f=p.frame;
    if(fit.box.a<f.left||fit.box.b<f.bottom||fit.box.c>f.left+p.width*f.pitch||fit.box.d>f.bottom+p.height*f.pitch||
       !chart::sameReceivers(p,m)||!chart::covered(p,m))return false;
    Delta d;if(!chart::paint(p,m,fit,d))return false;out=std::move(d);return true;
}
inline bool commit(Patch& p,const Delta& d){
    if(d.stampIndex!=p.stamps.size()||d.stampIndex>=512||!d.rect.w||!d.rect.h||d.rect.x>=p.width||d.rect.y>=p.height||
       d.rect.w>p.width-d.rect.x||d.rect.h>p.height-d.rect.y||d.rgba.size()!=std::size_t(d.rect.w)*d.rect.h||d.owner.size()!=d.rgba.size()||
       (!p.ids.empty()&&d.id<=p.ids.back()))return false;
    p.stamps.reserve(512);p.ids.reserve(511);
    for(unsigned y=0;y<d.rect.h;++y){
        auto first=(y+d.rect.y)*p.width+d.rect.x,at=y*d.rect.w;
        std::copy_n(d.rgba.data()+at,d.rect.w,p.rgba.data()+first);std::copy_n(d.owner.data()+at,d.rect.w,p.owner.data()+first);
    }
    p.stamps.push_back(d.stamp);p.ids.push_back(d.id);return true;
}
inline bool build(const std::vector<const Mark*>& pending,std::uint32_t now,const Config& c,Patch& out){
    using namespace chart;
    if(pending.size()<c.minimum||c.minimum<2||c.maximum<c.minimum||c.maximum>=512||c.maxDimension<4||c.maxDimension>4096||
       !std::isfinite(c.nearDistance)||c.nearDistance<0||!std::isfinite(c.planeTolerance)||c.planeTolerance<0||
       !std::isfinite(c.normalCos)||c.normalCos<0||c.normalCos>1)return false;
    std::vector<const Mark*> eligible;
    for(std::size_t i=0;i<std::min<std::size_t>(pending.size(),MaxPending);++i){
        const Mark* m=pending[i];if(m&&valid(*m)&&fade(m->stamp,now)>0)eligible.push_back(m);
    }
    std::stable_sort(eligible.begin(),eligible.end(),[](const Mark* a,const Mark* b){return a->id<b->id;});
    for(std::size_t i=0;i+c.minimum<=eligible.size();++i){
        const auto* first=eligible[i];Patch p;if(!seedFrame(*first,p.frame))continue;
        std::vector<const Mark*> group;Box box;double density=0;p.sector=first->sector;p.material=first->material;
        std::vector<Fit> fits;
        for(std::size_t j=i;j<eligible.size()&&group.size()<c.maximum;++j){
            auto* m=eligible[j];if(m->sector!=p.sector||m->material!=p.material||(!group.empty()&&m->id==group.back()->id))continue;
            Fit q;if(!fit(*m,p.frame,c,q))continue;
            if(!group.empty()&&std::none_of(fits.begin(),fits.end(),[&](const Fit& n){return distance(n.box,q.box)<=c.nearDistance;}))continue;
            Box proposed=box;proposed.add(q.box);auto den=std::max(density,q.density);Frame f=p.frame;unsigned w,h;
            if(!extent(proposed,den,c,f,w,h))continue;
            group.push_back(m);fits.push_back(q);box=proposed;density=den;p.frame=f;p.width=w;p.height=h;
        }
        if(group.size()<c.minimum||!mesh(p,group,c)||!pays(p,group))continue;
        p.stamps.reserve(512);p.ids.reserve(511);p.stamps.emplace_back();
        p.rgba.resize(std::size_t(p.width)*p.height);p.owner.resize(p.rgba.size());
        for(std::size_t n=0;n<group.size();++n){Delta d;if(paint(p,*group[n],fits[n],d)&&!commit(p,d))return false;}
        if(p.ids.size()<c.minimum)continue;
        out=std::move(p);return true;
    }
    return false;
}
} // namespace PainterStaticBlood
