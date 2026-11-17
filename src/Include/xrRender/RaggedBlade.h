#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace painter_detail {
struct RaggedBlade {
    static constexpr double lattice=.015,stepAmp=.022,bigScale=.12,bigAmp=.016,smallScale=.025,smallAmp=.006,jitterAmp=.004,faceScale=1.9;
    double half{},width{},reach{},upperFace{},lowerFace{},dt{},ds{},nodeMin{},nodeMax{};
    std::uint32_t seed32{};int nt{},ns{};
    bool active{};
    std::vector<double> nodes; // (nt+1) x (ns+1), row = depth index
    struct Bounds {double lowerMin,lowerMax,upperMin,upperMax;};
    static std::uint64_t mix(std::uint64_t x) {
        x^=x>>30;x*=0xbf58476d1ce4e5b9ull;x^=x>>27;
        x*=0x94d049bb133111ebull;return x^(x>>31);
    }
    static std::uint32_t hash32(std::uint32_t x) {x^=x>>16;x*=0x7feb352du;x^=x>>15;x*=0x846ca68bu;x^=x>>16;return x;}
    static double unit01(std::uint32_t seed,std::int32_t a,std::int32_t b,std::uint32_t salt) {
        return double(hash32(seed^hash32(std::uint32_t(a)*0x9e3779b1u+hash32(std::uint32_t(b)+salt))))*(1./4294967296.);
    }
    static double valueNoise(std::uint32_t seed,std::uint32_t salt,double x,double y) {
        const double x0=std::floor(x),y0=std::floor(y);const std::int32_t xi=std::int32_t(x0),yi=std::int32_t(y0);
        const double fx=x-x0,fy=y-y0,sx=fx*fx*(3-2*fx),sy=fy*fy*(3-2*fy);
        auto v=[&](std::int32_t a,std::int32_t b){return 2*unit01(seed,a,b,salt)-1;};
        const double a=v(xi,yi)+(v(xi+1,yi)-v(xi,yi))*sx,b=v(xi,yi+1)+(v(xi+1,yi+1)-v(xi,yi+1))*sx;
        return a+(b-a)*sy;
    }
    static std::uint32_t fold(std::uint64_t seed){const auto s=std::uint32_t(seed^(seed>>32));return s?s:0x9e3779b9u;}
    // Torn chunk of node (i, j): Voronoi plateaus on the node lattice - one feature node per 2 x 2 block (integer jitter),
    // nearest by integer squared distance, ties to the first in scan order - so the shader picks the same chunk exactly.
    static double plateau(std::uint32_t seed,int i,int j) {
        auto fdiv=[](int a){return a>=0?a/2:-((1-a)/2);};
        const int ci=fdiv(i),cj=fdiv(j);int best=1<<30;std::int32_t bi=0,bj=0;
        for(int a=-1;a<=1;++a)for(int b=-1;b<=1;++b){const std::int32_t x=ci+a,y=cj+b;
            const std::uint32_t h=hash32(seed^hash32(std::uint32_t(x)*0x9e3779b1u+hash32(std::uint32_t(y)+7u)));
            const int fx=2*x+int(h&1u),fy=2*y+int((h>>1)&1u),d=(i-fx)*(i-fx)+(j-fy)*(j-fy);
            if(d<best){best=d;bi=x;bj=y;}}
        return 2*unit01(seed,bi,bj,5u)-1;
    }
    // Centre displacement of lattice node (i along t from 0, j along s from -width) for a blade's lattice.
    static double nodeValue(std::uint32_t seed,int i,int j,double dt,double ds,double width) {
        const double t=i*dt,s=-width+j*ds;
        return stepAmp*plateau(seed,i,j)+bigAmp*valueNoise(seed,1u,t/bigScale,s/bigScale)
              +smallAmp*valueNoise(seed,2u,t/smallScale,s/smallScale)+jitterAmp*(2*unit01(seed,i,j,3u)-1);
    }
    RaggedBlade()=default;
    RaggedBlade(std::uint64_t seed,double h,double w,double p):half(h),width(w),reach(p),active(seed!=0) {
        if(!active)return;
        seed32=fold(seed);upperFace=lowerFace=faceScale*h;
        nt=std::max(1,int(std::lround(p/lattice)));ns=std::max(1,int(std::lround(2*w/lattice)));dt=p/nt;ds=2*w/ns;
        nodes.resize(std::size_t(nt+1)*(ns+1));nodeMin=1e300;nodeMax=-1e300;
        for(int i=0;i<=nt;++i)for(int j=0;j<=ns;++j){const double d=nodeValue(seed32,i,j,dt,ds,w);
            nodes[std::size_t(i)*(ns+1)+j]=d;nodeMin=std::min(nodeMin,d);nodeMax=std::max(nodeMax,d);}
    }
    double node(int i,int j)const{return nodes[std::size_t(i)*(ns+1)+j];}
    // Lattice cell and local coordinates (u along t, v along s) of a point; t is clamped to [0, reach].
    void cell(double t,double s,int&i,int&j,double&u,double&v)const {
        const double x=std::clamp(t,0.,reach)/dt,y=std::clamp(s+width,0.,2*width)/ds;
        i=std::clamp(int(std::floor(x)),0,nt-1);j=std::clamp(int(std::floor(y)),0,ns-1);u=x-i;v=y-j;
    }
    // Cell (i, j) splits on the diagonal (0,0)-(1,1) when i+j is even, else on (1,0)-(0,1).
    static bool evenSplit(int i,int j){return ((i+j)&1)==0;}
    double disp(double t,double s)const {
        if(!active)return 0;int i,j;double u,v;cell(t,s,i,j,u,v);
        const double d00=node(i,j),d10=node(i+1,j),d01=node(i,j+1),d11=node(i+1,j+1);
        if(evenSplit(i,j))return u>=v?d00+(d10-d00)*u+(d11-d10)*v:d00+(d11-d01)*u+(d01-d00)*v;
        return u+v<=1?d00+(d10-d00)*u+(d01-d00)*v:d11+(d01-d11)*(1-u)+(d10-d11)*(1-v);
    }
    // The same centre from published parameters (the shader's evaluation, in double): probe emulation.
    static double dispFrom(std::uint32_t seed,int nt,int ns,double dt,double ds,double width,double reach,double t,double s) {
        const double x=std::clamp(t,0.,reach)/dt,y=std::clamp(s+width,0.,2*width)/ds;
        const int i=std::clamp(int(std::floor(x)),0,nt-1),j=std::clamp(int(std::floor(y)),0,ns-1);const double u=x-i,v=y-j;
        auto N=[&](int a,int b){return nodeValue(seed,a,b,dt,ds,width);};
        const double d00=N(i,j),d10=N(i+1,j),d01=N(i,j+1),d11=N(i+1,j+1);
        if(evenSplit(i,j))return u>=v?d00+(d10-d00)*u+(d11-d10)*v:d00+(d11-d01)*u+(d01-d00)*v;
        return u+v<=1?d00+(d10-d00)*u+(d01-d00)*v:d11+(d01-d11)*(1-u)+(d10-d11)*(1-v);
    }
    bool contains(double n,double s,double t)const {
        if(std::abs(s)>width)return false;
        const double v=n-disp(t,s);return v>=-lowerFace&&v<=upperFace;
    }
    // Face extremes over [t0,t1] x [s0,s1]: a piecewise-planar function takes its extremes at nodes of the cells
    // the rectangle meets (or on their edges, between node values).
    Bounds bounds(double t0,double t1,double s0,double s1)const {
        if(!active)return {-half,-half,half,half};
        int i0,j0,i1,j1;double u,v;cell(t0,s0,i0,j0,u,v);cell(t1,s1,i1,j1,u,v);
        double lo=1e300,hi=-1e300;
        for(int i=i0;i<=i1+1;++i)for(int j=j0;j<=j1+1;++j){const double d=node(i,j);lo=std::min(lo,d);hi=std::max(hi,d);}
        return {lo-lowerFace,hi-lowerFace,lo+upperFace,hi+upperFace};
    }
    double extent()const {
        if(!active)return half;
        return std::max(nodeMax+upperFace,lowerFace-nodeMin);
    }
    // Distance along a unit ray in the section plane (x along the normal, y along the side) from the axis to the
    // sheet's boundary at depth t (the polar planner; the torn sheet itself is published by planarSheet).
    double radial(double x,double y,double t)const {
        const double d=disp(t,0.);const double up=std::max(1e-6,d+upperFace),down=std::max(1e-6,lowerFace-d);
        const double n=std::abs(x)>1e-18?(x>0?up:down)/std::abs(x):1e300;
        const double s=std::abs(y)>1e-18?width/std::abs(y):1e300;
        return std::min(n,s);
    }
    void seedDepths(std::vector<double>&out)const {
        if(active)for(int i=1;i<nt;++i)out.push_back(i*dt);
    }
    void seedAngles(std::vector<double>&out)const {
        constexpr double tau=6.2831853071795864769;
        for(double s:{-width,width})for(double n:{nodeMin-lowerFace,nodeMax+upperFace}){
            double a=std::atan2(s,n);if(a<0)a+=tau;out.push_back(a);
        }
    }
};
}
