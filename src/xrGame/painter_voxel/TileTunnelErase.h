#pragma once
// Include at global scope. The dependent Grid/Cut types may be defined later.
// Exact tile-native removal planning only: no geometry and no Grid mutation.
#include "FullTileErase.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
#ifdef GFY_RX_PROF
#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#endif
#ifdef _MSC_VER
#include <intrin.h>
#endif
#ifdef __AVX__
#include <immintrin.h>
#endif

namespace painter_detail {
namespace tile_tunnel_detail {
inline unsigned popcount(std::uint64_t x) noexcept {
#ifdef _MSC_VER
    return unsigned(__popcnt64(x));
#else
    return unsigned(__builtin_popcountll(x));
#endif
}
inline unsigned firstBit(std::uint64_t x) noexcept {
#ifdef _MSC_VER
    unsigned long b;_BitScanForward64(&b,x);return unsigned(b);
#else
    return unsigned(__builtin_ctzll(x));
#endif
}
inline unsigned lastBit(std::uint64_t x) noexcept { // highest set bit (BSR: no LZCNT on AVX-only CPUs)
#ifdef _MSC_VER
    unsigned long b;_BitScanReverse64(&b,x);return unsigned(b);
#else
    return 63u-unsigned(__builtin_clzll(x));
#endif
}
inline unsigned byteSpan(int first,int last) noexcept {
    return ((1u<<(last-first+1))-1u)<<first;
}
// The analytic tunnel cutter (LiveDamageWorker.cpp Tunnel): an entry bore with a seeded ragged edge profile. For it the
// planner decides the cells of an undecided column with TunnelCellFilter before it asks the exact predicate.
template<class,class=void> struct IsProfiledTunnel:std::false_type{};
template<class C> struct IsProfiledTunnel<C,std::void_t<decltype(std::declval<const C&>().edge.radiusTables()),decltype(std::declval<const C&>().edge_u),
    decltype(std::declval<const C&>().edge_v),decltype(std::declval<const C&>().cracks),decltype(std::declval<const C&>().burst_len),
    decltype(std::declval<const C&>().radiusAt(0.)),decltype(std::declval<const C&>().inlet),decltype(std::declval<const C&>().cap)>>:std::true_type{};
// Slab cutters (R18 explosion blades) are boxes: the round-channel cell filter and SIMD columns do not apply to them.
template<class,class=void> struct HasSlabFlag:std::false_type{};
template<class C> struct HasSlabFlag<C,std::void_t<decltype(std::declval<const C&>().slab)>>:std::true_type{};
template<class C> inline bool cutIsSlab(const C& c){if constexpr(HasSlabFlag<C>::value)return c.slab;else return false;}
// Cell membership for Tunnel::contains without its exact angle: the same arithmetic for the depth t, the axis distance
// and the radius limit (so every decision made from them is identical), and the edge-profile multiplier at the
// polynomial angle (WoundEdgeProfile::approximateAngle) instead of std::atan2. The profile is piecewise linear in the
// angle, so the multiplier at the exact angle lies within `err` (steepest table step per radian x the angle bound) of
// the one computed here; cells whose distance falls inside that band (micrometres wide) are left to contains().
// classify(): 1 = contains() is true, 0 = contains() is false, -1 = undecided. The burst-crack zone is undecided.
template<class Cut> struct TunnelCellFilter {
    using Profile=typename std::decay<decltype(std::declval<const Cut&>().edge)>::type;
    static constexpr double tau=6.283185307179586476925286766559;
    const Cut& cut;const std::array<std::array<double,129>,2>* tables{};double err{};bool edge{};
    explicit TunnelCellFilter(const Cut& c):cut(c),edge(c.edge.enabled()){
        if(edge){tables=&c.edge.radiusTables();double step=0;
            for(const auto& table:*tables)for(unsigned i=0;i<128;++i)step=std::max(step,std::abs(table[i+1]-table[i]));
            err=step*(128/tau)*Profile::angle_error+1e-12;}
    }
    template<class Vec> int classify(const Vec& p) const {
        const Vec q=p-cut.entry;const double t=dot(q,cut.direction);
        if(t< -cut.inlet||t>cut.penetration)return 0;
        const double radial2=std::max(0.,dot(q,q)-t*t);
        const double z=cut.exit_distance<0?std::max(0.,t-(cut.penetration-cut.cap))/cut.cap:0;
        const double r=cut.radiusAt(t);
        const double limit=r*r*std::max(0.,1-z*z);
        if(cut.cracks>0&&t>=cut.exit_distance-cut.burst_len)return -1;
        if(radial2>limit)return 0;
        if(!edge||radial2<=limit*Profile::minimum_radius*Profile::minimum_radius)return 1; // contains()'s own expression
        const double eu=dot(q,cut.edge_u),ev=dot(q,cut.edge_v);
        if(!(std::max(std::abs(eu),std::abs(ev))>1e-300))return -1;
        double angle=Profile::approximateAngle(eu,ev);if(angle<0)angle+=tau;
        const double coordinate=angle*(128/tau);const unsigned index=std::min(unsigned(coordinate),127u);const double fraction=coordinate-index;
        const auto& a=(*tables)[0];const auto& b=(*tables)[1];
        const double first=a[index]+(a[index+1]-a[index])*fraction,last=b[index]+(b[index+1]-b[index])*fraction;
        const double scale=first+(last-first)*std::clamp(t/cut.penetration,0.,1.);
        const double lo=std::max(0.,scale-err),hi=scale+err;
        if(radial2<limit*lo*lo)return 1;
        if(radial2>limit*hi*hi)return 0;
        return -1;
    }
};
#ifdef __AVX__
// The cells of one tile column (up to 8, stepping one pitch along `axis`) at once, AVX single precision. Per column the
// depth t (linear in the cell index), the squared axis distance (a quadratic) and the edge-plane coordinates (linear)
// are set up in double; the lanes evaluate them in single precision and every decision keeps a margin that bounds the
// single-precision error: E on the squared distance, Et on the depth, 1e-5 relative on the radius limit, `err` on the
// edge multiplier (as TunnelCellFilter). contains()'s radius limit, radiusAt(t)^2 x (1-z^2) of the tip cap, is monotone
// in t along the whole channel when there is no cavitation swell (flat, then growing in the exit flare; or flat, then
// shrinking in the cap), so a lane bounds it by its values at t-Et and t+Et. Lanes inside a margin, in the swell or the
// burst-crack zone, at the very tip, or at the inlet/penetration planes are reported undecided and go through
// TunnelCellFilter and Tunnel::contains: the result is identical to contains() for every cell.
template<class Cut> struct TunnelColumnEval {
    using Profile=typename std::decay<decltype(std::declval<const Cut&>().edge)>::type;
    static constexpr double tau=6.283185307179586476925286766559;
    bool usable{},edge{},swell{},cracks{},flare{},flareAll{},cap{};int axis{};
    double entry[3]{},dir[3]{},eu[3]{},ev[3]{},pitch{},inlet{},pen{},cav0{},cav1{},crackFrom{};
    double radius{},grow{},flareFrom{},invFlare{},capFrom{},invCap{},m2{}; // limit(t)=(radius*(1+grow*blend(t)))^2*(1-z(t)^2)
    // Per cut and column axis: depth step, the axis-normal step w (squared length ww) of one cell along the column, the
    // edge-plane steps; the limit is the constant lFlat for depths up to flatTo (before the flare/cap).
    double dt{},w[3]{},ww{},halfInvC{},du{},dv{},flatTo{},lFlat{};
    float err{},invPen{};
    alignas(16) float table[128*4]{}; // per sector: first-table value, its step to the next sector, last-table value, its step
    TunnelColumnEval(const Cut& c,int ax,double p):edge(c.edge.enabled()),axis(ax),pitch(p){
        for(int k=0;k<3;++k){entry[k]=c.entry[k];dir[k]=c.direction[k];eu[k]=edge?c.edge_u[k]:0.;ev[k]=edge?c.edge_v[k]:0.;}
        inlet=c.inlet;pen=c.penetration;radius=c.radius;if(!(pen>0)||!(radius>0))return;
        if(c.exit_distance>=0){flare=true;grow=c.exit_mult*(1+c.burst_amp)-1;const double span=std::min(c.exit_distance,c.radius*1.5);
            if(span>1e-12){flareFrom=c.exit_distance-span;invFlare=1/span;}else flareAll=true;}
        else{cap=true;capFrom=c.penetration-c.cap;invCap=1/c.cap;}
        swell=c.cav_amp>0;cav0=c.cav_t0;cav1=c.cav_t1;cracks=c.cracks>0;crackFrom=c.exit_distance-c.burst_len;
        m2=Profile::minimum_radius*Profile::minimum_radius;invPen=float(1/pen);
        dt=pitch*dir[axis];for(int k=0;k<3;++k)w[k]=(k==axis?pitch:0.)-dir[k]*dt;ww=w[0]*w[0]+w[1]*w[1]+w[2]*w[2];halfInvC=ww>0?1/(2*ww):0.;
        du=pitch*eu[axis];dv=pitch*ev[axis];
        flatTo=flare?(flareAll?std::numeric_limits<double>::infinity():flareFrom):capFrom;lFlat=flareAll?limitAt(0.):radius*radius;
        if(edge){const auto& tables=c.edge.radiusTables();double step=0;
            for(unsigned i=0;i<128;++i){
                table[4*i]=float(tables[0][i]);table[4*i+1]=float(tables[0][i+1]-tables[0][i]);
                table[4*i+2]=float(tables[1][i]);table[4*i+3]=float(tables[1][i+1]-tables[1][i]);
                step=std::max({step,std::abs(tables[0][i+1]-tables[0][i]),std::abs(tables[1][i+1]-tables[1][i])});}
            // angle bound (float polynomial + its approximation error << angle_error), depth and single-precision lerps
            err=float(step*(128/tau)*Profile::angle_error+8e-6);}
        usable=true;
    }
    double limitAt(double t)const{
        const double blend=flare?(flareAll?1.:std::clamp((t-flareFrom)*invFlare,0.,1.)):0.;const double r=radius*(1+grow*blend);
        const double z=cap?std::max(0.,t-capFrom)*invCap:0.;return r*r*std::max(0.,1-z*z);}
    __m256 limitAt(__m256 T,__m256& z)const{
        const __m256 zero=_mm256_setzero_ps(),one=_mm256_set1_ps(1.f);__m256 r=_mm256_set1_ps(float(radius));
        if(flare){const __m256 blend=flareAll?one:_mm256_min_ps(_mm256_max_ps(_mm256_mul_ps(_mm256_sub_ps(T,_mm256_set1_ps(float(flareFrom))),_mm256_set1_ps(float(invFlare))),zero),one);
            r=_mm256_mul_ps(r,_mm256_add_ps(one,_mm256_mul_ps(_mm256_set1_ps(float(grow)),blend)));}
        __m256 L=_mm256_mul_ps(r,r);z=zero;
        if(cap){z=_mm256_mul_ps(_mm256_max_ps(_mm256_sub_ps(T,_mm256_set1_ps(float(capFrom))),zero),_mm256_set1_ps(float(invCap)));
            L=_mm256_mul_ps(L,_mm256_max_ps(_mm256_sub_ps(one,_mm256_mul_ps(z,z)),zero));}
        return L;}
    // Cells s=0..n-1 of the column whose first cell centre is p0: bit s of `inside` = contains() is true, bit s of
    // `undecided` = not decided here; a cell in neither mask is outside.
    // Returns 0 when the whole column was decided in double, 1 after the lanes, 2 after the lanes and the edge angle.
    template<class Vec> int column(const Vec& p0,int n,unsigned& inside,unsigned& undecided) const {
        const unsigned lanes=(1u<<n)-1u;
        double q0[3],qp[3];for(int k=0;k<3;++k)q0[k]=p0[k]-entry[k];
        const double t0=q0[0]*dir[0]+q0[1]*dir[1]+q0[2]*dir[2],last=double(n-1);
        for(int k=0;k<3;++k)qp[k]=q0[k]-dir[k]*t0;
        const double a=qp[0]*qp[0]+qp[1]*qp[1]+qp[2]*qp[2],b=2*(qp[0]*w[0]+qp[1]*w[1]+qp[2]*w[2]);
        const double tMin=std::min(t0,t0+dt*last),tMax=std::max(t0,t0+dt*last);
        // Whole-column decisions in double (margins far above the rounding of contains()'s |q|^2-t^2 and its limit).
        if(tMax< -inlet-1e-9||tMin>pen+1e-9){inside=0;undecided=0;return 0;}
        const bool plain=!(swell&&tMax>cav0-1e-9&&tMin<cav1+1e-9)&&!(cracks&&tMax>=crackFrom-1e-9);
        if(plain){double rMin=std::min(a,a+last*(b+last*ww)),rMax=std::max(a,a+last*(b+last*ww));
            if(ww>0){const double s=-b*halfInvC;if(s>0&&s<last)rMin=std::min(rMin,a+s*(b+s*ww));}
            const double slack=1e-9*(a+std::abs(b)*last+ww*last*last)+1e-15;
            const bool flat=tMax<flatTo-1e-9;const double l0=flat?lFlat:limitAt(tMin),l1=flat?lFlat:limitAt(tMax),lMax=std::max(l0,l1),lMin=std::min(l0,l1);
            if(rMin>lMax*(1+1e-9)+slack){inside=0;undecided=0;return 0;}
            if(tMin> -inlet+1e-9&&tMax<pen-1e-9&&rMax<lMin*m2*(1-1e-9)-slack){inside=lanes;undecided=0;return 0;}}
        // Lanes (single precision).
        const double E=4e-6*(a+std::abs(b)*last+ww*last*last)+1e-15,Et=2e-6*(std::abs(t0)+std::abs(dt)*last+inlet+pen)+1e-9;
        const __m256 S=_mm256_setr_ps(0.f,1.f,2.f,3.f,4.f,5.f,6.f,7.f),zero=_mm256_setzero_ps(),Etv=_mm256_set1_ps(float(Et));
        const __m256 T=_mm256_add_ps(_mm256_set1_ps(float(t0)),_mm256_mul_ps(S,_mm256_set1_ps(float(dt))));
        const __m256 R2=_mm256_add_ps(_mm256_set1_ps(float(a)),_mm256_mul_ps(S,_mm256_add_ps(_mm256_set1_ps(float(b)),_mm256_mul_ps(S,_mm256_set1_ps(float(ww))))));
        const __m256 Rin=_mm256_add_ps(R2,_mm256_set1_ps(float(E))),Rout=_mm256_sub_ps(R2,_mm256_set1_ps(float(E)));
        const __m256 tOut=_mm256_or_ps(_mm256_cmp_ps(T,_mm256_set1_ps(float(-inlet-Et)),_CMP_LT_OQ),_mm256_cmp_ps(T,_mm256_set1_ps(float(pen+Et)),_CMP_GT_OQ));
        __m256 valid=_mm256_and_ps(_mm256_cmp_ps(T,_mm256_set1_ps(float(-inlet+Et)),_CMP_GT_OQ),_mm256_cmp_ps(T,_mm256_set1_ps(float(pen-Et)),_CMP_LT_OQ));
        if(swell)valid=_mm256_and_ps(valid,_mm256_or_ps(_mm256_cmp_ps(T,_mm256_set1_ps(float(cav0-Et)),_CMP_LT_OQ),_mm256_cmp_ps(T,_mm256_set1_ps(float(cav1+Et)),_CMP_GT_OQ)));
        if(cracks)valid=_mm256_and_ps(valid,_mm256_cmp_ps(T,_mm256_set1_ps(float(crackFrom-Et)),_CMP_LT_OQ));
        __m256 L0,L1;
        if(tMax+Et<flatTo-1e-9)L0=L1=_mm256_set1_ps(float(lFlat)); // every lane (and its +-Et) before the flare/cap
        else{__m256 zLo,zHi;L0=limitAt(_mm256_sub_ps(T,Etv),zLo);L1=limitAt(_mm256_add_ps(T,Etv),zHi);
            if(cap)valid=_mm256_and_ps(valid,_mm256_cmp_ps(_mm256_max_ps(zLo,zHi),_mm256_set1_ps(.99f),_CMP_LT_OQ));} // the very tip: exact
        const __m256 Lmin=_mm256_mul_ps(_mm256_min_ps(L0,L1),_mm256_set1_ps(1-1e-5f)),Lmax=_mm256_mul_ps(_mm256_max_ps(L0,L1),_mm256_set1_ps(1+1e-5f));
        const __m256 out1=_mm256_cmp_ps(Rout,Lmax,_CMP_GT_OQ);
        __m256 in=_mm256_cmp_ps(Rin,edge?_mm256_mul_ps(Lmin,_mm256_set1_ps(float(m2))):Lmin,_CMP_LT_OQ),out=out1;int how=1;
        if(edge&&(_mm256_movemask_ps(_mm256_andnot_ps(_mm256_or_ps(in,out1),valid))&int(lanes))){how=2;
            // The edge multiplier at the polynomial angle of each lane (WoundEdgeProfile::approximateAngle).
            const __m256 sign=_mm256_set1_ps(-0.f);
            const __m256 U=_mm256_add_ps(_mm256_set1_ps(float(q0[0]*eu[0]+q0[1]*eu[1]+q0[2]*eu[2])),_mm256_mul_ps(S,_mm256_set1_ps(float(du))));
            const __m256 V=_mm256_add_ps(_mm256_set1_ps(float(q0[0]*ev[0]+q0[1]*ev[1]+q0[2]*ev[2])),_mm256_mul_ps(S,_mm256_set1_ps(float(dv))));
            const __m256 ax=_mm256_andnot_ps(sign,U),ay=_mm256_andnot_ps(sign,V);
            const __m256 r=_mm256_div_ps(_mm256_min_ps(ax,ay),_mm256_max_ps(ax,ay)),r2=_mm256_mul_ps(r,r);
            __m256 poly=_mm256_add_ps(_mm256_set1_ps(0.05265332f),_mm256_mul_ps(r2,_mm256_set1_ps(-0.01172120f)));
            poly=_mm256_add_ps(_mm256_set1_ps(-0.11643287f),_mm256_mul_ps(r2,poly));
            poly=_mm256_add_ps(_mm256_set1_ps(0.19354346f),_mm256_mul_ps(r2,poly));
            poly=_mm256_add_ps(_mm256_set1_ps(-0.33262347f),_mm256_mul_ps(r2,poly));
            poly=_mm256_add_ps(_mm256_set1_ps(0.99997726f),_mm256_mul_ps(r2,poly));
            __m256 angle=_mm256_mul_ps(r,poly);
            angle=_mm256_blendv_ps(angle,_mm256_sub_ps(_mm256_set1_ps(float(tau*.25)),angle),_mm256_cmp_ps(ay,ax,_CMP_GT_OQ));
            angle=_mm256_blendv_ps(angle,_mm256_sub_ps(_mm256_set1_ps(float(tau*.5)),angle),_mm256_cmp_ps(U,zero,_CMP_LT_OQ));
            angle=_mm256_blendv_ps(angle,_mm256_xor_ps(angle,sign),_mm256_cmp_ps(V,zero,_CMP_LT_OQ));
            angle=_mm256_blendv_ps(angle,_mm256_add_ps(angle,_mm256_set1_ps(float(tau))),_mm256_cmp_ps(angle,zero,_CMP_LT_OQ));
            const __m256 coordinate=_mm256_mul_ps(angle,_mm256_set1_ps(float(128/tau)));
            const __m256 clamped=_mm256_min_ps(_mm256_max_ps(coordinate,zero),_mm256_set1_ps(127.f)); // NaN -> 0: a safe row
            alignas(32) int index[8];_mm256_store_si256(reinterpret_cast<__m256i*>(index),_mm256_cvttps_epi32(clamped));
            const __m256 fraction=_mm256_sub_ps(coordinate,_mm256_cvtepi32_ps(_mm256_load_si256(reinterpret_cast<const __m256i*>(index))));
            const __m256 t0v=_mm256_insertf128_ps(_mm256_castps128_ps256(_mm_load_ps(table+4*index[0])),_mm_load_ps(table+4*index[4]),1);
            const __m256 t1v=_mm256_insertf128_ps(_mm256_castps128_ps256(_mm_load_ps(table+4*index[1])),_mm_load_ps(table+4*index[5]),1);
            const __m256 t2v=_mm256_insertf128_ps(_mm256_castps128_ps256(_mm_load_ps(table+4*index[2])),_mm_load_ps(table+4*index[6]),1);
            const __m256 t3v=_mm256_insertf128_ps(_mm256_castps128_ps256(_mm_load_ps(table+4*index[3])),_mm_load_ps(table+4*index[7]),1);
            const __m256 u0=_mm256_unpacklo_ps(t0v,t1v),u1=_mm256_unpackhi_ps(t0v,t1v),u2=_mm256_unpacklo_ps(t2v,t3v),u3=_mm256_unpackhi_ps(t2v,t3v);
            const __m256 f0=_mm256_shuffle_ps(u0,u2,_MM_SHUFFLE(1,0,1,0)),d0=_mm256_shuffle_ps(u0,u2,_MM_SHUFFLE(3,2,3,2));
            const __m256 f1=_mm256_shuffle_ps(u1,u3,_MM_SHUFFLE(1,0,1,0)),d1=_mm256_shuffle_ps(u1,u3,_MM_SHUFFLE(3,2,3,2));
            const __m256 first=_mm256_add_ps(f0,_mm256_mul_ps(d0,fraction)),lastv=_mm256_add_ps(f1,_mm256_mul_ps(d1,fraction));
            const __m256 depth=_mm256_min_ps(_mm256_max_ps(_mm256_mul_ps(T,_mm256_set1_ps(invPen)),zero),_mm256_set1_ps(1.f));
            const __m256 M=_mm256_add_ps(first,_mm256_mul_ps(_mm256_sub_ps(lastv,first),depth));
            const __m256 lo=_mm256_max_ps(_mm256_sub_ps(M,_mm256_set1_ps(err)),zero),hi=_mm256_add_ps(M,_mm256_set1_ps(err));
            in=_mm256_or_ps(in,_mm256_cmp_ps(Rin,_mm256_mul_ps(_mm256_mul_ps(Lmin,lo),lo),_CMP_LT_OQ));
            out=_mm256_or_ps(out1,_mm256_cmp_ps(Rout,_mm256_mul_ps(_mm256_mul_ps(Lmax,hi),hi),_CMP_GT_OQ));}
        const unsigned yes=unsigned(_mm256_movemask_ps(_mm256_and_ps(valid,in)))&lanes;
        const unsigned no=(unsigned(_mm256_movemask_ps(tOut))|unsigned(_mm256_movemask_ps(_mm256_and_ps(valid,out))))&lanes&~yes;
        inside=yes;undecided=lanes&~yes&~no;return how;
    }
};
#endif
#ifdef GFY_RX_PROF
struct ProfileTimer {
    std::uint64_t& ns;std::chrono::steady_clock::time_point begin=std::chrono::steady_clock::now();bool done{};
    explicit ProfileTimer(std::uint64_t& value):ns(value){}
    void finish(){if(!done){ns+=std::uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-begin).count());done=true;}}
    ~ProfileTimer(){finish();}
};
struct Profile {
    std::size_t worker{},segmentCalls{},laneColumns{},wholeColumns{},angleColumns{},containsCalls{},filterCalls{},rleRows{},rleCells{},uniformTiles{},uniformAir{};
    std::uint64_t lookupNs{},predicateNs{},partialNs{},countNs{};
};
#endif
struct NoCellFilter {template<class Cut> explicit NoCellFilter(const Cut&){} template<class Vec> int classify(const Vec&)const{return -1;}};
struct NoColumnEval {bool usable=false;template<class Cut> NoColumnEval(const Cut&,int,double){} template<class Vec> int column(const Vec&,int,unsigned&,unsigned&)const{return 0;}};
// GFY_TILE_FILTER=0 turns the cell filter off, GFY_TILE_SIMD=0 the column lanes (A/B measurements); results are
// identical either way.
inline bool cellFilterEnabled(){static const bool on=[]{const char* e=std::getenv("GFY_TILE_FILTER");return !e||std::atoi(e)!=0;}();return on;}
inline bool columnLanesEnabled(){static const bool on=[]{const char* e=std::getenv("GFY_TILE_SIMD");return !e||std::atoi(e)!=0;}();return on;}
} // namespace tile_tunnel_detail

template<class Grid> struct TileTunnelPlan {
    using Cell=typename std::decay<decltype(std::declval<Grid&>().size)>::type;
    using TileMap=typename std::decay<decltype(std::declval<Grid&>().tiles)>::type;
    using Tile=typename TileMap::mapped_type;
    struct Stage {Tile finalMask,fresh;};
    bool supported{};
    const char* refusal{}; // implementation fallback reason, never an impact rejection
    Cell tileLo{},tileSize{};
    static constexpr std::uint32_t noStage=~std::uint32_t(0);
    std::vector<std::uint32_t> stageAt; // dense tile window -> compact stage, or noStage (use old grid)
    std::vector<Cell> stageKeys;
    std::vector<Stage> stages; // compact lexicographic x,y,z tile order, z fastest
    std::vector<Cell> touched; // same deterministic order; only nonzero fresh masks
    std::vector<Cell> boundary; // fresh cells with a kept/invalid face neighbor
    std::size_t freshMask{},freshSolid{};
    std::array<std::size_t,64> freshBySlot{};
    std::size_t fullTiles{},emptyTiles{},unresolvedTiles{},oldFullTiles{},scratchBytes{},coarseEmptyTiles{},coarseFullTiles{};
    double classify_ms{},boundary_ms{};
    std::size_t windowIndex(Cell t)const noexcept {
        return (std::size_t(t.x-tileLo.x)*std::size_t(tileSize.y)+std::size_t(t.y-tileLo.y))*std::size_t(tileSize.z)+std::size_t(t.z-tileLo.z);
    }
    std::size_t index(Cell t)const noexcept {return inside(t)?stageAt[windowIndex(t)]:noStage;}
    Cell windowKey(std::size_t i)const noexcept {
        return {tileLo.x+int(i/(std::size_t(tileSize.y)*tileSize.z)),tileLo.y+int((i/std::size_t(tileSize.z))%std::size_t(tileSize.y)),tileLo.z+int(i%std::size_t(tileSize.z))};
    }
    Cell key(std::size_t i)const noexcept {return stageKeys[i];}
    bool inside(Cell t)const noexcept {
        return t.x>=tileLo.x&&t.y>=tileLo.y&&t.z>=tileLo.z&&t.x<tileLo.x+tileSize.x&&t.y<tileLo.y+tileSize.y&&t.z<tileLo.z+tileSize.z;
    }
    // A coarse-empty block has no staged masks: it retains the exact old
    // grid state. Every boundary/oracle read must use this explicit fallback.
    const Tile* finalTile(const Grid& grid,Cell t)const {
        if(t.x<0||t.y<0||t.z<0||t.x>=(grid.size.x+7)/8||t.y>=(grid.size.y+7)/8||t.z>=(grid.size.z+7)/8)return nullptr;
        const auto i=index(t);if(i!=noStage)return &stages[i].finalMask;
        return grid.findTile(t); // overlay-aware (Grid::overlay_base)
    }
};

// parallel(count, grain, callback(first,last,worker)) must finish its callbacks
// before returning, as core::parallelChunks does. Each callback writes only
// its own tile block/counts/output span. Neither this function nor its readers
// may run concurrently with a mutation of grid.
//
// Caller contract: low/high are the same conservative, valid cell bounds used
// by the analytic cutter's original eraseTunnel. Cut::classifyBox and
// classifySegment have their established conservative {-1,0,+1} contracts.
// countField is the immutable tissue field belonging to this Grid's prepared
// anatomy/slot map, or nullptr. Unsupported plans fall back without mutation.
//
// Integration: if !supported use the old backend. If freshSolid==0, preserve
// the old no-op (do not commit even air). Otherwise journal/create EVERY
// touched key before writing, assign stages[index(key)].finalMask, then pass
// boundary to the existing resolveEdgeContacts and expand its touched set.
// This uses sorted tile insertion order, not the former axis-column order.
template<class Grid,class Cut,class Field,class Parallel>
TileTunnelPlan<Grid> planTileTunnel(const Grid& grid,const Cut& cut,
    typename TileTunnelPlan<Grid>::Cell low,typename TileTunnelPlan<Grid>::Cell high,
    const Field* countField,Parallel&& parallel,std::size_t maxScratchBytes=64u<<20) {
    using Plan=TileTunnelPlan<Grid>;using Cell=typename Plan::Cell;using Tile=typename Plan::Tile;
    using Vec=typename std::decay<decltype(grid.origin)>::type;
    using namespace tile_tunnel_detail;
    auto unsupported=[](const char* why){Plan p;p.refusal=why;return p;};
    if(grid.ownership&&!grid.anatomy)return unsupported("owned_grid_without_anatomy");
    Plan plan;
    for(int a=0;a<3;++a)if(high[a]<low[a]){plan.supported=true;return plan;}
    if(!grid.valid(low)||!grid.valid(high))return unsupported("invalid_window");
    const auto started=std::chrono::steady_clock::now();
    std::size_t windowTileCount=1,mapCells=0;
    for(int a=0;a<3;++a){(&plan.tileLo.x)[a]=low[a]/8;(&plan.tileSize.x)[a]=high[a]/8-low[a]/8+1;
        const auto n=std::size_t(plan.tileSize[a]);if(n>maxScratchBytes/sizeof(std::uint32_t)/windowTileCount)return unsupported("tile_index_budget");
        windowTileCount*=n;mapCells+=std::size_t(high[a]-low[a]+1);}
    // Reject coarse 32-cell boxes before ANY per-tile hash lookup or mask
    // allocation. Fully inside boxes likewise certify all their tile centers.
    constexpr int coarseTiles=4;
    const Cell coarseSize{(plan.tileSize.x+coarseTiles-1)/coarseTiles,(plan.tileSize.y+coarseTiles-1)/coarseTiles,(plan.tileSize.z+coarseTiles-1)/coarseTiles};
    const std::size_t coarseCount=std::size_t(coarseSize.x)*coarseSize.y*coarseSize.z;
    const std::size_t mapsBytes=grid.anatomy?mapCells*sizeof(int):0;
    const std::size_t indexBytes=windowTileCount*sizeof(std::uint32_t)+coarseCount*sizeof(std::int8_t);
    if(indexBytes+mapsBytes>maxScratchBytes)return unsupported("tile_index_budget");
    std::vector<std::int8_t> coarse(coarseCount);
    auto coarseIndex=[&](Cell t){return (std::size_t((t.x-plan.tileLo.x)/coarseTiles)*coarseSize.y+unsigned((t.y-plan.tileLo.y)/coarseTiles))*coarseSize.z+unsigned((t.z-plan.tileLo.z)/coarseTiles);};
    parallel(coarseCount,std::size_t(128),[&](std::size_t first,std::size_t last,std::size_t){for(std::size_t i=first;i<last;++i){
        const Cell k{int(i/(std::size_t(coarseSize.y)*coarseSize.z)),int((i/std::size_t(coarseSize.z))%std::size_t(coarseSize.y)),int(i%std::size_t(coarseSize.z))};
        Vec center,half;for(int a=0;a<3;++a){const int firstCell=(plan.tileLo[a]+k[a]*coarseTiles)*8;
            const int lastCell=(plan.tileLo[a]+std::min(plan.tileSize[a],(k[a]+1)*coarseTiles))*8-1;
            center[a]=grid.origin[a]+double(firstCell+lastCell+1)*.5*grid.pitch;half[a]=double(lastCell-firstCell)*.5*grid.pitch;}
        coarse[i]=std::int8_t(cut.classifyBox(center,half));
    }});
    std::size_t tileCount=0;
    for(int x=0;x<coarseSize.x;++x)for(int y=0;y<coarseSize.y;++y)for(int z=0;z<coarseSize.z;++z){
        const std::size_t n=std::size_t(std::min(coarseTiles,plan.tileSize.x-x*coarseTiles))*
            std::min(coarseTiles,plan.tileSize.y-y*coarseTiles)*std::min(coarseTiles,plan.tileSize.z-z*coarseTiles);
        const int certain=coarse[(std::size_t(x)*coarseSize.y+y)*coarseSize.z+z];
        if(certain<0)plan.coarseEmptyTiles+=n;else{tileCount+=n;if(certain>0)plan.coarseFullTiles+=n;}}
    if(tileCount>=Plan::noStage)return unsupported("tile_index_budget");
    static const std::size_t blockTilesSet=[]{const char*e=std::getenv("GFY_SF_TILE_BLOCK");const long v=e?std::atol(e):16;return std::size_t(v>=8&&v<=4096?v:16);}(); // speedfix: was 128
    const std::size_t blockTiles=blockTilesSet,blocks=(tileCount+blockTiles-1)/blockTiles;
    struct Counts {std::size_t mask{},solid{},full{},empty{},unresolved{},oldFull{},changed{};std::array<std::size_t,64> slot{};
#ifdef GFY_RX_PROF
        Profile profile;
#endif
    };
    const std::size_t fixedBytes=indexBytes+tileCount*(sizeof(typename Plan::Stage)+sizeof(Cell))+blocks*sizeof(Counts)+mapsBytes+(blocks+1)*sizeof(std::size_t);
    if(fixedBytes>maxScratchBytes)return unsupported("tile_scratch_budget");
    plan.stageAt.assign(windowTileCount,Plan::noStage);plan.stageKeys.reserve(tileCount);
    // Preserve lexicographic tile order, but skip an empty coarse z-span in
    // one step. Avoid two full-window coordinate/division scans for large
    // mostly-empty oblique cutter boxes.
    for(int x=0;x<plan.tileSize.x;++x)for(int y=0;y<plan.tileSize.y;++y){
        const auto row=(std::size_t(x)*plan.tileSize.y+y)*plan.tileSize.z;
        const auto coarseRow=(std::size_t(x/coarseTiles)*coarseSize.y+y/coarseTiles)*coarseSize.z;
        for(int cz=0;cz<coarseSize.z;++cz){if(coarse[coarseRow+cz]<0)continue;
            for(int z=cz*coarseTiles;z<std::min(plan.tileSize.z,(cz+1)*coarseTiles);++z){
                plan.stageAt[row+z]=std::uint32_t(plan.stageKeys.size());
                plan.stageKeys.push_back({plan.tileLo.x+x,plan.tileLo.y+y,plan.tileLo.z+z});}}}
    plan.stages.resize(tileCount);std::vector<Counts> counts(blocks);
    // Use Grid's source-double center/floor arithmetic, not the float field
    // origin/pitch. Grid collar expansion can shift every anatomy alignment.
    std::array<std::vector<int>,3> coordinates;
    if(grid.anatomy){const auto& an=*grid.anatomy;
        for(int a=0;a<3;++a){auto& row=coordinates[a];row.resize(std::size_t(high[a]-low[a]+1));
            for(int c=low[a];c<=high[a];++c){Cell q{};(&q.x)[a]=c;row[std::size_t(c-low[a])]=int(std::floor((grid.center(q)[a]-an.origin[a])/an.pitch));}}}
    auto coordinate=[&](int cell,int a){return coordinates[a][std::size_t(cell-low[a])];};
    // Same pitch as the anatomy (the fullbody profile): anatomy z of a cell = zStart + (z - low.z) over the whole window.
    bool zConsecutive=false;int zStart=0;
    if(grid.anatomy){zConsecutive=true;zStart=coordinate(low[2],2);
        for(int z=low[2];z<high[2];++z)if(coordinate(z+1,2)!=coordinate(z,2)+1){zConsecutive=false;break;}}
    const auto* slots=grid.material_slots.get();
    auto slotOf=[&](std::uint16_t label){return slots&&label<slots->size()?unsigned((*slots)[label]&63u):1u;};
    const bool fieldAvailable=grid.anatomy&&slots&&countField&&countField->available&&countField->size==grid.anatomy->size;
    int axis=0;for(int a=1;a<3;++a)if(std::abs(cut.direction[a])>std::abs(cut.direction[axis]))axis=a;
    const int u=(axis+1)%3,v=(axis+2)%3;Vec step{};step[axis]=grid.pitch;
    using Filter=typename std::conditional<IsProfiledTunnel<Cut>::value,TunnelCellFilter<Cut>,NoCellFilter>::type;
    const Filter filter(cut);const bool filtering=cellFilterEnabled()&&!cutIsSlab(cut);
#ifdef __AVX__
    using Columns=typename std::conditional<IsProfiledTunnel<Cut>::value,TunnelColumnEval<Cut>,NoColumnEval>::type;
#else
    using Columns=NoColumnEval;
#endif
    const Columns columns(cut,axis,grid.pitch);const bool lanes=columnLanesEnabled()&&columns.usable&&!cutIsSlab(cut);
    auto setRow=[&](Tile& t,Cell c,unsigned row){
        if(axis==2)t.removed[c.x&7]|=std::uint64_t(row)<<((c.y&7)*8);
        else if(axis==1){std::uint64_t spread=row;spread=(spread|(spread<<28))&0x0000000F0000000Full;
            spread=(spread|(spread<<14))&0x0003000300030003ull;spread=(spread|(spread<<7))&0x0101010101010101ull;
            t.removed[c.x&7]|=spread<<(c.z&7);}
        else{const auto bit=std::uint64_t(1)<<((c.y&7)*8+(c.z&7));for(unsigned x=0;x<8;++x)if(row&(1u<<x))t.removed[x]|=bit;}
    };
#ifdef GFY_RX_PROF
    const double setupMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-started).count();
#endif
    // Measured partial-tile predicate cost varies sharply across the spatial
    // window. Keep fixed output blocks, but let the existing worker width pull
    // them dynamically so one boundary-heavy slice cannot hold up the job.
    // Each block still owns exactly the same stage/count slots; reduction and
    // boundary emission stay in deterministic block/key order.
    std::atomic<std::size_t> nextBlock{0};
    parallel(blocks,std::size_t(1),[&](std::size_t,std::size_t,std::size_t worker){
        for(std::size_t block;(block=nextBlock.fetch_add(1,std::memory_order_relaxed))<blocks;){auto& sum=counts[block];
#ifdef GFY_RX_PROF
            auto& prof=sum.profile;prof.worker=worker;
#endif
            for(std::size_t i=block*blockTiles;i<std::min(tileCount,(block+1)*blockTiles);++i){
#ifdef GFY_RX_PROF
                ProfileTimer lookupTimer(prof.lookupNs);
#endif
                const Cell key=plan.key(i),base{key.x*8,key.y*8,key.z*8};auto& staged=plan.stages[i];
                const Tile* previous=grid.findTile(key);if(previous)staged.finalMask=*previous;
                std::uint64_t all=~std::uint64_t(0);for(auto word:staged.finalMask.removed)all&=word;
#ifdef GFY_RX_PROF
                lookupTimer.finish();
#endif
                if(all==~std::uint64_t(0)){++sum.oldFull;continue;}
#ifdef GFY_RX_PROF
                ProfileTimer predicateTimer(prof.predicateNs);
#endif
                const Vec center=grid.origin+Vec{double(base.x+4),double(base.y+4),double(base.z+4)}*grid.pitch;
                const int coarseCertificate=coarse[coarseIndex(key)];
                const int certificate=coarseCertificate>0?1:cut.classifyBox(center,Vec{3.5,3.5,3.5}*grid.pitch);
                if(certificate<0){++sum.empty;continue;}
                Cell lo,hi;for(int a=0;a<3;++a){(&lo.x)[a]=std::max(low[a],base[a]);(&hi.x)[a]=std::min(high[a],base[a]+7);}
                // Fresh starts as the analytic cut mask, then becomes cut&~old.
                auto& damage=staged.fresh;
                if(certificate>0){++sum.full;
                    const unsigned zMask=byteSpan(lo.z-base.z,hi.z-base.z);std::uint64_t yz=0;
                    for(int y=lo.y-base.y;y<=hi.y-base.y;++y)yz|=std::uint64_t(zMask)<<(y*8);
                    for(int x=lo.x-base.x;x<=hi.x-base.x;++x)damage.removed[x]=yz;
                }else{++sum.unresolved;
#ifdef GFY_RX_PROF
                    ProfileTimer partialTimer(prof.partialNs);
#endif
                    for(int i0=lo[u];i0<=hi[u];++i0)for(int j0=lo[v];j0<=hi[v];++j0){
                        Cell c=lo;(&c.x)[u]=i0;(&c.x)[v]=j0;const int n=hi[axis]-lo[axis]+1;
                        // Already removed samples cannot add fresh damage. A
                        // whole old column needs no analytic predicate at all.
                        unsigned oldRow=0;
                        if(previous){if(axis==2)oldRow=unsigned((staged.finalMask.removed[c.x&7]>>((c.y&7)*8))&255u);
                        else for(int a=lo[axis];a<=hi[axis];++a){Cell q=c;(&q.x)[axis]=a;if(((staged.finalMask.removed[q.x&7]>>((q.y&7)*8+(q.z&7)))&1u))oldRow|=1u<<unsigned(a-base[axis]);}}
                        const unsigned span=byteSpan(lo[axis]-base[axis],hi[axis]-base[axis]);
                        if((oldRow&span)==span)continue;
                        if(lanes){ // the column's cells at once; the few undecided ones through the filter and contains()
#ifdef GFY_RX_PROF
                            ++prof.laneColumns;
#endif
                            const int shift=lo[axis]-base[axis];unsigned inside=0,open=0;const int how=columns.column(grid.center(c),n,inside,open);(void)how;
#ifdef GFY_RX_PROF
                            if(how==0)++prof.wholeColumns;else if(how==2)++prof.angleColumns;
#endif
                            unsigned row=inside<<shift;open=(open<<shift)&~oldRow;
                            for(;open;open&=open-1){const unsigned bit=firstBit(open);(&c.x)[axis]=base[axis]+int(bit);
                                const Vec p=grid.center(c);int in=filtering?filter.classify(p):-1;
#ifdef GFY_RX_PROF
                                if(in<0)++prof.containsCalls;else ++prof.filterCalls;
#endif
                                if(in<0)in=cut.contains(p)?1:0;
                                if(in)row|=1u<<bit;}
                            if(row)setRow(damage,c,row);
                            continue;}
#ifdef GFY_RX_PROF
                        ++prof.segmentCalls;
#endif
                        const int certain=cut.classifySegment(grid.center(c),step,n);if(certain<0)continue;
                        unsigned row=0;if(certain>0)row=byteSpan(lo[axis]-base[axis],hi[axis]-base[axis]);
                        else for(int a=lo[axis];a<=hi[axis];++a){if(oldRow&(1u<<unsigned(a-base[axis])))continue;(&c.x)[axis]=a;
                            const Vec p=grid.center(c);int inside=filtering?filter.classify(p):-1;
#ifdef GFY_RX_PROF
                            if(inside<0)++prof.containsCalls;else ++prof.filterCalls;
#endif
                            if(inside<0)inside=cut.contains(p)?1:0;
                            if(inside)row|=1u<<unsigned(a-base[axis]);}
                        if(row)setRow(damage,c,row);
                    }
                }
                if(grid.ownership){ // a piece session (PieceSeed.h): other pieces' tissue is never erased (Grid::erase)
                    const auto& own=*grid.ownership;const auto& an=*grid.anatomy;
                    auto ownedBits=[&](const auto& columns,const auto& spans,int x,int y)->unsigned{
                        auto col=std::lower_bound(columns.begin(),columns.end(),std::make_pair(x,y),[](const auto& a,const std::pair<int,int>& b){return a.y<b.second||(a.y==b.second&&a.x<b.first);});
                        if(col==columns.end()||col->x!=x||col->y!=y)return 0u;unsigned bits=0;
                        for(auto s=spans.begin()+col->first,e=s+col->count;s!=e;++s){const int z0=std::max(s->lo,base.z),z1=std::min(s->hi,base.z+8);
                            for(int z=z0;z<z1;++z)bits|=1u<<unsigned(z-base.z);}
                        return bits;};
                    for(int x=lo.x;x<=hi.x;++x){auto& word=damage.removed[x&7];if(!word)continue;const int ax=coordinate(x,0);
                        for(int y=lo.y;y<=hi.y;++y){const unsigned row=unsigned((word>>((y&7)*8))&255u);if(!row)continue;
                            const unsigned owned=ownedBits(own.body_columns,own.body_spans,x,y)|ownedBits(own.cloth_columns,own.cloth_spans,x,y);
                            if((row&~owned)==0)continue;
                            unsigned solid=0;const int ay=coordinate(y,1);
                            if(ax>=0&&ay>=0&&std::uint32_t(ax)<an.size[0]&&std::uint32_t(ay)<an.size[1]){const auto column=std::size_t(ay)*an.size[0]+unsigned(ax);
                                auto begin=an.runs.begin()+an.offsets[column],end=an.runs.begin()+an.offsets[column+1];
                                for(unsigned bits=row&~owned;bits;bits&=bits-1){const unsigned b=firstBit(bits);const int az=coordinate(base.z+int(b),2);
                                    if(az<0||std::uint32_t(az)>=an.size[2])continue;
                                    auto run=std::lower_bound(begin,end,az,[](const auto& r,int z){return int(r.hi)<=z;});
                                    if(run!=end&&int(run->lo)<=az&&run->label)solid|=1u<<b;}}
                            word&=~(std::uint64_t(solid)<<((y&7)*8));}}
                }
                unsigned fresh=0;for(unsigned w=0;w<8;++w){damage.removed[w]&=~staged.finalMask.removed[w];staged.finalMask.removed[w]|=damage.removed[w];fresh+=popcount(damage.removed[w]);}
#ifdef GFY_RX_PROF
                predicateTimer.finish();
#endif
                if(!fresh)continue;sum.mask+=fresh;++sum.changed;
#ifdef GFY_RX_PROF
                ProfileTimer countTimer(prof.countNs);
#endif
                int uniform=-1;
                if(fieldAvailable){std::array<int,3> al,ah;for(int a=0;a<3;++a){al[a]=coordinate(lo[a],a);ah[a]=coordinate(hi[a],a);}
                    uniform=fullTileUniformSlot(*countField,al,ah);}
                if(uniform>=0){if(uniform){sum.solid+=fresh;sum.slot[unsigned(uniform)]+=fresh;}
#ifdef GFY_RX_PROF
                    ++prof.uniformTiles;if(!uniform)++prof.uniformAir;
#endif
                    continue;}
                if(grid.anatomy){const auto& an=*grid.anatomy;
                    // Count mixed tiles in the anatomy's native z-run direction.
                    // A run cursor is shared by the eight samples of a byte;
                    // no per-voxel source-volume binary search or float sampler.
                    for(int x=lo.x;x<=hi.x;++x){const auto word=damage.removed[x&7];if(!word)continue;
                        const int ax=coordinate(x,0);if(ax<0||std::uint32_t(ax)>=an.size[0])continue;
                        for(int y=lo.y;y<=hi.y;++y){const unsigned row=unsigned((word>>((y&7)*8))&255u);if(!row)continue;
#ifdef GFY_RX_PROF
                            ++prof.rleRows;
#endif
                            const int ay=coordinate(y,1);if(ay<0||std::uint32_t(ay)>=an.size[1])continue;
                            const auto column=std::size_t(ay)*an.size[0]+unsigned(ax);
                            auto begin=an.runs.begin()+an.offsets[column],end=an.runs.begin()+an.offsets[column+1];if(begin==end)continue;
                            const int azFirst=coordinate(base.z+int(firstBit(row)),2);
                            auto run=std::lower_bound(begin,end,azFirst,[](const auto& r,int z){return int(r.hi)<=z;});
                            if(zConsecutive){ // anatomy z = zStart + (cell z - low.z): count each run's overlap with the row at once
                                const int zOrigin=zStart+(base.z-low[2]); // anatomy z of the row's bit 0
                                unsigned inRange=row;{const int b0=-zOrigin,b1=int(an.size[2])-zOrigin; // bits with 0<=az<size
                                    for(int b=0;b<8;++b)if(b<b0||b>=b1)inRange&=~(1u<<b);}
                                const int azLast=zOrigin+int(lastBit(row));
                                for(;run!=end&&int(run->lo)<=azLast;++run){if(!run->label)continue;
                                    const int b0=std::max(0,int(run->lo)-zOrigin),b1=std::min(7,int(run->hi)-1-zOrigin);if(b0>b1)continue;
                                    const unsigned n=popcount(std::uint64_t(inRange&byteSpan(b0,b1)));
#ifdef GFY_RX_PROF
                                    prof.rleCells+=n;
#endif
                                    if(n){sum.solid+=n;sum.slot[slotOf(run->label)]+=n;}}
                                continue;}
                            for(unsigned bits=row;bits;bits&=bits-1){const int az=coordinate(base.z+int(firstBit(bits)),2);
#ifdef GFY_RX_PROF
                                ++prof.rleCells;
#endif
                                if(az<0||std::uint32_t(az)>=an.size[2])continue;
                                while(run!=end&&int(run->hi)<=az)++run;
                                if(run!=end&&int(run->lo)<=az&&run->label){++sum.solid;++sum.slot[slotOf(run->label)];}
                            }
                        }
                    }
                }else{
                    // Legacy immutable base bitset has no anatomy interval API.
                    for(unsigned x=0;x<8;++x)for(auto bits=damage.removed[x];bits;bits&=bits-1){const unsigned b=firstBit(bits);Cell c{base.x+int(x),base.y+int(b/8),base.z+int(b%8)};
                        if(const auto label=grid.materialBase(c)){++sum.solid;++sum.slot[slotOf(label)];}}
                }
            }
        }
    });
    std::size_t changed=0;
    for(const auto& sum:counts){plan.freshMask+=sum.mask;plan.freshSolid+=sum.solid;changed+=sum.changed;
        plan.fullTiles+=sum.full;plan.emptyTiles+=sum.empty;plan.unresolvedTiles+=sum.unresolved;plan.oldFullTiles+=sum.oldFull;
        for(unsigned s=0;s<64;++s)plan.freshBySlot[s]+=sum.slot[s];}
    auto now=std::chrono::steady_clock::now();plan.classify_ms=std::chrono::duration<double,std::milli>(now-started).count();
#ifdef GFY_RX_PROF
    // Durations are elapsed time accumulated by each worker, not additive
    // wall time. partial_ms is INCLUDED in predicate_ms; never sum them.
    // One line per call, written in one piece: planTileTunnel runs concurrently on the parallel tear lanes.
    {Profile total;std::map<std::size_t,int> workers;
    for(const auto& sum:counts){const auto& p=sum.profile;workers[p.worker]=1;
        total.lookupNs+=p.lookupNs;total.predicateNs+=p.predicateNs;total.partialNs+=p.partialNs;total.countNs+=p.countNs;
        total.segmentCalls+=p.segmentCalls;total.laneColumns+=p.laneColumns;total.wholeColumns+=p.wholeColumns;total.angleColumns+=p.angleColumns;total.containsCalls+=p.containsCalls;total.filterCalls+=p.filterCalls;total.rleRows+=p.rleRows;total.rleCells+=p.rleCells;total.uniformTiles+=p.uniformTiles;total.uniformAir+=p.uniformAir;}
    std::size_t full=0,empty=0,unresolved=0,oldFull=0;for(const auto& sum:counts){full+=sum.full;empty+=sum.empty;unresolved+=sum.unresolved;oldFull+=sum.oldFull;}
    char line[768];std::snprintf(line,sizeof line,"tile_countprof call setup_ms=%.3f classify_wall_ms=%.3f staged_tiles=%zu coarse_empty=%zu coarse_full=%zu full=%zu empty=%zu unresolved=%zu old_full=%zu"
        " lookup_ms=%.3f predicate_ms=%.3f partial_ms=%.3f anatomy_ms=%.3f segments=%zu lane_columns=%zu whole_columns=%zu angle_columns=%zu contains=%zu filtered=%zu rle_rows=%zu rle_cells=%zu uniform_tiles=%zu uniform_air=%zu workers=%zu radius=%.4f pen=%.4f\n",
        setupMs,plan.classify_ms,tileCount,plan.coarseEmptyTiles,plan.coarseFullTiles,full,empty,unresolved,oldFull,total.lookupNs/1e6,total.predicateNs/1e6,total.partialNs/1e6,total.countNs/1e6,
        total.segmentCalls,total.laneColumns,total.wholeColumns,total.angleColumns,total.containsCalls,total.filterCalls,total.rleRows,total.rleCells,total.uniformTiles,total.uniformAir,workers.size(),double(cut.radius),double(cut.penetration));
    std::cerr.write(line,std::streamsize(std::strlen(line)));std::cerr.flush();}
#endif
    plan.scratchBytes=fixedBytes;plan.supported=true;
    if(!plan.freshSolid)return plan; // identical all-air/sub-cell no-op admission
    if(changed>(maxScratchBytes-fixedBytes)/sizeof(Cell))return unsupported("touched_scratch_budget");
    plan.touched.reserve(changed);
    for(std::size_t i=0;i<tileCount;++i){std::uint64_t any=0;for(auto w:plan.stages[i].fresh.removed)any|=w;if(any)plan.touched.push_back(plan.key(i));}
    const auto boundaryStart=std::chrono::steady_clock::now();
    auto finalTile=[&](Cell t)->const Tile*{return plan.finalTile(grid,t);};
    // Only CELL masks are generated. The six word-neighbor equations match
    // the old exact boundary; there are no unit-face records or triangles.
    auto boundaryWords=[&](std::size_t i,auto&& visit){
        const auto& stage=plan.stages[i];std::uint64_t any=0;for(auto word:stage.fresh.removed)any|=word;if(!any)return;
        const Cell t=plan.key(i);const auto& w=stage.finalMask.removed;
        const Tile* xm=finalTile({t.x-1,t.y,t.z}),*xp=finalTile({t.x+1,t.y,t.z}),*ym=finalTile({t.x,t.y-1,t.z}),*yp=finalTile({t.x,t.y+1,t.z}),*zm=finalTile({t.x,t.y,t.z-1}),*zp=finalTile({t.x,t.y,t.z+1});
        constexpr std::uint64_t z0=0x0101010101010101ull,z7=z0<<7;
        for(unsigned x=0;x<8;++x){const auto fresh=stage.fresh.removed[x];if(!fresh)continue;const auto c=w[x];
            const std::uint64_t nxm=x?w[x-1]:(xm?xm->removed[7]:0),nxp=x<7?w[x+1]:(xp?xp->removed[0]:0);
            const std::uint64_t nym=(c<<8)|(ym?(ym->removed[x]>>56):0),nyp=(c>>8)|(yp?((yp->removed[x]&255u)<<56):0);
            const std::uint64_t nzm=((c<<1)&~z0)|(zm?((zm->removed[x]&z7)>>7):0),nzp=((c>>1)&~z7)|(zp?((zp->removed[x]&z0)<<7):0);
            const auto boundary=fresh&~(nxm&nxp&nym&nyp&nzm&nzp);if(boundary)visit(t,x,boundary);
        }
    };
    std::vector<std::size_t> offsets(blocks+1);
    parallel(blocks,std::size_t(1),[&](std::size_t first,std::size_t last,std::size_t){for(std::size_t b=first;b<last;++b){std::size_t n=0;
        for(std::size_t i=b*blockTiles;i<std::min(tileCount,(b+1)*blockTiles);++i)boundaryWords(i,[&](Cell,unsigned,std::uint64_t bits){n+=popcount(bits);});offsets[b+1]=n;}});
    for(std::size_t b=0;b<blocks;++b)offsets[b+1]+=offsets[b];
    const auto used=fixedBytes+plan.touched.size()*sizeof(Cell);
    if(offsets.back()>(maxScratchBytes-used)/sizeof(Cell))return unsupported("boundary_scratch_budget");
    plan.boundary.resize(offsets.back());plan.scratchBytes=used+plan.boundary.size()*sizeof(Cell);
    parallel(blocks,std::size_t(1),[&](std::size_t first,std::size_t last,std::size_t){for(std::size_t b=first;b<last;++b){std::size_t at=offsets[b];
        for(std::size_t i=b*blockTiles;i<std::min(tileCount,(b+1)*blockTiles);++i)boundaryWords(i,[&](Cell t,unsigned x,std::uint64_t bits){
            for(;bits;bits&=bits-1){const unsigned bit=firstBit(bits);plan.boundary[at++]={t.x*8+int(x),t.y*8+int(bit/8),t.z*8+int(bit%8)};}});
    }});
    plan.boundary_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-boundaryStart).count();
    return plan;
}
// ---- Checkerboard edge contacts, one tile word at a time (LiveDamageWorker.cpp resolveEdgeContacts) ----
// One round of resolveEdgeContacts looks at every frontier cell c (a removed cell) and each of its 12 incident voxel
// edges: the edge is a contact when c's two in-plane face neighbours of that edge are valid and not removed and the
// diagonal cell is valid and removed (the four-cell test with c removed); the repair cell is the one of the two
// neighbours the cutter intersects that lies closest to the cutter point (ties: the lower of the edge's cell indices
// n=ia*2+ib). This finds the same edges with 64-bit tile-word shifts over the frontier's tiles (grids without piece
// ownership: there Grid::removed is the stored bit) and evaluates intersects() only at the candidates. The mask is
// read-only here, and the result is a set: identical to the per-cell loop.
namespace edge_contact_detail {
constexpr std::uint64_t z0=0x0101010101010101ull,z7=0x8080808080808080ull;
inline std::uint64_t shiftZ(std::uint64_t x,std::uint64_t next,int dz)noexcept{
    return dz==0?x:dz>0?(((x>>1)&~z7)|((next&z0)<<7)):(((x<<1)&~z0)|((next&z7)>>7));}
inline std::uint64_t shiftY(std::uint64_t x,std::uint64_t next,int dy)noexcept{
    return dy==0?x:dy>0?((x>>8)|((next&0xFFull)<<56)):((x<<8)|(next>>56));}
}
inline bool edgeWordsEnabled(){static const bool on=[]{const char* e=std::getenv("GFY_EDGE_WORDS");return !e||std::atoi(e)!=0;}();return on;}
template<class Grid,class Cell,class Vec,class Intersects,class Parallel>
std::vector<Cell> edgeContactRepairs(const Grid& g,const std::vector<Cell>& frontier,const Vec& cutter,Intersects&& intersects,Parallel&& parallel){
    using namespace edge_contact_detail;using tile_tunnel_detail::firstBit;
    using Tile=typename std::decay<decltype(g.tiles.begin()->second)>::type;
    // Frontier cells grouped by tile (key, 512-bit mask). The planner's boundary list is contiguous per tile, so the
    // map is consulted once per run.
    struct Group {Cell key;std::array<std::uint64_t,8> mask;};std::vector<Group> groups;
    struct KeyHash {std::size_t operator()(const Cell& c)const noexcept{return (std::size_t(c.x)*73856093u)^(std::size_t(c.y)*19349663u)^(std::size_t(c.z)*83492791u);}};
    {std::unordered_map<Cell,std::size_t,KeyHash> index;Cell lastKey{INT_MIN,INT_MIN,INT_MIN};std::size_t current=0;
     for(const Cell& c:frontier){const Cell k=Grid::tileKey(c);
        if(!(k==lastKey)){lastKey=k;const auto found=index.find(k);
            if(found==index.end()){current=groups.size();index.emplace(k,current);groups.push_back({k,{}});}else current=found->second;}
        const unsigned bit=Grid::local(c);groups[current].mask[bit/64]|=std::uint64_t(1)<<(bit%64);}}
    const int TX=(g.size.x+7)/8,TY=(g.size.y+7)/8,TZ=(g.size.z+7)/8;
    // Validity bits of local row x of tile k (cells inside the grid).
    auto validWord=[&](Cell k,int x)->std::uint64_t{
        if(k.x<0||k.y<0||k.z<0||k.x>=TX||k.y>=TY||k.z>=TZ||k.x*8+x>=g.size.x)return 0;
        const int ny=std::min(8,g.size.y-k.y*8),nz=std::min(8,g.size.z-k.z*8);if(ny<=0||nz<=0)return 0;
        const std::uint64_t row=nz>=8?0xFFull:((1ull<<nz)-1);std::uint64_t w=0;for(int y=0;y<ny;++y)w|=row<<(y*8);return w;};
    std::vector<std::vector<Cell>> parts(64);std::atomic<std::size_t> next{0};
    parallel(groups.size(),std::size_t(1),[&](std::size_t,std::size_t,std::size_t worker){
        auto& out=parts[worker%parts.size()];
        for(std::size_t gi;(gi=next.fetch_add(1,std::memory_order_relaxed))<groups.size();){const Group& grp=groups[gi];const Cell t=grp.key;
            const Tile* around[3][3][3];
            for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)for(int dz=-1;dz<=1;++dz){
                around[dx+1][dy+1][dz+1]=g.findTile(Cell{t.x+dx,t.y+dy,t.z+dz});}
            const bool interior=t.x>=1&&t.y>=1&&t.z>=1&&(t.x+2)*8<=g.size.x&&(t.y+2)*8<=g.size.y&&(t.z+2)*8<=g.size.z;
            // Removed (and valid) bits of the cells at offset (dx,dy,dz) from each cell of the tile, row by row.
            auto gather=[&](int dx,int dy,int dz,std::array<std::uint64_t,8>& removed,std::array<std::uint64_t,8>& valid){
                for(int x=0;x<8;++x){const int xs=x+dx,ox=xs<0?-1:(xs>7?1:0),xl=xs&7;
                    auto word=[&](int oy,int oz)->std::uint64_t{const Tile* p=around[ox+1][oy+1][oz+1];return p?p->removed[xl]:0;};
                    removed[x]=shiftY(shiftZ(word(0,0),word(0,dz),dz),shiftZ(word(dy,0),word(dy,dz),dz),dy);
                    if(interior)valid[x]=~0ull;
                    else{auto vw=[&](int oy,int oz){return validWord(Cell{t.x+ox,t.y+oy,t.z+oz},xl);};
                        valid[x]=shiftY(shiftZ(vw(0,0),vw(0,dz),dz),shiftZ(vw(dy,0),vw(dy,dz),dz),dy);}}};
            // open[axis][s]: the face neighbour at -1/+1 along axis is valid and not removed.
            std::array<std::uint64_t,8> open[3][2];
            for(int a=0;a<3;++a)for(int s=0;s<2;++s){int d[3]={0,0,0};d[a]=s?1:-1;std::array<std::uint64_t,8> r,v;gather(d[0],d[1],d[2],r,v);
                for(int x=0;x<8;++x)open[a][s][x]=v[x]&~r[x];}
            for(int axis=0;axis<3;++axis){const int a=(axis+1)%3,b=(axis+2)%3;
                for(int da=0;da<=1;++da)for(int db=0;db<=1;++db){
                    std::array<std::uint64_t,8> cand;std::uint64_t any=0;
                    for(int x=0;x<8;++x){cand[x]=grp.mask[x]&open[a][da][x]&open[b][db][x];any|=cand[x];}
                    if(!any)continue;
                    int d[3]={0,0,0};d[a]=da?1:-1;d[b]=db?1:-1;std::array<std::uint64_t,8> r,v;gather(d[0],d[1],d[2],r,v);
                    for(int x=0;x<8;++x)for(std::uint64_t bits=cand[x]&v[x]&r[x];bits;bits&=bits-1){const unsigned bit=firstBit(bits);
                        const Cell c{t.x*8+x,t.y*8+int(bit/8),t.z*8+int(bit%8)};
                        // q[n], n=ia*2+ib, spans c and its neighbours toward da/db; the open ones are the a- and b-neighbours.
                        Cell na=c,nb=c;(&na.x)[a]+=da?1:-1;(&nb.x)[b]+=db?1:-1;
                        const int ia=da*2+(1-db),ib=(1-da)*2+db;const Cell* order[2]={ia<ib?&na:&nb,ia<ib?&nb:&na};
                        const Cell* best=nullptr;double bestDistance=1e100;
                        for(const Cell* q:order)if(intersects(*q)){const Vec dv=g.center(*q)-cutter;const double distance=dot(dv,dv);if(distance<bestDistance){bestDistance=distance;best=q;}}
                        if(best)out.push_back(*best);}}}
        }});
    std::vector<Cell> add;for(auto& p:parts)add.insert(add.end(),p.begin(),p.end());
    std::sort(add.begin(),add.end());add.erase(std::unique(add.begin(),add.end()),add.end());
#ifdef GFY_EDGE_VERIFY
    { // test builds: the former per-cell four-cell test must choose exactly the same repair cells
        std::vector<Cell> reference;
        for(const Cell& c:frontier){bool open[3][2];
            for(int a=0;a<3;++a)for(int s=0;s<2;++s){Cell n=c;(&n.x)[a]+=s?1:-1;open[a][s]=g.valid(n)&&!g.removed(n);}
            for(int axis=0;axis<3;++axis){int a=(axis+1)%3,b=(axis+2)%3;
                for(int da=0;da<=1;++da)for(int db=0;db<=1;++db){if(!open[a][da]||!open[b][db])continue;
                    Cell e=c;(&e.x)[a]+=da;(&e.x)[b]+=db;Cell q[4];bool removed[4];bool eligible=true;
                    for(int ia=0;ia<=1;++ia)for(int ib=0;ib<=1;++ib){int n=ia*2+ib;q[n]=e;(&q[n].x)[a]+=ia-1;(&q[n].x)[b]+=ib-1;if(!g.valid(q[n])){eligible=false;break;}removed[n]=g.removed(q[n]);}
                    if(!eligible||removed[0]!=removed[3]||removed[1]!=removed[2]||removed[0]==removed[1])continue;
                    int best=-1;double bestDistance=1e100;for(int n=0;n<4;++n)if(!removed[n]&&intersects(q[n])){const Vec d=g.center(q[n])-cutter;const double distance=dot(d,d);if(distance<bestDistance){bestDistance=distance;best=n;}}
                    if(best>=0)reference.push_back(q[best]);}}}
        std::sort(reference.begin(),reference.end());reference.erase(std::unique(reference.begin(),reference.end()),reference.end());
        if(reference!=add)throw std::runtime_error("edge_words_verify");
    }
#endif
    return add;
}
} // namespace painter_detail
