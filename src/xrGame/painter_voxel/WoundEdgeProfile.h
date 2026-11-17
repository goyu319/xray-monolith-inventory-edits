#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>

namespace gfy_live {
// A connected irregular bore, rather than independent random voxel deletions.
// The ammo radius remains the outer bound. Tables are built once per impact;
// the inner 65% radius needs no angular lookup during voxel traversal.
class WoundEdgeProfile {
public:
    static constexpr double minimum_radius = .65;
    static constexpr unsigned samples = 128;
    WoundEdgeProfile() = default;
    explicit WoundEdgeProfile(std::uint64_t seed,double variation=1.) : enabled_(seed != 0&&std::isfinite(variation)&&variation>0.) {
        if(!enabled_)return;
        // Broad asymmetric tears and smaller edge nicks use different angular
        // scales. Three low-amplitude sine waves produced an almost circular
        // outline even though it was technically classified as irregular.
        for(auto& table:radius_){
            std::array<double,7> broad;std::array<double,19> tears;
            std::array<double,47> nicks;
            const double offset=phase(seed);
            for(auto& r:broad)r=phase(seed)/tau*2.-1.;
            for(auto& r:tears)r=phase(seed)/tau*2.-1.;
            for(auto& r:nicks)r=phase(seed)/tau*2.-1.;
            double low=2.,high=-2.;
            for(unsigned i=0;i<samples;++i){
                const double position=double(i)/samples;
                table[i]=.86+.085*std::sin(tau*position+offset)+
                    .12*noise(broad,position)+.075*noise(tears,position)+
                    .025*noise(nicks,position);
                low=std::min(low,table[i]);high=std::max(high,table[i]);
            }
            // Keep the ammo's exact maximum reach while ensuring each impact
            // has visible asymmetry, even when the random bands cancel out.
            const double span=std::max(high-low,1e-12);
            for(unsigned i=0;i<samples;++i)
                table[i]=std::clamp(minimum_radius+(1.-minimum_radius)*(table[i]-low)/span,
                    minimum_radius,1.);
            // 100% retains the exact existing tables and seed stream. Larger
            // values deepen nicks, but never invalidate the 65% inner bound.
            if(variation!=1.)for(unsigned i=0;i<samples;++i)
                table[i]=std::clamp(1.-(1.-table[i])*std::clamp(variation,0.,2.),minimum_radius,1.);
            table[samples]=table[0];
        }
        // Per-sector bounds of radiusMultiplier over both depth tables.
        for(unsigned i=0;i<samples;++i){
            sector_low_[i]=std::min({radius_[0][i],radius_[0][i+1],radius_[1][i],radius_[1][i+1]});
            sector_high_[i]=std::max({radius_[0][i],radius_[0][i+1],radius_[1][i],radius_[1][i+1]});
            for(unsigned t=0;t<2;++t){table_low_[t][i]=std::min(radius_[t][i],radius_[t][i+1]);table_high_[t][i]=std::max(radius_[t][i],radius_[t][i+1]);}
        }
        // Circular sparse range table: conservative arc queries occur for many
        // cells of the same shot. Build extrema once, before rasterization.
        // Copies of a cutter share only immutable bounds, never mutable state.
        auto ranges=std::make_shared<RangeTable>();
        for(unsigned i=0;i<samples;++i)(*ranges)[0][i]={table_low_[0][i],table_high_[0][i],table_low_[1][i],table_high_[1][i]};
        for(unsigned level=1;level<8;++level)for(unsigned i=0;i<samples;++i)
            (*ranges)[level][i]=mergeRange((*ranges)[level-1][i],(*ranges)[level-1][(i+(1u<<(level-1)))&(samples-1)]);
        ranges_=std::move(ranges);
    }
    // Conservative bounds [low,high] of radiusMultiplier(x,y,d) for every depth
    // fraction d. Uses a polynomial angle whose error against std::atan2 is
    // far below angle_error (checked by the kernel tests), then takes every
    // table sector that the widened angle interval touches. Returns false when
    // the direction is too short to bound; callers then evaluate exactly.
    bool sectorBounds(double x,double y,double& low,double& high) const noexcept {
        if(!enabled_)return false;
        const double ax=std::abs(x),ay=std::abs(y);const double big=std::max(ax,ay);
        if(!(big>1e-300)||!std::isfinite(big))return false;
        const double angle=approximateAngle(x,y);
        const double c0=(angle-angle_error)*(samples/tau),c1=(angle+angle_error)*(samples/tau);
        int first=int(std::floor(c0)),last=int(std::floor(c1));
        low=2.;high=-2.;
        for(int k=first;k<=last;++k){const unsigned i=unsigned(k)&(samples-1);
            low=std::min(low,sector_low_[i]);high=std::max(high,sector_high_[i]);}
        return true;
    }
    // Bounds over every direction on the straight 2-D segment (x0,y0)-(x1,y1),
    // which must stay away from the origin (caller-checked); the swept angle is
    // the minor arc between its endpoint angles.
    bool arcBounds(double x0,double y0,double x1,double y1,double& low,double& high) const noexcept {
        if(!enabled_||!ranges_)return false;
        for(double v:{x0,y0,x1,y1})if(!std::isfinite(v))return false;
        if(std::max(std::abs(x0),std::abs(y0))<1e-300||std::max(std::abs(x1),std::abs(y1))<1e-300)return false;
        const double a0=approximateAngle(x0,y0);double d=approximateAngle(x1,y1)-a0;
        if(d>tau*.5)d-=tau;if(d< -tau*.5)d+=tau;if(std::abs(d)>tau*.25)return false;
        const double from=std::min(a0,a0+d)-angle_error,to=std::max(a0,a0+d)+angle_error;
        const int first=int(std::floor(from*(samples/tau))),last=int(std::floor(to*(samples/tau)));
        const auto bounds=rangeBounds(first,last);
        low=std::min(bounds.fl,bounds.ll);high=std::max(bounds.fh,bounds.lh);
        return true;
    }
    // Like arcBounds, but for depth fractions within [d0,d1] (0<=d0<=d1<=1):
    // radiusMultiplier = first*(1-d)+last*d with first/last each within their
    // own table's sector range, so its extremes lie at d0 or d1.
    bool arcDepthBounds(double x0,double y0,double x1,double y1,double d0,double d1,double& low,double& high) const noexcept {
        if(!enabled_||!ranges_||!(d0>=0&&d1<=1&&d0<=d1))return false;
        for(double v:{x0,y0,x1,y1})if(!std::isfinite(v))return false;
        if(std::max(std::abs(x0),std::abs(y0))<1e-300||std::max(std::abs(x1),std::abs(y1))<1e-300)return false;
        const double a0=approximateAngle(x0,y0);double d=approximateAngle(x1,y1)-a0;
        if(d>tau*.5)d-=tau;if(d< -tau*.5)d+=tau;if(std::abs(d)>tau*.25)return false;
        const double from=std::min(a0,a0+d)-angle_error,to=std::max(a0,a0+d)+angle_error;
        const int first=int(std::floor(from*(samples/tau))),last=int(std::floor(to*(samples/tau)));
        const auto bounds=rangeBounds(first,last);
        const double fl=bounds.fl,fh=bounds.fh,ll=bounds.ll,lh=bounds.lh;
        low=std::min(fl*(1-d0)+ll*d0,fl*(1-d1)+ll*d1);high=std::max(fh*(1-d0)+lh*d0,fh*(1-d1)+lh*d1);
        return true;
    }
    static constexpr double angle_error=1e-4;
    // Angle in (-pi,pi] from an odd minimax arctangent polynomial (|error| < 2e-6 rad).
    static double approximateAngle(double x,double y) noexcept {
        const double ax=std::abs(x),ay=std::abs(y);const bool swap=ay>ax;
        const double r=swap?ax/ay:ay/ax,r2=r*r;
        double a=r*(0.99997726+r2*(-0.33262347+r2*(0.19354346+r2*(-0.11643287+r2*(0.05265332+r2*(-0.01172120))))));
        if(swap)a=tau*.25-a;if(x<0)a=tau*.5-a;if(y<0)a=-a;
        return a;
    }
    bool enabled() const noexcept {return enabled_;}
    // Exact semantic data for immutable accepted-cut history; derived bounds
    // and sparse range acceleration are intentionally not retained.
    const std::array<std::array<double,samples+1>,2>& radiusTables()const noexcept{return radius_;}
    double radiusMultiplier(double x,double y,double depth_fraction=0.) const noexcept {
        if(!enabled_)return 1.;
        double angle=std::atan2(y,x);if(angle<0)angle+=tau;
        const double coordinate=angle*(samples/tau);
        const unsigned index=std::min(unsigned(coordinate),samples-1);
        const double fraction=coordinate-index;
        const double first=radius_[0][index]+(radius_[0][index+1]-radius_[0][index])*fraction;
        const double last=radius_[1][index]+(radius_[1][index+1]-radius_[1][index])*fraction;
        // Morph along the bore instead of extruding one outline into straight
        // parallel channels. Every section retains the same connected core.
        return first+(last-first)*std::clamp(depth_fraction,0.,1.);
    }
private:
    struct Range {double fl,fh,ll,lh;};
    static_assert(samples>0&&samples<=256&&(samples&(samples-1))==0,
        "Circular range tables require a power-of-two sample count no greater than 256");
    using RangeTable=std::array<std::array<Range,samples>,8>;
    std::shared_ptr<const RangeTable> ranges_;
    static Range mergeRange(const Range&a,const Range&b) noexcept {
        return {std::min(a.fl,b.fl),std::max(a.fh,b.fh),std::min(a.ll,b.ll),std::max(a.lh,b.lh)};
    }
    Range rangeBounds(int first,int last) const noexcept {
        const unsigned count=std::min(unsigned(last-first+1),samples);
        unsigned level=0,n=count;
        if(n>=16){level+=4;n>>=4;}if(n>=4){level+=2;n>>=2;}if(n>=2)++level;
        return mergeRange((*ranges_)[level][unsigned(first)&(samples-1)],
            (*ranges_)[level][unsigned(first+int(count-(1u<<level)))&(samples-1)]);
    }
    static constexpr double tau=6.283185307179586476925286766559;
    bool enabled_=false;
    std::array<std::array<double,samples+1>,2> radius_{};
    std::array<double,samples> sector_low_{},sector_high_{};
    std::array<std::array<double,samples>,2> table_low_{},table_high_{};
    template<std::size_t N>
    static double noise(const std::array<double,N>& knots,double position) noexcept {
        const double coordinate=position*N;
        const auto index=static_cast<unsigned>(coordinate);
        double blend=coordinate-index;
        blend=blend*blend*(3.-2.*blend);
        return knots[index%N]+(knots[(index+1)%N]-knots[index%N])*blend;
    }
    static double phase(std::uint64_t& state) noexcept {
        state+=0x9e3779b97f4a7c15ull;
        auto z=state;z=(z^(z>>30))*0xbf58476d1ce4e5b9ull;
        z=(z^(z>>27))*0x94d049bb133111ebull;z^=z>>31;
        return double(z>>11)*(tau/9007199254740992.);
    }
};
}
