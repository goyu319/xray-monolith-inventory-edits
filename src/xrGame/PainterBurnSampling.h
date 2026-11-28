#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

// Enumerates world-space directions; it never accepts or paints a surface.
// The host raycasts from its original emitter and tests the first hit, owner,
// rest-space extent, material and source power on every ray.
namespace painter_burn_sampling {
constexpr double Pi = 3.14159265358979323846;
constexpr double PitchMetres = .0035;

namespace detail {
struct V { double x{},y{},z{}; };
inline V add(V a,V b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
inline V mul(V a,double v){return {a.x*v,a.y*v,a.z*v};}
inline V cross(V a,V b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline double dot(V a,V b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline V unit(V a){const double d=std::sqrt(dot(a,a));return std::isfinite(d)&&d>1.e-12?mul(a,1./d):V{};}
inline void basis(V n,V& u,V& v){
    u=unit(cross(std::abs(n.y)<.90?V{0,1,0}:V{1,0,0},n));v=cross(n,u);
}
inline double rotation(std::uint32_t seed){
    seed^=seed>>16;seed*=0x7feb352du;seed^=seed>>15;seed*=0x846ca68bu;seed^=seed>>16;
    return double(seed)*(2.*Pi/4294967296.);
}
}

// One low-discrepancy discovery direction in the actual receiver's cone. A
// whole sphere uses cosHalfAngle=-1. These are proposals for ignition points.
template<class F> inline void discover(double dx,double dy,double dz,double cosHalfAngle,
        std::uint32_t seed,unsigned index,unsigned count,F&& visit){
    using namespace detail;
    if(!count||index>=count||!std::isfinite(cosHalfAngle))return;
    const V axis=unit({dx,dy,dz});if(dot(axis,axis)<.5)return;
    V u,v;basis(axis,u,v);
    const double cap=std::max(-1.,std::min(1.,cosHalfAngle));
    const double z=1.-(1.-cap)*(double(index)+.5)/double(count);
    const double radial=std::sqrt(std::max(0.,1.-z*z));
    const double phi=rotation(seed)+2.39996322972865332*double(index);
    const V d=add(mul(axis,z),add(mul(u,radial*std::cos(phi)),mul(v,radial*std::sin(phi))));
    visit(d.x,d.y,d.z);
}

// Densely samples only ONE already-ignited patch. Keep its PRNG seed stable
// across pulses so growth retains its sampled centre. Radius is on the target,
// in metres. Each successive ring is <=3.5 mm from its neighbour; tangential
// spacing is also <=3.5 mm. The centre ray is first and is the only ray for
// radius zero. The caller may keep this object across worker slices.
class Patch {
    detail::V centre{},u{},v{};
    double radius{},phase{};
    unsigned ring{},rings{},point{},points{1};
    bool finished{true};
public:
    Patch(double dx,double dy,double dz,double anchorDistance,double radiusMetres,std::uint32_t seed){
        const auto dir=detail::unit({dx,dy,dz});
        if(detail::dot(dir,dir)<.5 || !std::isfinite(anchorDistance)||anchorDistance<=0 ||
           !std::isfinite(radiusMetres)||radiusMetres<0 || radiusMetres>100.)return;
        centre=detail::mul(dir,anchorDistance);detail::basis(dir,u,v);
        radius=radiusMetres;phase=detail::rotation(seed);
        rings=unsigned(std::ceil(radius/PitchMetres));finished=false;
    }
    // Optional genuine posed-world surface normal for oblique hits. A rest
    // normal is unsuitable here. Omitting it uses the source-facing projection.
    void orient(double nx,double ny,double nz){
        const auto n=detail::unit({nx,ny,nz});
        if(!ring&&!point&&detail::dot(n,n)>.5)detail::basis(n,u,v);
    }
    // Continuous footprint shading fills the area between certified samples. Use
    // a fixed disk, including its centre and boundary, irrespective of old radius.
    template<class F> void contacts(unsigned rays,F&& visit) const {
        if(finished||!rays)return;
        for(unsigned i=0;i<rays;++i){
            const double r=rays==1?0.:radius*std::sqrt(double(i)/double(rays-1));
            const double a=phase+2.39996322972865332*double(i);
            const auto q=detail::add(centre,detail::add(detail::mul(u,r*std::cos(a)),detail::mul(v,r*std::sin(a))));
            const auto d=detail::unit(q);if(detail::dot(d,d)>.5)visit(d.x,d.y,d.z);
        }
    }
    bool done()const{return finished;}
    template<class F> bool slice(unsigned maxRays,F&& visit){
        while(!finished&&maxRays--){
            const double r=std::min(radius,double(ring)*PitchMetres);
            const double a=phase+2.39996322972865332*ring+2.*Pi*double(point)/double(points);
            const auto q=detail::add(centre,detail::add(detail::mul(u,r*std::cos(a)),detail::mul(v,r*std::sin(a))));
            const auto d=detail::unit(q);
            if(detail::dot(d,d)>.5)visit(d.x,d.y,d.z);
            if(++point<points)continue;
            if(++ring>rings){finished=true;break;}
            point=0;
            const double nextRadius=std::min(radius,double(ring)*PitchMetres);
            points=std::max(1u,unsigned(std::ceil(2.*Pi*nextRadius/PitchMetres)));
        }
        return finished;
    }
};
}
