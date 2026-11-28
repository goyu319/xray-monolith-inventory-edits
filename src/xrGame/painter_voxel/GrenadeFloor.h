// Transient cut-depth field in the plane facing the incoming blast. Each ray
// cuts from its entry to exactly one floor. The existing bore predicate still
// clips every sample, including the inlet, mouth and distal cap.
#pragma once

struct Tunnel;
struct GrenadeBite {
    Tunnel* capture{};
    std::uint64_t seed{}, blast{};
    Vec limbAxis{0,1,0};
    const painter_detail::TissueField* display{};
    double depthLimit{1.};
    double inlet{};
};

struct GrenadeFloor {
    Vec u{}, v{};
    double half{}, invStep{};
    int width{};
    std::vector<float> depths;

    static double phase(std::uint64_t seed, unsigned lane) {
        return double(core::quantMix(seed + lane * 0x9e3779b97f4a7c15ull) >> 11)
             * (6.283185307179586 / 9007199254740992.);
    }

    // Smooth interpolation of hashed lattice samples, with no repeating bands.
    static double grain(double x,double y,std::uint64_t seed) {
        const auto i=static_cast<std::int64_t>(std::floor(x)),j=static_cast<std::int64_t>(std::floor(y));
        const double fx=x-i,fy=y-j,a=fx*fx*(3.-2.*fx),b=fy*fy*(3.-2.*fy);
        auto node=[&](std::int64_t u,std::int64_t v){
            const auto h=core::quantMix(seed^(std::uint64_t(u)*0xc2b2ae3d27d4eb4full)^(std::uint64_t(v)*0x9e3779b97f4a7c15ull));
            return double(h>>11)*(2./9007199254740992.)-1.;
        };
        return (node(i,j)*(1.-a)+node(i+1,j)*a)*(1.-b)+(node(i,j+1)*(1.-a)+node(i+1,j+1)*a)*b;
    }

    template<class Sample,class Overlap>
    void prepare(const Vec& entry, const Vec& dir, double radius, double reach,
                 double pitch, const GrenadeBite& bite, Sample sample, Overlap overlap) {
        u = unit(cross(dir, std::abs(dir.x) < .8 ? Vec{1,0,0} : Vec{0,1,0}));
        v = cross(dir,u);
        const double twist = phase(bite.seed,1);
        const Vec cutAxis = u*std::cos(twist) + v*std::sin(twist);
        const Vec across = cross(dir,cutAxis);
        Vec along = bite.limbAxis;
        if (dot(along,along) < .5) along = Vec{0,1,0};
        Vec side = cross(along,Vec{0,0,1});
        side = dot(side,side) > .04 ? unit(side) : unit(cross(along,Vec{1,0,0}));
        Vec side2 = cross(along,side);
        // The group directions of the fitted muscle shader in the limb frame.
        static const double group[8][3] = {
            {.99619,.08716,0},{.81915,.43939,.36868},{.96593,-.24321,-.08852},
            {.34202,-.32139,.88302},{.64279,.38302,-.66341},{.90631,-.366, .21131},
            {.08716,-.34072,-.93611},{.98481,.03015,.17101}};
        Vec crossFibre[9];
        for (int k=0;k<9;++k) {
            const Vec fibre = k==8 ? along : along*group[k][0]+side*group[k][1]+side2*group[k][2];
            const Vec transverse = cross(dir,fibre);
            crossFibre[k] = dot(transverse,transverse)>.02 ? unit(transverse) : across;
        }
        // Sample a continuous field once per bite. Resolution follows the
        // removal grid and has a fixed upper bound for the largest mode.
        half = radius * 1.04;
        width = std::clamp(int(std::ceil(2*half/std::max(.0025,pitch)))+1, 9, 73);
        const double step=2*half/(width-1);
        invStep=1./step;
        depths.resize(std::size_t(width)*width);
        // Identical native sample distances and direction offsets for every floor ray.
        const double sampleStep=std::max(.002,std::min(.004,pitch*1.5));
        std::vector<std::pair<double,Vec>> samples;
        for(double z=-bite.inlet;z<=reach+.012;z+=sampleStep)
            samples.emplace_back(z,dir*z);
        core::parallelChunks(std::size_t(width),8,[&](std::size_t first,std::size_t last,std::size_t){
        for(int j=int(first);j<int(last);++j) for(int i=0;i<width;++i) {
            const double x=i*step-half,y=j*step-half;
            const Vec q=u*x+v*y, base=entry+q;
            const double s=dot(q,cutAxis)/radius, t=dot(q,across)/radius;
            // A skewed fissure with chipped margins. Neighbouring bites share
            // their material relief in the same rest-space blast frame.
            const double A=base.x+.37*base.z,B=base.y-.44*base.z;
            const double broad=grain(A*46,B*46,bite.blast);
            const double ridge=grain(A*92+.37*broad,B*92+.74*broad,bite.blast+1);
            const double nick=grain(A*198,B*198,bite.seed);
            const double centre=.20*broad+.12*ridge+.16*s;
            const double stripWidth=.47+.13*broad+.11*ridge+.065*nick;
            double outline=std::min(1.09-std::abs(s),stripWidth-std::abs(t-centre));
            // A secondary branch of the tear turns across the main fissure.
            const double branch=std::min(.63-std::abs(t+.21),.30+.085*ridge-std::abs(s+.28-.64*t-.12*broad));
            outline=std::max(outline,branch);
            if(x*x+y*y<radius*radius && overlap(base,dir,radius,reach))
                outline=std::max(outline,.24+.12*broad+.06*ridge);
            const auto at=std::size_t(j)*width+i;
            if (outline<=0) { depths[at]=float(std::max(-reach,outline*4.*reach)); continue; }
            const double rim=std::min(1.,outline*5.);
            double bottom=reach*rim*(.62+.21*broad+.13*ridge+.035*nick);

            unsigned floorSlot=0;
            double firstBone=reach+1, lastTissue=0;
            bool entered=false, backedByAnatomy=false, foundBack=false;
            // Scan from the exterior start. A padded garment wall can lie
            // wholly before the first load-bearing anatomy on this ray.
            // Value 80 is drawn solid. Only literal 0 and 16 are empty.
            for(const auto& probe:samples) {
                const double z=probe.first;
                const unsigned label=sample(base+probe.second);
                if(label!=0 && label!=16) {
                    entered=true;
                    lastTissue=z;
                    if(!(label&64u))backedByAnatomy=true;
                    if(z<=bottom+.003)floorSlot=label&63u;
                    if((label&63u)==2 || (label&63u)==21)firstBone=std::min(firstBone,z);
                } else if(entered && z-lastTissue>=.008) { foundBack=true;break; }
            }
            if(entered && !backedByAnatomy){depths[at]=float(-reach);continue;}
            if(floorSlot==5 || (floorSlot>=27 && floorSlot<=34)) {
                const Vec crosswise=crossFibre[floorSlot==5 ? 8 : floorSlot-27];
                const Vec tangent=cross(dir,crosswise);
                const double acrossGrain=dot(base,crosswise)/.0037+.91*ridge+1.32*broad;
                const double alongGrain=dot(base,tangent)/.014+.28*nick;
                const double fibres=grain(acrossGrain,alongGrain,bite.blast+7);
                bottom+=rim*std::min(.0035,.075*reach)*fibres;
            } else if(floorSlot==15 || floorSlot==1) {
                const double lobes=std::abs(grain(A*138+.4*ridge,B*138+.3*broad,bite.blast+13));
                bottom+=rim*std::min(.006,.13*reach)*(1.-2.*lobes);
            } else {
                bottom+=rim*std::min(.006,.11*reach)*nick;
            }
            if(firstBone<reach)bottom=std::min(bottom,firstBone+.0015+.003*std::abs(nick));
            if(foundBack)
                bottom=std::min(bottom,std::max(0.,lastTissue-std::max(.004,2.*pitch)));
            depths[at]=float(std::clamp(bottom,0.,std::min(reach,bite.depthLimit)));
        }
        });
    }

    bool keepsCut(const Vec& offset,double t) const {
        const double x=(dot(offset,u)+half)*invStep, y=(dot(offset,v)+half)*invStep;
        if(x<0 || y<0 || x>=width-1 || y>=width-1)return false;
        const int i=int(x),j=int(y);
        const double a=x-i,b=y-j;
        const float* p=depths.data()+std::size_t(j)*width+i;
        const double d0=p[0]+a*(p[1]-p[0]), d1=p[width]+a*(p[width+1]-p[width]);
        const double depth=d0+b*(d1-d0);
        return depth>0 && t<=depth;
    }
};
