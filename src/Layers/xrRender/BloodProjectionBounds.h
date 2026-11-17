#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

// Optional conservative broad phase. Blocks address the original ordered index
// stream, so duplicates, winding, and the exact projection arithmetic survive.
namespace BloodProjectionBounds
{
constexpr unsigned BlockVertices = 96; // 32 consecutive triangles
constexpr unsigned MaxBones = 8;
struct Block
{
    float lo[3]{}, hi[3]{};
    std::uint16_t bones[MaxBones]{};
    std::uint16_t bone_count = 0; // zero means unbounded: use the original range
};

template<class Vertices, class Indices>
Block Build(const Vertices& vertices, const Indices& indices, size_t first, size_t end)
{
    Block block;
    for (unsigned axis=0; axis<3; ++axis) {
        block.lo[axis]=(std::numeric_limits<float>::max)();
        block.hi[axis]=-(std::numeric_limits<float>::max)();
    }
    for (size_t i=first; i<end; ++i) {
        if (indices[i]>=vertices.size()) return Block{};
        const auto& v=vertices[indices[i]];
        if (v.job<1 || v.job>4) return Block{};
        double weight_sum=0.;
        for (unsigned j=0; j+1<v.job; ++j) {
            const float w=v.weights[j];
            if (!std::isfinite(w) || w<0.f || w>1.f) return Block{};
            weight_sum+=w;
        }
        // 2W is lerp; 3/4W use explicit weights and a chained remainder.
        // Both lie in the bone-transform convex hull, up to the error allowance
        // in Outside. Non-convex/unusual source data keeps the original path.
        if (weight_sum>1.) return Block{};
        for (unsigned axis=0; axis<3; ++axis) {
            if (!std::isfinite(v.p[axis])) return Block{};
            block.lo[axis]=(std::min)(block.lo[axis],v.p[axis]);
            block.hi[axis]=(std::max)(block.hi[axis],v.p[axis]);
        }
        for (unsigned j=0; j<v.job; ++j) {
            if (v.ids[j]>(std::numeric_limits<std::uint16_t>::max)()) return Block{};
            unsigned b=0;
            while (b<block.bone_count && block.bones[b]!=v.ids[j]) ++b;
            if (b==block.bone_count) {
                if (b==MaxBones) return Block{};
                block.bones[block.bone_count++]=std::uint16_t(v.ids[j]);
            }
        }
    }
    return block;
}

template<class Pose, class Matrix>
bool Outside(const Block& block, const Pose& pose, const Matrix& view, float depth)
{
    if (!block.bone_count || block.bone_count>MaxBones || !std::isfinite(depth) || depth<0.f) return false;
    double lo[3]={1.e300,1.e300,1.e300}, hi[3]={-1.e300,-1.e300,-1.e300};
    constexpr double guard=64.*std::numeric_limits<float>::epsilon();
    constexpr double safe=(std::numeric_limits<float>::max)()/16.;
    for (unsigned b=0; b<block.bone_count; ++b) {
        if (block.bones[b]>=pose.size()) return false;
        const auto& m=pose[block.bones[b]];
        for (unsigned axis=0; axis<3; ++axis) {
            double low=m.m[3][axis], high=low, magnitude=std::abs(low);
            for (unsigned j=0; j<3; ++j) {
                const double a=double(m.m[j][axis])*block.lo[j];
                const double c=double(m.m[j][axis])*block.hi[j];
                low+=(std::min)(a,c); high+=(std::max)(a,c);
                magnitude+=(std::max)(std::abs(a),std::abs(c));
            }
            if (!std::isfinite(low) || !std::isfinite(high) || !std::isfinite(magnitude) || magnitude>safe) return false;
            // Covers transform roundoff, 2W lerp and 3/4W weighted sums,
            // including their differently rounded CPU/GPU remainders.
            const double error=guard*(1.+magnitude);
            lo[axis]=(std::min)(lo[axis],low-error);
            hi[axis]=(std::max)(hi[axis],high+error);
        }
    }
    bool separated=false;
    for (unsigned axis=0; axis<3; ++axis) {
        double low=view.m[3][axis], high=low, magnitude=std::abs(low);
        for (unsigned j=0; j<3; ++j) {
            const double a=double(view.m[j][axis])*lo[j];
            const double c=double(view.m[j][axis])*hi[j];
            low+=(std::min)(a,c); high+=(std::max)(a,c);
            magnitude+=(std::max)(std::abs(a),std::abs(c));
        }
        if (!std::isfinite(low) || !std::isfinite(high) || !std::isfinite(magnitude) || magnitude>safe) return false;
        const double error=guard*(1.+magnitude);
        const double extent=axis==2 ? double(depth) : 1.;
        separated|=low-error>extent || high+error<-extent;
    }
    return separated;
}
}
