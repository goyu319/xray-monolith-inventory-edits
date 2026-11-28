#pragma once
#include <algorithm>
#include <cmath>

namespace PainterFireFront
{
struct Node
{
    float x, y, z, radius, height, fade;
};
struct Sum
{
    float dx = 0.f, dz = 0.f, y = 0.f, r = 0.f, weight = 0.f, strongest = 0.f;
};
struct Shape
{
    float steerX = 0.f, steerZ = 0.f, wider = 0.f;
    float peerX = 0.f, peerY = 0.f, peerZ = 0.f, invRadius = 0.f, join = 0.f;
    float planeX = 0.f, planeZ = 0.f;
};
inline float unit(float x) { return std::max(0.f, std::min(1.f, x)); }

// Call only for a directly linked surface or another lobe on the same visible source.
inline void accumulate(const Node& a, const Node& b, Sum& s)
{
    const float dx = b.x-a.x, dz = b.z-a.z, dy = b.y-a.y;
    const float reach = 1.4f * (a.radius+b.radius);
    if (a.fade <= 0.f || b.fade <= 0.f || reach <= .02f) return;
    const float level = unit(1.f-std::max(0.f,dy)/std::max(.02f,.96f*a.height)) *
                        unit(1.f-std::max(0.f,-dy)/std::max(.02f,.96f*b.height));
    const float w = unit(1.55f*(1.f-(dx*dx+dz*dz)/(reach*reach))) * level * b.fade;
    s.dx += dx*w; s.dz += dz*w; s.y += (b.y + .30f*b.height)*w;
    s.r += std::max(b.radius,.47f*b.height)*w;
    s.weight += w;
    s.strongest = std::max(s.strongest,w);
}

inline Shape resolve(const Node& a, const Sum& s, float time, float windX, float windZ,
                     float uphillX, float uphillZ, float wallX, float wallZ, float body)
{
    Shape out{};
    if (s.weight <= .0001f) return out;
    const float w = unit(s.strongest*1.55f)*a.fade;
    const float dx = s.dx/s.weight, dz = s.dz/s.weight;
    const float invDistance = 1.f/std::max(a.radius,std::sqrt(dx*dx+dz*dz));
    const float gust = .5f + .5f*std::sin(.98f*time+.14f*a.x+.13f*a.z);
    const float wx = windX*(.039f+.014f*gust), wz = windZ*(.039f+.014f*gust);
    const float lateral=std::min(1.f,1.65f/std::max(.01f,a.radius));
    const float pull=.38f*lateral*lateral;
    float sx = pull*dx*invDistance+.35f*lateral*uphillX+wx, sz = pull*dz*invDistance+.35f*lateral*uphillZ+wz;
    // A wall can deflect the hot front along its face, but cannot move its fuel base through the wall.
    const float normalLen = wallX*wallX+wallZ*wallZ;
    if (normalLen>.1f)
    {
        const float into = (sx*wallX+sz*wallZ)/normalLen;
        sx-=wallX*into; sz-=wallZ*into;
    }
    const float limit = body ? .22f : .48f;
    const float scale = (limit/std::max(limit,std::sqrt(sx*sx+sz*sz))) * a.height;
    out.steerX=sx*scale; out.steerZ=sz*scale;
    out.wider = (body ? .15f : .32f*lateral*lateral)*w;
    out.peerX=a.x+dx; out.peerY=s.y/s.weight; out.peerZ=a.z+dz;
    out.invRadius=1.f/std::max(.03f,1.85f*s.r/s.weight);
    out.join=w;
    return out;
}
}
