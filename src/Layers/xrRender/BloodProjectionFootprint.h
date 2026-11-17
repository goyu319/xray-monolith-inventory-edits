#pragma once

// Candidate collection only: UV scale and the pixel footprint are unchanged.
// BuildMatrix scales all axes by 1/(0.9*size). Keep the old +/-size axial reach.
namespace BloodProjectionFootprint
{
constexpr float Depth = 1.1111112f;
constexpr float CollectorScale = 1.619f; // > sqrt(0.9^2 + 0.9^2 + 1)

// Outer surface only: some corner of the triangle lies no deeper than OuterDepth(size) behind the contact along the
// mark normal (which points out of the surface, toward the shooter). Mirrored in BloodGpuProjection.cpp.
inline float OuterDepth(float size) { return _max(0.02f, _min(0.04f, size * 0.35f)); }
// A surface call's own depth limit (place_skeleton_blood_surface with its depth argument, 2026-11-12 sever splashes).
// 0 = the rule above. Bounded to 1-25 cm: the most a sever splash needs is a torso (contact 4 cm outside the skin of a
// 15 cm half-width plus the side points of a flat chest seen at 45 degrees: about 24 cm); 25 cm stays below a torso's
// front-to-back thickness with gear. Skin facing away (the far side) is dropped by the face test at any depth; the
// bound limits how far into folds and onto neighbouring parts that face the same way one decal can reach.
constexpr float OuterDepthMin = 0.01f, OuterDepthMax = 0.25f;
inline float OuterDepthOverride(float depth) { return _valid(depth) && depth > 0.f ? _max(OuterDepthMin, _min(OuterDepthMax, depth)) : 0.f; }
inline bool Outer(const Fvector& contact, const Fvector& normal, float size, const Fvector* p, float depth = 0.f)
{
    const float limit = depth > 0.f ? depth : OuterDepth(size);
    for (int k = 0; k < 3; ++k)
        if ((contact.x - p[k].x) * normal.x + (contact.y - p[k].y) * normal.y + (contact.z - p[k].z) * normal.z <= limit) return true;
    return false;
}

inline bool Separated(const Fvector& a, const Fvector& p0, const Fvector& p1, const Fvector& p2)
{
    const float p = p0.x*a.x + p0.y*a.y + p0.z*a.z;
    const float q = p1.x*a.x + p1.y*a.y + p1.z*a.z;
    const float r = p2.x*a.x + p2.y*a.y + p2.z*a.z;
    const float extent = _abs(a.x) + _abs(a.y) + Depth*_abs(a.z);
    return _min(p, _min(q, r)) > extent || _max(p, _max(q, r)) < -extent;
}

inline bool IntersectsProjected(const Fvector& p0, const Fvector& p1, const Fvector& p2)
{
    if (!_valid(p0) || !_valid(p1) || !_valid(p2)) return false;
    if (_min(p0.x, _min(p1.x,p2.x)) > 1.f || _max(p0.x, _max(p1.x,p2.x)) < -1.f ||
        _min(p0.y, _min(p1.y,p2.y)) > 1.f || _max(p0.y, _max(p1.y,p2.y)) < -1.f ||
        _min(p0.z, _min(p1.z,p2.z)) > Depth || _max(p0.z, _max(p1.z,p2.z)) < -Depth) return false;
    Fvector edges[3], axis;
    edges[0].sub(p1,p0); edges[1].sub(p2,p1); edges[2].sub(p0,p2);
    axis.crossproduct(edges[0],edges[1]);
    if (Separated(axis,p0,p1,p2)) return false;
    for (const Fvector& edge : edges)
    {
        axis.set(0.f,-edge.z,edge.y); if (Separated(axis,p0,p1,p2)) return false;
        axis.set(edge.z,0.f,-edge.x); if (Separated(axis,p0,p1,p2)) return false;
        axis.set(-edge.y,edge.x,0.f); if (Separated(axis,p0,p1,p2)) return false;
    }
    return true;
}

inline bool Intersects(const Fmatrix& view, const Fvector* points)
{
    Fvector p[3];
    for (unsigned i=0; i<3; ++i) view.transform_tiny(p[i],points[i]);
    return IntersectsProjected(p[0],p[1],p[2]);
}
}
