#pragma once
// wallmarkpick (2026-11-16): the mesh work of CKinematics::AddWallmark (every bullet mark and every blood decal on a body) on
// the posed copy of triggerhitch (PainterPickNearest.h: every vertex posed once by the same get_pos_bones, a box per face list
// and per 32-face run of it).
//
// The old work, per call, on the main thread: (1) the bullet pick: for every bone the ray's OBB test admits and every child,
// PickBone(bound) = the FIRST face of that list with a culled ray hit dist < bound that the ownership test accepts (bound <-
// its dist); (2) a surface decal's look-back: per pass, for every admitted bone and every child receiving marks, PickBone again
// and again (at most 8 answers a list) with the shrinking bound; (3) the decal faces: for every child receiving marks and
// every bone whose OBB meets the mark's sphere, every face of the list is skinned and tested (FillVertices). Every visit
// skinned the three corners again: GFY 1.06 lists 298,683 (face, bone) entries.
//
// Here the same visits in the same order with the same tests on the posed corners; a list, or a run of 32 faces of it, is
// skipped only when its posed box (grown by 5 mm, relative beyond 1 m from the model origin) cannot contain a face that the
// visit would accept:
//  - picks: the box does not meet the ray line before the bound (+ the margin): no face in it passes dist < bound;
//  - decal faces: the box is farther than the mark's reach from the contact (size for the sphere test, size * 1.619 =
//    BloodProjectionFootprint::CollectorScale, the circumscribed radius of the square footprint's box), or wholly beyond one
//    face of the footprint's own box in view space (the footprint test's first test), or wholly deeper behind the contact
//    than the outer limit (BloodProjectionFootprint::Outer) - no face in it can pass the tests a face must pass. A mark with
//    no receiver in its box (the sever splashes of a committed separation) skips the body's runs and costs only the boxes.
// A box with a non-finite corner is never skipped. Answers, records, and the faces of a mark and their order are those of the
// old loops. Engine-independent (V needs x, y, z): the offline check compiles this same file.
#include "PainterPickNearest.h"

namespace painter_wallmark_scan {
using painter_pick_nearest::Bounds;
using painter_pick_nearest::Box;
using painter_pick_nearest::Line;
using painter_pick_nearest::bound_list;
using painter_pick_nearest::margin;
using painter_pick_nearest::run;
constexpr unsigned max_cover_answers = 8; // AddWallmark's look-back: again < 8

// Can a point of the box lie within `reach` of the contact?
struct Ball {
    double c[3], reach, scale;
    bool usable;
    template <class V> Ball(const V& C, float R) {
        c[0] = C.x; c[1] = C.y; c[2] = C.z;
        reach = R;
        usable = std::isfinite(reach) && reach >= 0.0;
        scale = 1.0;
        for (unsigned a = 0; a < 3; ++a) {
            if (!std::isfinite(c[a])) usable = false;
            scale = std::fmax(scale, std::fabs(c[a]));
        }
    }
    bool may_touch(const Bounds& b) const {
        if (!b.finite || !usable) return true;
        if (b.empty) return false;
        const double grow = double(margin) * std::fmax(scale, double(b.extent));
        double d2 = 0.0;
        for (unsigned a = 0; a < 3; ++a) {
            const double lo = double(b.lo[a]) - grow, hi = double(b.hi[a]) + grow;
            const double d = c[a] < lo ? lo - c[a] : (c[a] > hi ? c[a] - hi : 0.0);
            d2 += d * d;
        }
        const double r = reach + grow;
        return d2 <= r * r;
    }
};

// The decal's own box (BloodProjectionFootprint::Intersects: view space |x| <= 1, |y| <= 1, |z| <= Depth; the view is
// BuildMatrix's 1 / (0.9 size) camera at the contact): no corner of a box whose view-space extent lies wholly beyond one face
// of it can give a triangle the footprint test accepts (its first test is exactly that interval test).
struct Footprint {
    double row[3][4]; // view space axis a = row[a][0..2] . p + row[a][3]
    double limit[3];
    template <class M> Footprint(const M& v, float depth) {
        row[0][0] = v._11; row[0][1] = v._21; row[0][2] = v._31; row[0][3] = v._41;
        row[1][0] = v._12; row[1][1] = v._22; row[1][2] = v._32; row[1][3] = v._42;
        row[2][0] = v._13; row[2][1] = v._23; row[2][2] = v._33; row[2][3] = v._43;
        limit[0] = 1.0; limit[1] = 1.0; limit[2] = depth;
    }
    bool may_touch(const Bounds& b) const {
        if (!b.finite) return true;
        if (b.empty) return false;
        const double grow = double(margin) * std::fmax(1.0, double(b.extent));
        for (unsigned a = 0; a < 3; ++a) {
            double centre = row[a][3], half = 0.0, scale = 0.0;
            for (unsigned q = 0; q < 3; ++q) {
                const double c = (double(b.lo[q]) + double(b.hi[q])) * 0.5, h = (double(b.hi[q]) - double(b.lo[q])) * 0.5 + grow;
                centre += row[a][q] * c;
                half += std::fabs(row[a][q]) * h;
                scale += std::fabs(row[a][q]);
            }
            if (!std::isfinite(centre) || !std::isfinite(half)) return true;
            const double slack = 1e-4 * (1.0 + scale); // float rounding of view.transform_tiny
            if (centre - half > limit[a] + slack || centre + half < -limit[a] - slack) return false;
        }
        return true;
    }
};
// BloodProjectionFootprint::Outer: a triangle passes only with a corner p where (contact - p) . normal <= limit; no corner of a
// box whose every point lies deeper than that (+ the margin) can.
struct Front {
    double c[3], n[3], limit;
    template <class V> Front(const V& contact, const V& normal, float depth_limit) {
        c[0] = contact.x; c[1] = contact.y; c[2] = contact.z;
        n[0] = normal.x; n[1] = normal.y; n[2] = normal.z;
        limit = depth_limit;
    }
    bool may_touch(const Bounds& b) const {
        if (!b.finite) return true;
        if (b.empty) return false;
        const double grow = double(margin) * std::fmax(1.0, double(b.extent));
        double nearest = 0.0, scale = 0.0; // the box point with the least depth (contact - p) . n
        for (unsigned a = 0; a < 3; ++a) {
            const double p = n[a] >= 0.0 ? double(b.hi[a]) + grow : double(b.lo[a]) - grow;
            nearest += (c[a] - p) * n[a];
            scale += std::fabs(n[a]);
        }
        if (!std::isfinite(nearest) || !std::isfinite(limit)) return true;
        return nearest <= limit + grow * (1.0 + scale);
    }
};

// A mark's box tests together (a face is taken only when it passes the outer test AND the footprint or sphere test).
struct MarkReach {
    Ball ball;
    Front front;
    Footprint foot;
    bool square;
    template <class V, class M>
    MarkReach(const V& contact, const V& normal, float size, float outer_limit, bool square_footprint, const M& view, float collector, float depth)
        : ball(contact, size * (square_footprint ? collector : 1.f)), front(contact, normal, outer_limit), foot(view, depth), square(square_footprint) {}
    bool may_touch(const Bounds& b) const { return ball.may_touch(b) && front.may_touch(b) && (!square || foot.may_touch(b)); }
};

// Model:
//   std::uint32_t children();  bool receives(std::uint32_t child);   (receives_wallmarks)
//   const std::uint16_t* faces(std::uint16_t bone, std::uint32_t child, std::size_t& n);
//   const V* posed(std::uint32_t child);  std::uint32_t vertices(std::uint32_t child);
//   const std::uint16_t* indices(std::uint32_t child);  std::size_t index_count(std::uint32_t child);
//   Box& box(std::uint32_t child, std::uint16_t bone);  bool finite(std::uint32_t child);
//   bool test(const V& S, const V& D, V* tri, float& u, float& v, float& dist);   (CDB::TestRayTri, culled)
//   bool accept(std::uint32_t child, std::uint16_t face, float u, float v);         (PickBone's ownership test)
// admit(bone): the old loop's bone test for this ray (visible, pickable, ray-OBB).
// All three return -1 before any answer when a face list cannot be boxed (an index out of range): the caller runs the old loop.

template <class V, class Model>
inline bool box_of(Model& m, std::uint32_t c, std::uint16_t bone, const std::uint16_t* faces, std::size_t n, Box*& out) {
    out = &m.box(c, bone);
    return bound_list(*out, faces, n, m.posed(c), m.vertices(c), m.indices(c), m.index_count(c), m.finite(c));
}

// (1) The bullet pick. dist: in = the start bound (flt_max), out = the last answer; tri: its model-space corners.
template <class V, class Model, class Admit>
int first_hits(Model& m, Admit&& admit, const V& S, const V& D, std::uint16_t count, float& dist, V* tri) {
    const Line line(S, D);
    float bound = dist;
    bool found = false;
    const std::uint32_t children = m.children();
    for (std::uint16_t bone = 0; bone < count; ++bone) {
        if (!admit(bone)) continue;
        for (std::uint32_t c = 0; c < children; ++c) {
            std::size_t n = 0;
            const std::uint16_t* faces = m.faces(bone, c, n);
            if (!n) continue;
            Box* box = nullptr;
            if (!box_of<V>(m, c, bone, faces, n, box)) return -1;
            if (!line.may_hit(box->all, bound)) continue;
            const V* posed = m.posed(c);
            const std::uint16_t* idx = m.indices(c);
            bool answered = false;
            for (std::size_t r = 0; r < box->runs.size() && !answered; ++r) {
                if (!line.may_hit(box->runs[r], bound)) continue;
                const std::size_t end = (r + 1) * run < n ? (r + 1) * run : n;
                for (std::size_t i = r * run; i < end; ++i) {
                    const std::size_t base = std::size_t(faces[i]) * 3;
                    V t[3] = {posed[idx[base]], posed[idx[base + 1]], posed[idx[base + 2]]};
                    float u = 0.f, v = 0.f, d = std::numeric_limits<float>::max();
                    if (!(m.test(S, D, t, u, v, d) && d < bound)) continue;
                    if (!m.accept(c, faces[i], u, v)) continue;
                    bound = d; // PickBone's answer for this list: the next child is asked with it
                    found = true;
                    tri[0] = t[0];
                    tri[1] = t[1];
                    tri[2] = t[2];
                    answered = true;
                    break;
                }
            }
        }
    }
    dist = bound;
    return found ? 1 : 0;
}

template <class V, class Model, class Admit>
int cover_pass(Model& m, Admit& admit, const V& from, const V& back, std::uint16_t count, bool refuse, float& nearest, bool& covered) {
    const Line line(from, back);
    const std::uint32_t children = m.children();
    for (std::uint16_t bone = 0; bone < count && !(covered && refuse); ++bone) {
        if (!admit(bone)) continue;
        for (std::uint32_t c = 0; c < children && !(covered && refuse); ++c) {
            if (!m.receives(c)) continue;
            std::size_t n = 0;
            const std::uint16_t* faces = m.faces(bone, c, n);
            if (!n) continue;
            Box* box = nullptr;
            if (!box_of<V>(m, c, bone, faces, n, box)) return -1;
            if (!line.may_hit(box->all, nearest)) continue;
            const V* posed = m.posed(c);
            const std::uint16_t* idx = m.indices(c);
            unsigned answers = 0; // the old loop asked this list again after every answer, at most 8 answers
            for (std::size_t r = 0; r < box->runs.size() && answers < max_cover_answers; ++r) {
                if (!line.may_hit(box->runs[r], nearest)) continue;
                const std::size_t end = (r + 1) * run < n ? (r + 1) * run : n;
                for (std::size_t i = r * run; i < end; ++i) {
                    const std::size_t base = std::size_t(faces[i]) * 3;
                    V t[3] = {posed[idx[base]], posed[idx[base + 1]], posed[idx[base + 2]]};
                    float u = 0.f, v = 0.f, d = std::numeric_limits<float>::max();
                    if (!(m.test(from, back, t, u, v, d) && d < nearest)) continue;
                    if (!m.accept(c, faces[i], u, v)) continue;
                    covered = true;
                    nearest = d;
                    if (refuse) return 1;
                    if (++answers >= max_cover_answers) break;
                }
            }
        }
    }
    return 1;
}

// (2) One look-back pass. nearest: in = the start bound, out = the covering skin's distance; covered: set on any answer;
// refuse: stop at the first answer (a sever splash / blade cut refuses a covered contact). -1 leaves both as they were.
template <class V, class Model, class Admit>
int cover(Model& m, Admit&& admit, const V& from, const V& back, std::uint16_t count, bool refuse, float& nearest_out, bool& covered_out) {
    float nearest = nearest_out;
    bool covered = covered_out;
    const int got = cover_pass<V>(m, admit, from, back, count, refuse, nearest, covered);
    if (got >= 0) {
        nearest_out = nearest;
        covered_out = covered;
    }
    return got;
}

// (3) The faces of one mark: for every child receiving marks, for every test bone in order, the faces of its list in order
// that can reach the contact go to face(child, face id, posed corners) - the caller's exact per-face test and build. A face
// listed under several test bones comes once per list, as before (the old loop added it once per list it passed in).
template <class V, class Model, class Reach, class Face>
int fill(Model& m, const std::uint16_t* bones, std::size_t bone_count, const Reach& reach, Face&& face) {
    const std::uint32_t children = m.children();
    for (std::uint32_t c = 0; c < children; ++c) { // every list boxed first: a refusal adds no face
        if (!m.receives(c)) continue;
        for (std::size_t b = 0; b < bone_count; ++b) {
            std::size_t n = 0;
            const std::uint16_t* faces = m.faces(bones[b], c, n);
            Box* box = nullptr;
            if (n && !box_of<V>(m, c, bones[b], faces, n, box)) return -1;
        }
    }
    for (std::uint32_t c = 0; c < children; ++c) {
        if (!m.receives(c)) continue;
        const V* posed = m.posed(c);
        const std::uint16_t* idx = m.indices(c);
        for (std::size_t b = 0; b < bone_count; ++b) {
            std::size_t n = 0;
            const std::uint16_t* faces = m.faces(bones[b], c, n);
            if (!n) continue;
            Box& box = m.box(c, bones[b]);
            if (!reach.may_touch(box.all)) continue;
            for (std::size_t r = 0; r < box.runs.size(); ++r) {
                if (!reach.may_touch(box.runs[r])) continue;
                const std::size_t end = (r + 1) * run < n ? (r + 1) * run : n;
                for (std::size_t i = r * run; i < end; ++i) {
                    const std::size_t base = std::size_t(faces[i]) * 3;
                    V t[3] = {posed[idx[base]], posed[idx[base + 1]], posed[idx[base + 2]]};
                    face(c, faces[i], t);
                }
            }
        }
    }
    return 1;
}
} // namespace painter_wallmark_scan
