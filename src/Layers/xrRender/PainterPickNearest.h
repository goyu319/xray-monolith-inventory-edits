#pragma once
// triggerhitch (2026-11-15): the nearest posed-mesh face along a ray over ALL bones - the entry face of a received shot
// (impact particles, Lua level.painter_native_surface_hit, once per pellet) and the exit face (painter_exit_surface).
//
// The old answer (painter_exit_surface::nearest over CKinematics::PickBone): for bone 0..count-1 PickBone(bound) is asked
// again, the bound set to its last answer, until it answers nothing. PickBone answers the FIRST face (children in order,
// each bone's face list in order) whose culled ray test gives dist < bound and whose skin-ownership test accepts it, and
// it skins the three corners of every face it visits. So every pellet re-skinned every listed face of the whole body
// (88k faces on GFY 1.06, each listed once per influencing bone, plus one rescan per record) on the main thread.
//
// The old answers are exactly the running records of ONE pass over (bone, child, face) in that order with the strict
// test dist < bound: an answer lies after the previous one (every earlier face was already >= the larger bound or refused
// by the deterministic ownership test). scan() makes that pass with:
//  - the same face lists, order, per-bone visibility test, CDB::TestRayTri, strict dist < bound and ownership test;
//  - the same corners: each vertex posed once by the same get_pos_bones and the same bone render matrices (the caller's
//    memo is checked byte for byte against every bone matrix, the frame, the geometry generation and the children's arrays);
//  - the same refusals: a non-finite or negative record, more than 1024 PickBone calls (counted as the old loop counts:
//    one call per record plus one per bone);
//  - a face list, or a run of 32 consecutive faces of it, is skipped only when its posed box, grown by 5 mm (relative
//    beyond 1 m from the model origin), does not meet the ray line before bound + that margin (behind the start included):
//    none of its faces can then pass dist < bound - the float error of the ray test is micrometres at body scale. A box
//    with a non-finite corner is never skipped. Skipping keeps the order of every face that is tested.
// Engine-independent (V needs x, y, z) so the offline check runs this same code.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>
#include <xmmintrin.h>
#include <emmintrin.h>

namespace painter_pick_nearest {
constexpr float margin = 0.005f;
constexpr unsigned max_calls = 1024; // the old loop's budget
constexpr std::size_t run = 32;      // faces under one small box

struct Bounds {
    float lo[3], hi[3], extent; // extent: the largest |coordinate| of the box
    bool finite, empty;
    void clear() {
        const float inf = std::numeric_limits<float>::infinity();
        lo[0] = lo[1] = lo[2] = inf;
        hi[0] = hi[1] = hi[2] = -inf;
        extent = 0.f;
        finite = true;
        empty = true;
    }
    template <class V> void add(const V& p) {
        const float c[3] = {p.x, p.y, p.z};
        if (!(std::isfinite(c[0]) && std::isfinite(c[1]) && std::isfinite(c[2]))) { finite = false; return; }
        for (unsigned a = 0; a < 3; ++a) {
            if (c[a] < lo[a]) lo[a] = c[a];
            if (c[a] > hi[a]) hi[a] = c[a];
            const float m = std::fabs(c[a]);
            if (m > extent) extent = m;
        }
        empty = false;
    }
};

struct Box {
    const void* key = nullptr;
    std::size_t size = 0;
    bool ready = false;
    Bounds all{};
    std::vector<Bounds> runs; // faces [i*run, (i+1)*run) of the list
};

// x, y, z of a posed vertex in one register (no read past it)
template <class V> inline __m128 load3(const V& p) {
    static_assert(sizeof(V) == 3 * sizeof(float), "posed vertices are three floats");
    const float* f = reinterpret_cast<const float*>(&p);
    return _mm_movelh_ps(_mm_castpd_ps(_mm_load_sd(reinterpret_cast<const double*>(f))), _mm_load_ss(f + 2));
}

// The posed boxes of one face list. false: an index or vertex out of range (the caller answers with the old loop).
// finite_posed: every posed vertex of the child is finite (the caller checked), so the corners need no test.
template <class V>
bool bound_list(Box& b, const std::uint16_t* faces, std::size_t n, const V* posed, std::uint32_t vertices,
                const std::uint16_t* indices, std::size_t index_count, bool finite_posed) {
    if (b.ready && b.key == faces && b.size == n) return true;
    b.ready = false;
    b.all.clear();
    b.runs.resize((n + run - 1) / run);
    for (std::size_t r = 0; r < b.runs.size(); ++r) {
        Bounds& part = b.runs[r];
        part.clear();
        const std::size_t end = (r + 1) * run < n ? (r + 1) * run : n;
        if (finite_posed) {
            __m128 lo = _mm_set1_ps(std::numeric_limits<float>::infinity()), hi = _mm_set1_ps(-std::numeric_limits<float>::infinity());
            for (std::size_t i = r * run; i < end; ++i) {
                const std::size_t base = std::size_t(faces[i]) * 3;
                if (base + 2 >= index_count) return false;
                for (unsigned k = 0; k < 3; ++k) {
                    const std::uint32_t v = indices[base + k];
                    if (v >= vertices) return false;
                    const __m128 p = load3(posed[v]);
                    lo = _mm_min_ps(lo, p);
                    hi = _mm_max_ps(hi, p);
                }
            }
            alignas(16) float l[4], h[4];
            _mm_store_ps(l, lo);
            _mm_store_ps(h, hi);
            for (unsigned a = 0; a < 3; ++a) {
                part.lo[a] = l[a];
                part.hi[a] = h[a];
                part.extent = std::fmax(part.extent, std::fmax(std::fabs(l[a]), std::fabs(h[a])));
            }
            part.empty = end <= r * run;
        } else for (std::size_t i = r * run; i < end; ++i) {
            const std::size_t base = std::size_t(faces[i]) * 3;
            if (base + 2 >= index_count) return false;
            for (unsigned k = 0; k < 3; ++k) {
                const std::uint32_t v = indices[base + k];
                if (v >= vertices) return false;
                part.add(posed[v]);
            }
        }
        if (!part.finite) b.all.finite = false;
        if (!part.empty) {
            for (unsigned a = 0; a < 3; ++a) {
                if (part.lo[a] < b.all.lo[a]) b.all.lo[a] = part.lo[a];
                if (part.hi[a] > b.all.hi[a]) b.all.hi[a] = part.hi[a];
            }
            if (part.extent > b.all.extent) b.all.extent = part.extent;
            b.all.empty = false;
        }
    }
    b.key = faces;
    b.size = n;
    b.ready = true;
    return true;
}

// The ray's constants for the box test.
struct Line {
    double s[3], d[3], len, scale;
    bool usable;
    template <class V> Line(const V& S, const V& D) {
        s[0] = S.x; s[1] = S.y; s[2] = S.z;
        d[0] = D.x; d[1] = D.y; d[2] = D.z;
        usable = true;
        for (unsigned a = 0; a < 3; ++a)
            if (!std::isfinite(s[a]) || !std::isfinite(d[a])) usable = false;
        len = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        if (!(len > 1e-6)) usable = false;
        scale = 1.0;
        for (unsigned a = 0; a < 3; ++a) scale = std::fmax(scale, std::fabs(s[a]));
    }
    // Can a face inside the box give a ray parameter below `bound`? (The line S + t D for every t < bound + margin.)
    bool may_hit(const Bounds& b, float bound) const {
        if (!b.finite || !usable || !std::isfinite(bound)) return true;
        if (b.empty) return false; // no corner at all
        const double grow = double(margin) * std::fmax(scale, double(b.extent));
        double near_t = -std::numeric_limits<double>::infinity(), far_t = double(bound) + grow / len;
        for (unsigned a = 0; a < 3; ++a) {
            const double lo = double(b.lo[a]) - grow, hi = double(b.hi[a]) + grow;
            if (d[a] == 0.0) {
                if (s[a] < lo || s[a] > hi) return false;
                continue;
            }
            double t0 = (lo - s[a]) / d[a], t1 = (hi - s[a]) / d[a];
            if (t0 > t1) { const double t = t0; t0 = t1; t1 = t; }
            if (t0 > near_t) near_t = t0;
            if (t1 < far_t) far_t = t1;
            if (near_t > far_t) return false;
        }
        return true;
    }
};

struct Record {
    bool found = false;
    std::uint32_t child = 0;
    std::uint16_t face = 0;
    float dist = 0.f;
};

// Model:
//   bool visible(std::uint16_t bone);  std::uint32_t children();
//   const std::uint16_t* faces(std::uint16_t bone, std::uint32_t child, std::size_t& n);
//   const V* posed(std::uint32_t child);  std::uint32_t vertices(std::uint32_t child);
//   const std::uint16_t* indices(std::uint32_t child);  std::size_t index_count(std::uint32_t child);
//   Box& box(std::uint32_t child, std::uint16_t bone);  bool finite(std::uint32_t child);  (every posed vertex finite)
//   bool test(V* tri, float& u, float& v, float& dist);   (the culled ray test of PickBone, S and D bound inside)
//   bool accept(std::uint32_t child, std::uint16_t face, float u, float v);
// Returns -1: cannot answer (use the old loop), 0: no face / the old loop's refusal, 1: found (tri = model space corners).
template <class V, class Model>
int scan(Model& m, const V& S, const V& D, float range, std::uint16_t count, Record& out, V* tri) {
    unsigned calls = 0;
    float bound = range;
    out = Record{};
    const Line line(S, D);
    for (std::uint16_t bone = 0; bone < count; ++bone) {
        if (m.visible(bone)) {
            const std::uint32_t children = m.children();
            for (std::uint32_t c = 0; c < children; ++c) {
                std::size_t n = 0;
                const std::uint16_t* faces = m.faces(bone, c, n);
                if (!n) continue;
                const V* posed = m.posed(c);
                const std::uint16_t* idx = m.indices(c);
                Box& box = m.box(c, bone);
                if (!bound_list(box, faces, n, posed, m.vertices(c), idx, m.index_count(c), m.finite(c))) return -1;
                if (!line.may_hit(box.all, bound)) continue;
                for (std::size_t r = 0; r < box.runs.size(); ++r) {
                    if (!line.may_hit(box.runs[r], bound)) continue;
                    const std::size_t end = (r + 1) * run < n ? (r + 1) * run : n;
                    for (std::size_t i = r * run; i < end; ++i) {
                        const std::size_t base = std::size_t(faces[i]) * 3;
                        V t[3] = {posed[idx[base]], posed[idx[base + 1]], posed[idx[base + 2]]};
                        float u = 0.f, v = 0.f, dist = std::numeric_limits<float>::max();
                        if (!(m.test(t, u, v, dist) && dist < bound)) continue;
                        if (!m.accept(c, faces[i], u, v)) continue;
                        // a record = one PickBone call of the old loop that answered this face
                        if (++calls > max_calls) return 0;
                        if (!std::isfinite(dist) || dist < 0.f) return 0;
                        bound = dist;
                        out.found = true;
                        out.child = c;
                        out.face = faces[i];
                        out.dist = dist;
                        tri[0] = t[0];
                        tri[1] = t[1];
                        tri[2] = t[2];
                    }
                }
            }
        }
        if (++calls > max_calls) return 0; // the call that found nothing nearer on this bone
    }
    return out.found ? 1 : 0;
}
} // namespace painter_pick_nearest
