#pragma once
// Mask-only wounds, slice 2 (painter_voxel_mask_only 2): what a detached piece needs on the GPU, built from the decisions the
// separation already made (no wall geometry, no surface partition, no per-piece mesh packing).
//
// 1. The piece's MARKED field, in today's removal-field format (DamageField, 8^3 tiles, one bit per cell), so the renderer binds
//    it exactly as a removal field and slice 1's shader needs no ownership logic:
//      marked_k(c) = removed(c) | c is another piece's cell | (c is free and its nearest piece cell, within `reach`, belongs to
//                    another piece: strictly nearer than any cell of k)
//    free = not removed and no piece's cell (air, garment air, padding not in the topology). Ties keep the cell (both pieces).
//    A free cell with no piece within reach stays kept. Free cells in tiles with NO own cell within reach (tile level) are marked
//    whenever another piece is within reach of the tile (no distance needed there; such cells are > reach from this piece, so
//    marking them only lets a ray pass instead of ending on them). The piece's exterior is hidden where marked; a ray stepped
//    into the volume stops only at kept tissue, which is this piece's (or kept padding standing on it).
// 2. The piece's COVERAGE BOX: the oriented box (principal axes) around its torn cells (kept tissue 6-adjacent to removed or
//    other-piece cells) grown by `boxMargin` cells. Every ray that reaches the torn face crosses a front face of it.
//
// Cost model: tiles (8^3) are classified first. A tile with no other piece within reach of it (tile level, Chebyshev) needs no
// distance: marked = removed. A tile with no own cell within reach: every cell is marked. Only CONTESTED tiles (own and other
// within reach) need distances: Euclidean distances (separable squared EDT, Felzenszwalb-Huttenlocher) on a grid of
// `block`^3-cell blocks (block 1 = exact cell distances) over the bounding box of the contested tiles plus the reach, one
// transform per piece. Torn cells are found with 64-bit row operations on the tiles' bits.
//
// Generic over the caller's data (the engine adapter reads its Topology spans and Grid tiles; the offline harness dense arrays):
//   tileCells(key, piece[512], removed) -> the piece index (0..P-1, or -1) of every cell of tile `key` (bit order of bitOf) and its
//   removed (non-piece) cells;  tissueAt(x,y,z) -> kept tissue (the shader's SOLID class);  parallel(n, body(first,last)).
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <unordered_map>
#include <vector>

namespace painter_mask_pieces {
struct Key { int x, y, z; bool operator==(const Key& o) const noexcept { return x == o.x && y == o.y && z == o.z; } };
struct KeyHash { std::size_t operator()(const Key& k) const noexcept { return (std::size_t(std::uint32_t(k.x)) * 73856093u) ^ (std::size_t(std::uint32_t(k.y)) * 19349663u) ^ (std::size_t(std::uint32_t(k.z)) * 83492791u); } };
using Bits = std::array<std::uint64_t, 8>; // bit ((x&7)*8+(y&7))*8+(z&7), as Grid::Tile / DamageField
using Cells = std::array<signed char, 512>;
struct Box { std::array<double, 3> centre{}, axis[3]{}, lo{}, hi{}; std::size_t torn = 0; bool valid = false; };
struct Piece { std::unordered_map<Key, Bits, KeyHash> marked; Box box; std::size_t markedCells = 0; };
struct Stats { std::size_t tiles = 0, contestedTiles = 0, edtCells = 0; double tileMs = 0, edtMs = 0, markMs = 0, boxMs = 0; };

inline unsigned bitOf(int x, int y, int z) { return unsigned(((x & 7) * 8 + (y & 7)) * 8 + (z & 7)); }

// 1-D squared distance transform (Felzenszwalb-Huttenlocher) of f over n samples; f = 0 at seeds, INF elsewhere.
inline void edt1(const float* f, float* d, int n, std::vector<int>& v, std::vector<float>& z) {
    const float INF = std::numeric_limits<float>::infinity();
    v.resize(std::size_t(n)); z.resize(std::size_t(n) + 1);
    int k = -1;
    for (int q = 0; q < n; ++q) {
        if (f[q] == INF) continue;
        if (k < 0) { k = 0; v[0] = q; z[0] = -INF; z[1] = INF; continue; }
        float s;
        for (;;) {
            const int p = v[std::size_t(k)];
            s = ((f[q] + float(q) * float(q)) - (f[p] + float(p) * float(p))) / (2.f * float(q - p));
            if (s <= z[std::size_t(k)]) { if (--k < 0) break; } else break;
        }
        if (k < 0) { k = 0; v[0] = q; z[0] = -INF; z[1] = INF; continue; }
        ++k; v[std::size_t(k)] = q; z[std::size_t(k)] = s; z[std::size_t(k) + 1] = INF;
    }
    if (k < 0) { for (int q = 0; q < n; ++q) d[q] = INF; return; }
    int j = 0;
    for (int q = 0; q < n; ++q) {
        while (z[std::size_t(j) + 1] < float(q)) ++j;
        const int p = v[std::size_t(j)];
        d[q] = float(q - p) * float(q - p) + f[p];
    }
}

// Cells of tile C (bits) that have a 6-neighbour set in `cut` (C's own bits and its face neighbours' bits; null = none).
inline Bits faceAdjacent(const Bits& c, const Bits* x0, const Bits* x1, const Bits* y0, const Bits* y1, const Bits* z0, const Bits* z1) {
    const std::uint64_t zLow = 0x0101010101010101ull, zHigh = 0x8080808080808080ull;
    Bits out{};
    for (int x = 0; x < 8; ++x) {
        const std::uint64_t w = c[std::size_t(x)];
        std::uint64_t a = ((w >> 1) & ~zHigh) | ((w << 1) & ~zLow);                       // z +- 1 inside the tile
        if (z1) a |= (((*z1)[std::size_t(x)] & zLow) << 7);                                 // z = 7 <- next tile's z = 0
        if (z0) a |= (((*z0)[std::size_t(x)] & zHigh) >> 7);                                // z = 0 <- previous tile's z = 7
        a |= (w >> 8) | (w << 8);                                                          // y +- 1 inside the tile
        if (y1) a |= ((*y1)[std::size_t(x)] & 0xffull) << 56;                               // y = 7 <- next tile's y = 0
        if (y0) a |= ((*y0)[std::size_t(x)] >> 56) & 0xffull;                               // y = 0 <- previous tile's y = 7
        a |= x < 7 ? c[std::size_t(x) + 1] : (x1 ? (*x1)[0] : 0);                          // x +- 1
        a |= x > 0 ? c[std::size_t(x) - 1] : (x0 ? (*x0)[7] : 0);
        out[std::size_t(x)] = a;
    }
    return out;
}

// lo/hi: cell box of the region to consider (hi exclusive); tiles are the global 8^3 blocks.
template<class TileCells, class TissueAt, class Parallel>
std::vector<Piece> build(std::array<int, 3> lo, std::array<int, 3> hi, unsigned P, const std::array<double, 3>& origin, double pitch, double reach, double boxMargin,
                         TileCells&& tileCells, TissueAt&& tissueAt, Parallel&& parallel, Stats& stats,
                         double (*clockMs)() = nullptr, int block = 2,
                         const std::function<void(std::vector<std::uint16_t>&, std::vector<char>&, const std::array<int, 3>&, const std::array<int, 3>&)>* tileSummary = nullptr) {
    block = std::max(1, std::min(block, 4));
    auto now = [&] { return clockMs ? clockMs() : 0.0; };
    std::vector<Piece> out(P);
    if (P < 2 || P > 16) return out;
    const int R = int(std::ceil(reach / pitch));                 // reach in cells
    const int RT = (R + 7) / 8 + 1;                              // reach in tiles (Chebyshev, conservative)
    std::array<int, 3> tlo, thi, tn;
    for (int a = 0; a < 3; ++a) { tlo[a] = lo[a] >> 3; thi[a] = ((hi[a] - 1) >> 3) + 1; tn[a] = thi[a] - tlo[a]; }
    const std::size_t T = std::size_t(tn[0]) * std::size_t(tn[1]) * std::size_t(tn[2]);
    auto ti = [&](int x, int y, int z) { return (std::size_t(x) * std::size_t(tn[1]) + std::size_t(y)) * std::size_t(tn[2]) + std::size_t(z); };
    auto inRegion = [&](int x, int y, int z) { return x >= lo[0] && y >= lo[1] && z >= lo[2] && x < hi[0] && y < hi[1] && z < hi[2]; };
    double t0 = now();
    // ---- tile summary: pieces present (bit k), any removed (the caller may fill it from its sparse data, x-major order)
    std::vector<std::uint16_t> present(T, 0); std::vector<char> anyRemoved(T, 0);
    if (tileSummary && *tileSummary) (*tileSummary)(present, anyRemoved, tlo, tn);
    else parallel(std::size_t(tn[0]), [&](std::size_t first, std::size_t last) {
        Cells pc; Bits rm;
        for (std::size_t tx = first; tx < last; ++tx) for (int ty = 0; ty < tn[1]; ++ty) for (int tz = 0; tz < tn[2]; ++tz) {
            tileCells(Key{tlo[0] + int(tx), tlo[1] + ty, tlo[2] + tz}, pc, rm);
            std::uint16_t m = 0; for (auto p : pc) if (p >= 0) m |= std::uint16_t(1u << unsigned(p));
            std::uint64_t r = 0; for (auto w : rm) r |= w;
            present[ti(int(tx), ty, tz)] = m; anyRemoved[ti(int(tx), ty, tz)] = r ? 1 : 0;
        }
    });
    // ---- pieces within RT tiles (separable OR filter, slabs in parallel)
    std::vector<std::uint16_t> nearMask(present), tmp(T);
    for (int a = 0; a < 3; ++a) {
        parallel(std::size_t(tn[0]), [&](std::size_t first, std::size_t last) {
            for (std::size_t xx = first; xx < last; ++xx) { const int x = int(xx);
                for (int y = 0; y < tn[1]; ++y) for (int z = 0; z < tn[2]; ++z) {
                    std::uint16_t m = 0; const int c[3] = {x, y, z};
                    for (int s = -RT; s <= RT; ++s) { int q[3] = {x, y, z}; q[a] = c[a] + s; if (q[a] < 0 || q[a] >= tn[a]) continue; m |= nearMask[ti(q[0], q[1], q[2])]; }
                    tmp[ti(x, y, z)] = m; } }
        });
        nearMask.swap(tmp);
    }
    stats.tiles = T; stats.tileMs = now() - t0; t0 = now();
    // ---- contested tiles: two or more pieces within reach -> distances over their bounding box (+ R cells of seeds)
    std::array<int, 3> clo{INT32_MAX, INT32_MAX, INT32_MAX}, chi{INT32_MIN, INT32_MIN, INT32_MIN};
    for (int x = 0; x < tn[0]; ++x) for (int y = 0; y < tn[1]; ++y) for (int z = 0; z < tn[2]; ++z) {
        const auto m = nearMask[ti(x, y, z)]; if (!(m & (m - 1))) continue; ++stats.contestedTiles;
        const int c[3] = {x, y, z}; for (int a = 0; a < 3; ++a) { clo[a] = std::min(clo[a], c[a]); chi[a] = std::max(chi[a], c[a]); }
    }
    std::array<int, 3> elo{}, en{}; std::vector<float> dist; // per piece squared distance in blocks over the EDT box
    if (stats.contestedTiles) {
        for (int a = 0; a < 3; ++a) {
            int e0 = std::max(lo[a], (tlo[a] + clo[a]) * 8 - R - 1); e0 -= ((e0 % block) + block) % block;
            const int ehi = std::min(hi[a], (tlo[a] + chi[a] + 1) * 8 + R + 1);
            elo[a] = e0; en[a] = (ehi - e0 + block - 1) / block; }
        const std::size_t N = std::size_t(en[0]) * std::size_t(en[1]) * std::size_t(en[2]); stats.edtCells = N;
        dist.assign(N * P, std::numeric_limits<float>::infinity());
        auto ei = [&](int x, int y, int z) { return (std::size_t(x) * std::size_t(en[1]) + std::size_t(y)) * std::size_t(en[2]) + std::size_t(z); };
        // seeds: every piece cell of the tiles overlapping the EDT box (tiles with no piece skipped)
        const int sx0 = elo[0] >> 3, sx1 = (elo[0] + en[0] * block - 1) >> 3, sy0 = elo[1] >> 3, sy1 = (elo[1] + en[1] * block - 1) >> 3, sz0 = elo[2] >> 3, sz1 = (elo[2] + en[2] * block - 1) >> 3;
        parallel(std::size_t(sx1 - sx0 + 1), [&](std::size_t first, std::size_t last) {
            Cells pc; Bits rm;
            for (std::size_t i = first; i < last; ++i) { const int kx = sx0 + int(i);
                for (int ky = sy0; ky <= sy1; ++ky) for (int kz = sz0; kz <= sz1; ++kz) {
                    if (kx < tlo[0] || ky < tlo[1] || kz < tlo[2] || kx >= thi[0] || ky >= thi[1] || kz >= thi[2]) continue;
                    if (!present[ti(kx - tlo[0], ky - tlo[1], kz - tlo[2])]) continue;
                    tileCells(Key{kx, ky, kz}, pc, rm);
                    for (int u = 0; u < 8; ++u) for (int v = 0; v < 8; ++v) for (int w = 0; w < 8; ++w) {
                        const int p = pc[std::size_t((u * 8 + v) * 8 + w)]; if (p < 0) continue;
                        const int cx = kx * 8 + u - elo[0], cy = ky * 8 + v - elo[1], cz = kz * 8 + w - elo[2];
                        if (cx < 0 || cy < 0 || cz < 0) continue;
                        const int bx = cx / block, by = cy / block, bz = cz / block; if (bx >= en[0] || by >= en[1] || bz >= en[2]) continue;
                        dist[std::size_t(p) * N + ei(bx, by, bz)] = 0.f; } } }
        });
        for (unsigned k = 0; k < P; ++k) {
            float* D = dist.data() + std::size_t(k) * N;
            parallel(std::size_t(en[0]), [&](std::size_t first, std::size_t last) {
                std::vector<int> v; std::vector<float> zz, f(std::size_t(std::max({en[0], en[1], en[2]}))), d(f.size());
                for (std::size_t x = first; x < last; ++x) {
                    for (int y = 0; y < en[1]; ++y) { float* line = D + ei(int(x), y, 0); std::copy(line, line + en[2], f.begin()); edt1(f.data(), line, en[2], v, zz); }
                    for (int z = 0; z < en[2]; ++z) { for (int y = 0; y < en[1]; ++y) f[std::size_t(y)] = D[ei(int(x), y, z)]; edt1(f.data(), d.data(), en[1], v, zz); for (int y = 0; y < en[1]; ++y) D[ei(int(x), y, z)] = d[std::size_t(y)]; }
                }
            });
            parallel(std::size_t(en[1]), [&](std::size_t first, std::size_t last) {
                std::vector<int> v; std::vector<float> zz, f(static_cast<std::size_t>(en[0])), d(f.size());
                for (std::size_t y = first; y < last; ++y) for (int z = 0; z < en[2]; ++z) {
                    for (int x = 0; x < en[0]; ++x) f[std::size_t(x)] = D[ei(x, int(y), z)]; edt1(f.data(), d.data(), en[0], v, zz); for (int x = 0; x < en[0]; ++x) D[ei(x, int(y), z)] = d[std::size_t(x)]; }
            });
        }
    }
    stats.edtMs = now() - t0; t0 = now();
    const float R2 = float(reach / (pitch * block)) * float(reach / (pitch * block)); // in distance-grid units
    const std::size_t N = dist.size() / std::max(1u, P);
    auto ei = [&](int x, int y, int z) { return (std::size_t((x - elo[0]) / block) * std::size_t(en[1]) + std::size_t((y - elo[1]) / block)) * std::size_t(en[2]) + std::size_t((z - elo[2]) / block); };
    auto inEdt = [&](int x, int y, int z) { return x >= elo[0] && y >= elo[1] && z >= elo[2] && (x - elo[0]) / block < en[0] && (y - elo[1]) / block < en[1] && (z - elo[2]) / block < en[2]; };
    // ---- marked bits per piece and tile; cut bits (removed | other pieces' cells) wherever the cells were read
    struct Row { Key key; unsigned k; Bits marked, cut; bool full, cutKnown; };
    std::vector<std::vector<Row>> rows(static_cast<std::size_t>(tn[0]));
    parallel(std::size_t(tn[0]), [&](std::size_t first, std::size_t last) {
        Cells pc; Bits rm;
        for (std::size_t tx = first; tx < last; ++tx) for (int ty = 0; ty < tn[1]; ++ty) for (int tz = 0; tz < tn[2]; ++tz) {
            const std::size_t t = ti(int(tx), ty, tz); const auto near = nearMask[t];
            const Key key{tlo[0] + int(tx), tlo[1] + ty, tlo[2] + tz};
            bool read = false, nearestKnown = false;
            std::array<float, 512> d1, d2; std::array<signed char, 512> l1; // nearest and second-nearest piece distance per cell
            for (unsigned k = 0; k < P; ++k) {
                const std::uint16_t own = std::uint16_t(1u << k), others = std::uint16_t(present[t] & ~own);
                if (!anyRemoved[t] && !others && !(near & ~own)) continue;                 // nothing foreign anywhere near: all kept
                const bool contested = (near & own) && (near & ~own);
                const bool farFromOwn = !(near & own);
                Row row{key, k, {}, {}, false, false};
                // no own cell within reach and another piece near: every cell of the tile is marked
                if (farFromOwn && (near & ~own)) { row.marked.fill(~std::uint64_t(0)); row.full = true; rows[tx].push_back(row); continue; }
                if (!read) { tileCells(key, pc, rm); read = true; }
                bool any = false;
                for (int u = 0; u < 8; ++u) for (int v = 0; v < 8; ++v) for (int w = 0; w < 8; ++w) {
                    const int x = key.x * 8 + u, y = key.y * 8 + v, z = key.z * 8 + w;
                    if (!inRegion(x, y, z)) continue;
                    const unsigned b = unsigned((u * 8 + v) * 8 + w); const int p = pc[b];
                    const bool removed = ((rm[b >> 6] >> (b & 63u)) & 1u) != 0;
                    bool m, cut;
                    if (p >= 0) { m = cut = p != int(k); }
                    else if (removed) { m = cut = true; }
                    else if (farFromOwn) { m = (near & ~own) != 0; cut = false; }             // free, no own cell within reach
                    else if (!contested || !inEdt(x, y, z)) { m = cut = false; }               // free, only own within reach
                    else {
                        if (!nearestKnown) { // once per tile: every cell's nearest and second-nearest piece (P lookups per cell, not P per piece)
                            for (unsigned bb = 0; bb < 512; ++bb) { const int xx = key.x * 8 + int(bb >> 6), yy = key.y * 8 + int((bb >> 3) & 7u), zz = key.z * 8 + int(bb & 7u);
                                float a = std::numeric_limits<float>::infinity(), s = a; signed char la = -1;
                                if (inRegion(xx, yy, zz) && inEdt(xx, yy, zz)) { const std::size_t e = ei(xx, yy, zz);
                                    for (unsigned q = 0; q < P; ++q) { const float dq = dist[std::size_t(q) * N + e]; if (dq < a) { s = a; a = dq; la = static_cast<signed char>(q); } else if (dq < s) s = dq; } }
                                d1[bb] = a; d2[bb] = s; l1[bb] = la; }
                            nearestKnown = true; }
                        // the nearest OTHER piece's distance: the second-nearest when this piece is the nearest (a tie keeps the cell)
                        const float dk = dist[std::size_t(k) * N + ei(x, y, z)], dmin = l1[b] == static_cast<signed char>(k) ? d2[b] : d1[b];
                        m = dmin < dk && dmin <= R2; cut = false;
                    }
                    if (m) { row.marked[b >> 6] |= std::uint64_t(1) << (b & 63u); any = true; }
                    if (cut) row.cut[b >> 6] |= std::uint64_t(1) << (b & 63u);
                }
                row.cutKnown = true;
                if (any) rows[tx].push_back(row);
            }
        }
    });
    std::vector<std::unordered_map<Key, Bits, KeyHash>> cutOf(P);
    parallel(std::size_t(P), [&](std::size_t first, std::size_t last) { for (std::size_t k = first; k < last; ++k) { // one piece per job
        auto& piece = out[k]; std::size_t n = 0, nc = 0;
        for (const auto& list : rows) for (const auto& r : list) if (r.k == k) { ++n; nc += r.cutKnown ? 1 : 0; }
        piece.marked.reserve(n); cutOf[k].reserve(nc);
        for (const auto& list : rows) for (const auto& r : list) { if (r.k != k) continue;
            if (r.full) piece.markedCells += 512; else for (auto w : r.marked) for (; w; w &= w - 1) ++piece.markedCells;
            piece.marked.emplace(r.key, r.marked);
            if (r.cutKnown) cutOf[k].emplace(r.key, r.cut); } } });
    stats.markMs = now() - t0; t0 = now();
    // ---- coverage boxes: kept tissue 6-adjacent to a cut cell (removed or another piece's), found with row operations
    for (unsigned k = 0; k < P; ++k) {
        auto& piece = out[k]; auto& cut = cutOf[k];
        std::vector<Key> candidates;
        for (const auto& t : piece.marked) {
            const Key& key = t.first;
            // a fully marked tile holds no kept cell and lies beyond reach of every own cell: neither it nor a neighbour of it
            // reached only through it can hold this piece's torn face
            bool full = true; for (auto w : t.second) full = full && w == ~std::uint64_t(0);
            if (full) continue;
            candidates.push_back(key);
            const Key nb[6] = {{key.x - 1, key.y, key.z}, {key.x + 1, key.y, key.z}, {key.x, key.y - 1, key.z}, {key.x, key.y + 1, key.z}, {key.x, key.y, key.z - 1}, {key.x, key.y, key.z + 1}};
            for (const auto& q : nb) if (q.x >= tlo[0] && q.y >= tlo[1] && q.z >= tlo[2] && q.x < thi[0] && q.y < thi[1] && q.z < thi[2] && !piece.marked.count(q)) candidates.push_back(q);
        }
        std::sort(candidates.begin(), candidates.end(), [](const Key& a, const Key& b) { return a.x != b.x ? a.x < b.x : (a.y != b.y ? a.y < b.y : a.z < b.z); });
        candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());
        // full tiles next to a candidate: their cut bits (read once)
        {std::vector<Key> need;
         for (const auto& key : candidates) { const Key nb[6] = {{key.x - 1, key.y, key.z}, {key.x + 1, key.y, key.z}, {key.x, key.y - 1, key.z}, {key.x, key.y + 1, key.z}, {key.x, key.y, key.z - 1}, {key.x, key.y, key.z + 1}};
             for (const auto& q : nb) if (!cut.count(q) && piece.marked.count(q)) need.push_back(q); }
         std::sort(need.begin(), need.end(), [](const Key& a, const Key& b) { return a.x != b.x ? a.x < b.x : (a.y != b.y ? a.y < b.y : a.z < b.z); });
         need.erase(std::unique(need.begin(), need.end()), need.end());
         std::vector<Bits> got(need.size());
         parallel(need.size(), [&](std::size_t first, std::size_t last) { Cells pc; Bits rm;
             for (std::size_t i = first; i < last; ++i) { tileCells(need[i], pc, rm); Bits c = rm;
                 for (unsigned b = 0; b < 512; ++b) if (pc[b] >= 0 && pc[b] != int(k)) c[b >> 6] |= std::uint64_t(1) << (b & 63u);
                 got[i] = c; } });
         for (std::size_t i = 0; i < need.size(); ++i) cut.emplace(need[i], got[i]);}
        std::vector<std::vector<std::array<double, 3>>> found(candidates.size());
        parallel(candidates.size(), [&](std::size_t first, std::size_t last) { for (std::size_t ci = first; ci < last; ++ci) {
            const Key t = candidates[ci];
            // piecebox: only the torn face toward the other pieces. A tile with no other piece's cell within reach (nearMask) holds
            // old wounds of this piece - a corpse's bullet holes, whose walls its skin's hidden pixels draw; with them the box of
            // a body that lost a head chunk grew to the whole wounded body, rigid with one bone.
            {const int rx = t.x - tlo[0], ry = t.y - tlo[1], rz = t.z - tlo[2];
             if (rx < 0 || ry < 0 || rz < 0 || rx >= tn[0] || ry >= tn[1] || rz >= tn[2] || !(nearMask[ti(rx, ry, rz)] & std::uint16_t(~(1u << k)))) continue;}
            auto bitsOf = [&](const std::unordered_map<Key, Bits, KeyHash>& m, const Key& q) -> const Bits* { const auto it = m.find(q); return it == m.end() ? nullptr : &it->second; };
            static const Bits zero{};
            const Bits* c0 = bitsOf(cut, t); const Bits* mk = bitsOf(piece.marked, t);
            const Bits adj = faceAdjacent(c0 ? *c0 : zero, bitsOf(cut, {t.x - 1, t.y, t.z}), bitsOf(cut, {t.x + 1, t.y, t.z}), bitsOf(cut, {t.x, t.y - 1, t.z}),
                                          bitsOf(cut, {t.x, t.y + 1, t.z}), bitsOf(cut, {t.x, t.y, t.z - 1}), bitsOf(cut, {t.x, t.y, t.z + 1}));
            for (int u = 0; u < 8; ++u) { std::uint64_t w = adj[std::size_t(u)] & ~(mk ? (*mk)[std::size_t(u)] : 0);
                for (; w; w &= w - 1) {
                    unsigned b = 0; for (std::uint64_t q = w & (~w + 1); q > 1; q >>= 1) ++b;
                    const int x = t.x * 8 + u, y = t.y * 8 + int(b >> 3), z = t.z * 8 + int(b & 7u);
                    if (!inRegion(x, y, z) || !tissueAt(x, y, z)) continue;
                    found[ci].push_back({origin[0] + (x + .5) * pitch, origin[1] + (y + .5) * pitch, origin[2] + (z + .5) * pitch}); } }
        } });
        std::vector<std::array<double, 3>> pts;
        for (auto& f : found) pts.insert(pts.end(), f.begin(), f.end()); // candidate (tile) order: deterministic
        auto& b = piece.box; b.torn = pts.size(); if (pts.size() < 4) continue;
        std::array<double, 3> c{}; for (const auto& p : pts) for (int a = 0; a < 3; ++a) c[a] += p[a];
        for (int a = 0; a < 3; ++a) c[a] /= double(pts.size());
        double C[3][3] = {}; for (const auto& p : pts) for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) C[i][j] += (p[i] - c[i]) * (p[j] - c[j]);
        // Jacobi eigen-decomposition of the 3x3 covariance
        double V[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        for (int sweep = 0; sweep < 32; ++sweep) {
            int p = 0, q = 1; double big = std::abs(C[0][1]);
            if (std::abs(C[0][2]) > big) { p = 0; q = 2; big = std::abs(C[0][2]); }
            if (std::abs(C[1][2]) > big) { p = 1; q = 2; big = std::abs(C[1][2]); }
            if (big < 1e-18) break;
            const double th = .5 * std::atan2(2 * C[p][q], C[q][q] - C[p][p]), cs = std::cos(th), sn = std::sin(th);
            for (int r = 0; r < 3; ++r) { const double a = C[r][p], d = C[r][q]; C[r][p] = cs * a - sn * d; C[r][q] = sn * a + cs * d; }
            for (int r = 0; r < 3; ++r) { const double a = C[p][r], d = C[q][r]; C[p][r] = cs * a - sn * d; C[q][r] = sn * a + cs * d; }
            for (int r = 0; r < 3; ++r) { const double a = V[r][p], d = V[r][q]; V[r][p] = cs * a - sn * d; V[r][q] = sn * a + cs * d; }
        }
        b.centre = c;
        for (int a = 0; a < 3; ++a) { b.axis[a] = {V[0][a], V[1][a], V[2][a]}; b.lo[a] = 1e300; b.hi[a] = -1e300; }
        for (const auto& p : pts) for (int a = 0; a < 3; ++a) {
            const double s = (p[0] - c[0]) * b.axis[a][0] + (p[1] - c[1]) * b.axis[a][1] + (p[2] - c[2]) * b.axis[a][2];
            b.lo[a] = std::min(b.lo[a], s); b.hi[a] = std::max(b.hi[a], s); }
        // + half a cell (the cell's extent from its centre, any orientation: up to sqrt(3)/2) + the margin
        for (int a = 0; a < 3; ++a) { b.lo[a] -= (.8660254 + boxMargin) * pitch; b.hi[a] += (.8660254 + boxMargin) * pitch; }
        b.valid = true;
    }
    stats.boxMs = now() - t0;
    return out;
}

// The box as 8 corners (index bit 0: axis 0 hi, bit 1: axis 1 hi, bit 2: axis 2 hi) and 12 triangles wound outward
// (counter-clockwise seen from outside; the caller flips for its material's cull mode).
inline std::array<std::array<double, 3>, 8> corners(const Box& b) {
    std::array<std::array<double, 3>, 8> c{};
    for (int i = 0; i < 8; ++i) { const double s[3] = {(i & 1) ? b.hi[0] : b.lo[0], (i & 2) ? b.hi[1] : b.lo[1], (i & 4) ? b.hi[2] : b.lo[2]};
        for (int a = 0; a < 3; ++a) c[std::size_t(i)][std::size_t(a)] = b.centre[std::size_t(a)] + s[0] * b.axis[0][std::size_t(a)] + s[1] * b.axis[1][std::size_t(a)] + s[2] * b.axis[2][std::size_t(a)]; }
    return c;
}
inline std::array<std::uint16_t, 36> triangles(const Box& b) {
    std::array<std::uint16_t, 36> t = {0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5, 0, 1, 5, 0, 5, 4, 2, 6, 7, 2, 7, 3, 0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6};
    const auto& a = b.axis; const double det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if (det < 0) for (std::size_t q = 0; q < 36; q += 3) std::swap(t[q + 1], t[q + 2]);
    return t;
}
} // namespace painter_mask_pieces
