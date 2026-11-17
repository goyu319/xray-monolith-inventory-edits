#pragma once
// painter_ai/NodePath.h - the path length on the level graph for the native combat AI (the proposed v2.13
// NodePathLengthFn): A* over the nav nodes' links, centre-to-centre distances as the cost and the straight line to the
// goal as the estimate, bounded by max_nodes expansions. It reads only the graph (vertex links and centres) and the
// caller's scratch, so any thread may run it - unlike the engine's CGraphEngine, whose one search state every stalker's
// path building shares. No restrictions (the publish task re-checks the goal). No engine include: the graph comes in as an
// accessor, so the same code is tested on its own against the map files.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace painter { namespace combat {

struct PathScratch {                             // one per thread (thread_local at the caller); reused between searches
    std::vector<std::uint32_t> node, stamp;      // open addressing: a slot's node, and the search that wrote it
    std::vector<float> cost;                     // best known distance from the start
    std::vector<std::uint8_t> closed;
    std::vector<std::pair<float, std::uint32_t>> heap;   // (distance so far + estimate, node), a min-heap
    std::uint32_t generation = 0, mask = 0;
};

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4995)                  // xrCore marks sqrt deprecated (vector.h); the inner loop needs the plain one
#endif

constexpr std::uint32_t path_max_nodes = 65536; // the largest bound a caller gets (scratch ~13 MB per thread there)

// Metres along node centres from `from` to `to`, or -1 when either node is invalid or the goal is not reached within
// max_nodes expansions. Graph: bool valid(u32) const; void centre(u32, float (&)[3]) const; u32 link(u32 node, int k)
// const for k = 0..3 (an invalid id when there is no link that way).
template <class Graph>
float node_path_length(const Graph& graph, std::uint32_t from, std::uint32_t to, std::uint32_t max_nodes, PathScratch& s)
{
    if (!graph.valid(from) || !graph.valid(to)) return -1.f;
    if (from == to) return 0.f;
    max_nodes = std::min(std::max<std::uint32_t>(max_nodes, 1), path_max_nodes);
    // at most 1 + 4 per expansion distinct nodes enter the table: keep it at most half full
    const std::size_t want = std::size_t(max_nodes) * 8 + 8;
    std::size_t size = 1024;
    while (size < want) size <<= 1;
    if (s.node.size() < size)
    {
        s.node.assign(size, 0);
        s.stamp.assign(size, 0);
        s.cost.assign(size, 0.f);
        s.closed.assign(size, 0);
        s.generation = 0;
    }
    s.mask = std::uint32_t(s.node.size() - 1);
    if (++s.generation == 0)
    {
        std::fill(s.stamp.begin(), s.stamp.end(), 0u);
        s.generation = 1;
    }
    const auto slot = [&](std::uint32_t id) {
        std::uint32_t h = (id * 2654435761u) & s.mask;
        while (s.stamp[h] == s.generation && s.node[h] != id) h = (h + 1) & s.mask;
        if (s.stamp[h] != s.generation)
        {
            s.stamp[h] = s.generation;
            s.node[h] = id;
            s.cost[h] = std::numeric_limits<float>::max();
            s.closed[h] = 0;
        }
        return h;
    };
    float goal[3];
    graph.centre(to, goal);
    const auto distance = [](const float (&a)[3], const float (&b)[3]) {
        const float dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };
    const auto later = [](const std::pair<float, std::uint32_t>& a, const std::pair<float, std::uint32_t>& b) {
        return a.first > b.first;
    };
    s.heap.clear();
    float here[3];
    graph.centre(from, here);
    s.cost[slot(from)] = 0.f;
    s.heap.push_back({distance(here, goal), from});
    std::uint32_t expanded = 0;
    while (!s.heap.empty())
    {
        std::pop_heap(s.heap.begin(), s.heap.end(), later);
        const std::uint32_t id = s.heap.back().second;
        s.heap.pop_back();
        const std::uint32_t at = slot(id);
        if (s.closed[at]) continue;               // a stale entry: this node was settled at a lower cost
        s.closed[at] = 1;
        if (id == to) return s.cost[at];        // the estimate is consistent: the first time the goal settles is the shortest
        if (++expanded > max_nodes) return -1.f;
        graph.centre(id, here);
        const float base = s.cost[at];
        for (int k = 0; k < 4; ++k)
        {
            const std::uint32_t next = graph.link(id, k);
            if (!graph.valid(next)) continue;
            const std::uint32_t ns = slot(next);
            if (s.closed[ns]) continue;
            float there[3];
            graph.centre(next, there);
            const float g = base + distance(here, there);
            if (g < s.cost[ns])
            {
                s.cost[ns] = g;
                s.heap.push_back({g + distance(there, goal), next});
                std::push_heap(s.heap.begin(), s.heap.end(), later);
            }
        }
    }
    return -1.f;
}

#ifdef _MSC_VER
#pragma warning(pop)
#endif

}} // namespace painter::combat
