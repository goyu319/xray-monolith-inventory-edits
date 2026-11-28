#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <unordered_map>
#include <vector>

struct PainterBurntCrownIndex
{
    std::unordered_map<std::uint64_t, std::vector<unsigned>> tiles;
    std::vector<unsigned> wide;
    unsigned cursor = 0;
    bool ready = false;

    static bool tile(float x, int& out)
    {
        const double value = std::floor(double(x) * .125);
        if (!(value >= -1000000000. && value <= 1000000000.)) return false;
        out = static_cast<int>(value);
        return true;
    }
    static std::uint64_t key(int x, int z)
    {
        return (std::uint64_t(std::uint32_t(x)) << 32) | std::uint32_t(z);
    }
    void reset()
    {
        tiles.clear();
        wide.clear();
        cursor = 0;
        ready = false;
    }
    void add(unsigned i, float xmin, float zmin, float xmax, float zmax)
    {
        int x0, z0, x1, z1;
        if (!tile(xmin,x0) || !tile(zmin,z0) || !tile(xmax,x1) || !tile(zmax,z1) ||
            x1<x0 || z1<z0 || (std::int64_t(x1)-x0+1)*(std::int64_t(z1)-z0+1)>64) {
            wide.push_back(i);
            return;
        }
        for (int z=z0;z<=z1;++z)
            for (int x=x0;x<=x1;++x) tiles[key(x,z)].push_back(i);
    }
    template <class Visit> void visit(float x, float z, unsigned total, const Visit& accept) const
    {
        int tx,tz;
        if (!ready || !tile(x,tx) || !tile(z,tz)) {
            for (unsigned i=0;i<total;++i) accept(i);
            return;
        }
        const auto found=tiles.find(key(tx,tz));
        if (found==tiles.end()) {
            for (unsigned i:wide) accept(i);
            return;
        }
        // Both ranges carry original visual order. A leaf belongs to exactly one range.
        auto big=wide.begin();
        for (unsigned i:found->second) {
            while (big!=wide.end() && *big<i) accept(*big++);
            accept(i);
        }
        while (big!=wide.end()) accept(*big++);
    }
};
