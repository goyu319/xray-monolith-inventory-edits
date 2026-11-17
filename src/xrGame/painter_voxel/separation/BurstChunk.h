#pragma once
namespace burst_chunk {
inline bool of(const gfy_packed::Mesh* published, const gfy_connectivity::Topology& v, std::uint32_t id) {
    if (!published || !id || id >= v.components.size() || !v.components[id].cells || !(v.pitch > 0)) return false;
    const double cells = double(v.components[id].cells), p = v.pitch;
    for (const auto& tr : published->tears) {
        if (!(tr.block && tr.side == 255) || !head_burst_record::is(*tr.block)) continue; // a fresh burst record (MaskPieces3 fresh())
        head_burst_record::Function f;
        if (!head_burst_record::decode(*tr.block, f) || !(f.Rcut > 0) || !std::isfinite(f.Rcut)) continue;
        const double r2 = f.Rcut * f.Rcut;
        const int x0 = int(std::floor((f.B[0] - f.Rcut - v.origin[0]) / p)), x1 = int(std::floor((f.B[0] + f.Rcut - v.origin[0]) / p));
        const int y0 = int(std::floor((f.B[1] - f.Rcut - v.origin[1]) / p)), y1 = int(std::floor((f.B[1] + f.Rcut - v.origin[1]) / p));
        std::uint64_t inside = 0;
        for (const auto& col : v.columns) {
            if (col.x < x0 || col.x > x1 || col.y < y0 || col.y > y1) continue;
            const double dx = v.origin[0] + (col.x + .5) * p - f.B[0], dy = v.origin[1] + (col.y + .5) * p - f.B[1];
            const double rest = r2 - dx * dx - dy * dy;
            if (rest < 0) continue;
            const double h = std::sqrt(rest);
            // cell z is inside when |origin + (z + .5) p - B.z| <= h
            const double zlo = std::ceil((f.B[2] - h - v.origin[2]) / p - .5), zhi = std::floor((f.B[2] + h - v.origin[2]) / p - .5);
            for (auto s = col.first; s < col.first + col.count && s < v.spans.size(); ++s) {
                const auto& sp = v.spans[s];
                if (sp.component != id) continue;
                const double a = std::max(double(sp.lo), zlo), b = std::min(double(sp.hi) - 1, zhi); // a span holds cells lo .. hi - 1
                if (b >= a) inside += std::uint64_t(b - a + 1);
            }
        }
        if (double(inside) >= .9 * cells) return true;
    }
    return false;
}
} // namespace burst_chunk
