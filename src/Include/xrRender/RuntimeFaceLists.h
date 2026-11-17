#pragma once
#include <cstdint>
#include <limits>
#include <vector>

// Runtime triangles arrive in face order. Emit each positive-weight bone once
// per face so picking retains the same sorted/unique lists without first
// allocating up to twelve duplicate rows per triangle and sorting them again.
template<class Vertices, class Indices, class Emit>
void BuildRuntimeFaceLists(const Vertices& vertices, const Indices& indices,
    unsigned bone_count, Emit emit)
{
    std::vector<std::uint32_t> seen(bone_count,
        (std::numeric_limits<std::uint32_t>::max)());
    for (std::uint32_t face = 0; face < indices.size() / 3; ++face)
        for (unsigned corner = 0; corner < 3; ++corner)
        {
            const auto& vertex = vertices[indices[face * 3 + corner]];
            for (unsigned link = 0; link < 4; ++link)
                if (vertex.weight[link] != 0.f && seen[vertex.bone[link]] != face)
                {
                    seen[vertex.bone[link]] = face;
                    emit(vertex.bone[link], face);
                }
        }
}
