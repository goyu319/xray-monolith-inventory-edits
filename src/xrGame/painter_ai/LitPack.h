#pragma once
// painter_ai/LitPack.h - the building data's packed red-light tables (data v9, contract v2.4 Portal::lit_a / lit_b).
// A portal side's table is one string: base64 of, per lit node in increasing node order, the LEB128 node delta from the
// previous node (the first from 0), the standing byte and the crouched byte (0..255). The offline generator writes it
// (gen_lit.py); load_buildings decodes it here. No engine include, so it can be checked on its own.
#include <cstddef>
#include <cstdint>
#include <vector>
#include "CombatContract.h"

namespace painter { namespace combat {

// Decodes `length` characters of `packed` into `out` (cleared first). False, with `out` cleared, on anything malformed:
// a character outside base64, a cut-off entry, a node that does not increase, or a node id >= node_count.
inline bool unpack_lit(const char* packed, std::size_t length, std::uint32_t node_count, std::vector<Lit>& out)
{
    out.clear();
    if (!packed) return true;
    std::vector<std::uint8_t> bytes;
    bytes.reserve(length / 4 * 3 + 3);
    std::uint32_t bits = 0;
    int held = 0;
    std::size_t i = 0;
    for (; i < length && packed[i] != '='; ++i)
    {
        const char c = packed[i];
        int v;
        if (c >= 'A' && c <= 'Z') v = c - 'A';
        else if (c >= 'a' && c <= 'z') v = c - 'a' + 26;
        else if (c >= '0' && c <= '9') v = c - '0' + 52;
        else if (c == '+') v = 62;
        else if (c == '/') v = 63;
        else { out.clear(); return false; }
        bits = ((bits << 6) | std::uint32_t(v)) & 0xffffffu;
        held += 6;
        if (held >= 8)
        {
            held -= 8;
            bytes.push_back(std::uint8_t(bits >> held));
        }
    }
    for (; i < length; ++i)
        if (packed[i] != '=') { out.clear(); return false; }
    out.reserve(bytes.size() / 3);
    std::uint64_t node = 0;
    for (std::size_t at = 0; at < bytes.size();)
    {
        std::uint64_t delta = 0;
        for (int shift = 0;; shift += 7)
        {
            if (at >= bytes.size() || shift > 28) { out.clear(); return false; }
            const std::uint8_t b = bytes[at++];
            delta |= std::uint64_t(b & 0x7f) << shift;
            if (!(b & 0x80)) break;
        }
        if ((!out.empty() && !delta) || at + 2 > bytes.size()) { out.clear(); return false; }
        node += delta;
        if (node >= node_count) { out.clear(); return false; }
        Lit lit;
        lit.node = std::uint32_t(node);
        lit.standing = bytes[at];
        lit.crouched = bytes[at + 1];
        out.push_back(lit);
        at += 2;
    }
    return true;
}

}} // namespace painter::combat
