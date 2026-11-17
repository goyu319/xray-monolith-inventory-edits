#pragma once
#include "CombatContract.h"
#include <cstring>
#include <initializer_list>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4995)                  // xrCore marks strcmp deprecated; this header is engine-free
#endif

namespace painter { namespace combat {

namespace faction_parse_detail {
inline bool separator(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == ',' || c == '"'; }
inline bool digit(char c) { return c >= '0' && c <= '9'; }
inline void skip(const char*& at)
{
    while (*at && separator(*at)) ++at;
}
// A word field at `at`: one of `names` (its index), or a lone one-digit code (a longer digit run is ambiguous once the ini
// reader has removed the spaces, e.g. "115", and is refused).
inline bool word(const char*& at, std::initializer_list<const char*> names, std::uint8_t& out)
{
    skip(at);
    std::uint8_t code = 0;
    for (const char* name : names)
    {
        const std::size_t n = std::strlen(name);
        if (!std::strncmp(at, name, n)) { at += n; out = code; return true; }
        ++code;
    }
    if (digit(at[0]) && !digit(at[1]) && at[1] != '.' && unsigned(at[0] - '0') < names.size())
    {
        out = std::uint8_t(at[0] - '0');
        ++at;
        return true;
    }
    return false;
}
// The spread: digits with an optional fraction (no sign, no exponent: a following word may start with 'e').
inline bool number(const char*& at, float& out)
{
    skip(at);
    if (!digit(*at)) return false;
    double value = 0, scale = 0;
    for (; digit(*at); ++at) value = value * 10 + (*at - '0');
    if (*at == '.')
        for (++at, scale = 0.1; digit(*at); ++at, scale /= 10) value += (*at - '0') * scale;
    out = float(value);
    return true;
}
} // namespace faction_parse_detail

// The six fields of one line into `out` (FactionStyle::faction is the caller's: the community's index). False, with `out`
// untouched, for a line that is not exactly the six fields in range; the caller names it and skips it.
inline bool parse_faction_style(const char* text, FactionStyle& out)
{
    using namespace faction_parse_detail;
    if (!text) return false;
    const char* at = text;
    FactionStyle style;
    float spread = 0;
    std::uint8_t leave = 0;
    if (!word(at, {"normal", "chad"}, style.profile) || !word(at, {"loose", "drilled"}, style.doctrine) ||
        !number(at, spread) || !word(at, {"steady", "brittle", "fanatic"}, style.morale) || !word(at, {"0", "1"}, leave) ||
        !word(at, {"voice", "radio"}, style.comms))
        return false;
    skip(at);
    if (*at || !(spread > 0.f && spread < 100.f)) return false;                  // a seventh field, or out of range
    style.leave_wounded = leave == 1;
    style.spread_m = spread;
    style.faction = out.faction;
    out = style;
    return true;
}

}} // namespace painter::combat

#ifdef _MSC_VER
#pragma warning(pop)
#endif
