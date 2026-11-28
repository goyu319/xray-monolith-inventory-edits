// Source edits: small text edits applied to installed shader and script files when the engine reads them, so a mod can
// change a few lines of another mod's file without shipping a copy of the whole file.
//
// List syntax (gamedata/configs/painter_source_edits/*.txt, read in name order):
//   # comment                       outside blocks only
//   @file shaders/r3/ssfx_ssr.ps    gamedata relative; the edits below apply to this file
//   @fallback fallback/<path>       optional, relative to the list folder: served instead when an edit does not match
//   @replace [n]  <lines>  @with   <lines>  @end      replace the anchor lines (n = how often they must occur, default 1)
//   @after [n]    <lines>  @insert <lines>  @end      insert after the anchor lines
//   @before [n]   <lines>  @insert <lines>  @end      insert before the anchor lines
//   @prepend      <lines>  @end                       insert at the top of the file
//   @append       <lines>  @end                       add at the end of the file
// Lines inside a block are literal; a line starting with "@@" stands for the same line with one '@' less.
//
// Matching ignores whitespace (tabs, spaces, line endings). An anchor that covers whole lines replaces or inserts whole
// lines, written with the file's own line endings; blank lines that open or close such an anchor are taken along where
// the file has them. An edit whose result is already in the file counts as present and is
// not applied twice. Every edit of a file must apply or be present, otherwise none is applied.
#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace painter_source_edits
{
enum class EditKind
{
    Replace,
    After,
    Before,
    Prepend,
    Append
};

struct Edit
{
    EditKind kind = EditKind::Replace;
    std::string anchor; // Replace / After / Before: the lines to find, joined with '\n'
    std::string text;   // the new lines, joined with '\n'
    int expected = 1;   // how often the anchor must occur
    int line = 0;       // line of the directive in the list file
};

struct FileEdits
{
    std::string path;     // gamedata relative, lower case, '/' separators
    std::string fallback; // relative to the list folder, lower case, '/' separators; empty = keep the installed file
    std::string origin;   // list file name
    std::vector<Edit> edits;
};

enum class Status
{
    Applied, // at least one edit applied (others may have been present)
    Present, // every edit was already in the file
    Failed   // an edit did not match; nothing applied
};

struct Result
{
    Status status = Status::Present;
    std::string text; // the edited file when Applied
    int applied = 0;
    int present = 0;
    std::string message; // which edit failed and why
};

inline bool isSpace(char c)
{
    return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\v' || c == '\f';
}

inline std::string normalizePath(const std::string& p)
{
    std::string r;
    r.reserve(p.size());
    for (char c : p)
    {
        if (c == '\\')
            c = '/';
        else if (c >= 'A' && c <= 'Z')
            c = char(c - 'A' + 'a');
        if (c == '/' && !r.empty() && r.back() == '/')
            continue;
        r.push_back(c);
    }
    while (!r.empty() && r.front() == '/')
        r.erase(r.begin());
    if (r.compare(0, 2, "./") == 0)
        r.erase(0, 2);
    return r;
}

inline std::string lastComponent(const std::string& p)
{
    const size_t s = p.find_last_of('/');
    return s == std::string::npos ? p : p.substr(s + 1);
}

// 'given' is the path the engine opened (relative or absolute); 'target' a list path.
inline bool pathMatches(const std::string& given, const std::string& target)
{
    if (given == target)
        return true;
    return given.size() > target.size() && given[given.size() - target.size() - 1] == '/' &&
        given.compare(given.size() - target.size(), target.size(), target) == 0;
}

inline std::string trimmed(const std::string& s)
{
    size_t b = 0, e = s.size();
    while (b < e && isSpace(s[b]))
        ++b;
    while (e > b && isSpace(s[e - 1]))
        --e;
    return s.substr(b, e - b);
}

// Lines without their '\n' / "\r\n"; a final line break does not start another line.
inline std::vector<std::string> splitLines(const std::string& text)
{
    std::vector<std::string> lines;
    size_t start = 0;
    for (size_t i = 0; i < text.size(); ++i)
    {
        if (text[i] != '\n')
            continue;
        std::string l = text.substr(start, i - start);
        if (!l.empty() && l.back() == '\r')
            l.pop_back();
        lines.push_back(l);
        start = i + 1;
    }
    if (start < text.size())
    {
        std::string l = text.substr(start);
        if (!l.empty() && l.back() == '\r')
            l.pop_back();
        lines.push_back(l);
    }
    return lines;
}

inline std::string joinLines(const std::vector<std::string>& lines, size_t from, size_t to)
{
    std::string r;
    for (size_t i = from; i < to; ++i)
    {
        if (i > from)
            r.push_back('\n');
        r += lines[i];
    }
    return r;
}

inline const char* kindName(EditKind k)
{
    switch (k)
    {
    case EditKind::Replace: return "replace";
    case EditKind::After: return "after";
    case EditKind::Before: return "before";
    case EditKind::Prepend: return "prepend";
    default: return "append";
    }
}

// Appends the sections of one list file to 'out' (a path may appear in several sections and lists; callers merge them in
// order). Returns false when the list has an error; the sections parsed before the error are kept.
inline bool parse(const std::string& text, const std::string& origin, std::vector<FileEdits>& out,
    std::vector<std::string>& errors)
{
    std::vector<std::string> lines = splitLines(text);
    if (!lines.empty() && lines[0].compare(0, 3, "\xEF\xBB\xBF") == 0)
        lines[0].erase(0, 3);
    long current = -1;
    size_t i = 0;
    auto fail = [&](size_t ln, const std::string& msg) {
        errors.push_back(origin + " line " + std::to_string(ln + 1) + ": " + msg);
        return false;
    };
    auto literal = [](const std::string& raw) {
        const size_t at = raw.find_first_not_of(" \t");
        if (at != std::string::npos && raw.compare(at, 2, "@@") == 0)
            return raw.substr(0, at) + raw.substr(at + 1);
        return raw;
    };
    // collects block lines until one of the two closing directives; returns the index of the closing line
    auto collect = [&](size_t from, const char* close1, const char* close2, std::vector<std::string>& block, size_t& closeAt,
                       std::string& closedBy) {
        for (size_t k = from; k < lines.size(); ++k)
        {
            const std::string t = trimmed(lines[k]);
            if (t == close1 || (close2 && t == close2))
            {
                closeAt = k;
                closedBy = t;
                return true;
            }
            if (t == "@end" || t == "@with" || t == "@insert")
                return false;
            block.push_back(literal(lines[k]));
        }
        return false;
    };
    while (i < lines.size())
    {
        const std::string t = trimmed(lines[i]);
        if (t.empty() || t[0] == '#')
        {
            ++i;
            continue;
        }
        if (t.compare(0, 6, "@file ") == 0)
        {
            FileEdits fe;
            fe.path = normalizePath(trimmed(t.substr(6)));
            fe.origin = origin;
            if (fe.path.empty())
                return fail(i, "@file without a path");
            out.push_back(fe);
            current = long(out.size()) - 1;
            ++i;
            continue;
        }
        if (t.compare(0, 10, "@fallback ") == 0)
        {
            if (current < 0)
                return fail(i, "@fallback before @file");
            out[size_t(current)].fallback = normalizePath(trimmed(t.substr(10)));
            ++i;
            continue;
        }
        std::string word = t, arg;
        const size_t sp = t.find_first_of(" \t");
        if (sp != std::string::npos)
        {
            word = t.substr(0, sp);
            arg = trimmed(t.substr(sp));
        }
        Edit e;
        e.line = int(i + 1);
        if (word == "@replace")
            e.kind = EditKind::Replace;
        else if (word == "@after")
            e.kind = EditKind::After;
        else if (word == "@before")
            e.kind = EditKind::Before;
        else if (word == "@prepend")
            e.kind = EditKind::Prepend;
        else if (word == "@append")
            e.kind = EditKind::Append;
        else
            return fail(i, "unknown line '" + t + "'");
        if (current < 0)
            return fail(i, word + " before @file");
        if (!arg.empty())
        {
            int n = 0;
            for (char c : arg)
            {
                if (c < '0' || c > '9' || n > 100000)
                    return fail(i, "bad count '" + arg + "'");
                n = n * 10 + (c - '0');
            }
            if (n < 1 || e.kind == EditKind::Prepend || e.kind == EditKind::Append)
                return fail(i, "bad count '" + arg + "'");
            e.expected = n;
        }
        std::vector<std::string> a, b;
        size_t closeAt = 0;
        std::string closedBy;
        if (e.kind == EditKind::Prepend || e.kind == EditKind::Append)
        {
            if (!collect(i + 1, "@end", nullptr, b, closeAt, closedBy))
                return fail(i, word + " without @end");
        }
        else
        {
            const char* middle = e.kind == EditKind::Replace ? "@with" : "@insert";
            if (!collect(i + 1, middle, nullptr, a, closeAt, closedBy))
                return fail(i, word + std::string(" without ") + middle);
            if (!collect(closeAt + 1, "@end", nullptr, b, closeAt, closedBy))
                return fail(i, word + " without @end");
            e.anchor = joinLines(a, 0, a.size());
        }
        e.text = joinLines(b, 0, b.size());
        out[size_t(current)].edits.push_back(e);
        i = closeAt + 1;
    }
    return true;
}

struct Compact
{
    std::string s;           // the text without whitespace
    std::vector<size_t> pos; // offset in the original text of every kept character
};

inline Compact compact(const std::string& t)
{
    Compact c;
    c.s.reserve(t.size());
    c.pos.reserve(t.size());
    for (size_t i = 0; i < t.size(); ++i)
        if (!isSpace(t[i]))
        {
            c.s.push_back(t[i]);
            c.pos.push_back(i);
        }
    return c;
}

inline std::string squeeze(const std::string& t)
{
    std::string s;
    s.reserve(t.size());
    for (char c : t)
        if (!isSpace(c))
            s.push_back(c);
    return s;
}

inline std::vector<size_t> findAll(const std::string& hay, const std::string& needle)
{
    std::vector<size_t> r;
    if (needle.empty())
        return r;
    for (size_t p = hay.find(needle); p != std::string::npos; p = hay.find(needle, p + needle.size()))
        r.push_back(p);
    return r;
}

inline std::string newlineOf(const std::string& t)
{
    size_t lf = 0, crlf = 0;
    for (size_t i = 0; i < t.size(); ++i)
        if (t[i] == '\n')
        {
            ++lf;
            if (i > 0 && t[i - 1] == '\r')
                ++crlf;
        }
    return crlf > 0 && crlf * 2 >= lf ? "\r\n" : "\n";
}

inline std::string withNewlines(const std::string& text, const std::string& nl)
{
    if (nl == "\n")
        return text;
    std::string r;
    r.reserve(text.size() + text.size() / 16);
    for (char c : text)
    {
        if (c == '\n')
            r += nl;
        else
            r.push_back(c);
    }
    return r;
}

// the anchor starts a line (only spaces/tabs before it on that line): ls = start of that line
inline bool lineStart(const std::string& t, size_t s, size_t& ls)
{
    size_t i = s;
    while (i > 0 && (t[i - 1] == ' ' || t[i - 1] == '\t'))
        --i;
    if (i == 0 || t[i - 1] == '\n')
    {
        ls = i;
        return true;
    }
    return false;
}

// the anchor ends a line (only spaces/tabs/CR after it): le = just past the line break (or the end of the text)
inline bool lineEnd(const std::string& t, size_t e, size_t& le)
{
    size_t i = e;
    while (i < t.size() && (t[i] == ' ' || t[i] == '\t' || t[i] == '\r'))
        ++i;
    if (i == t.size())
    {
        le = i;
        return true;
    }
    if (t[i] == '\n')
    {
        le = i + 1;
        return true;
    }
    return false;
}

// blank lines at the start and end of an anchor (they do not take part in matching)
inline void blankEdges(const std::string& anchor, int& lead, int& trail)
{
    std::vector<std::string> parts;
    size_t start = 0;
    for (size_t i = 0; i <= anchor.size(); ++i)
        if (i == anchor.size() || anchor[i] == '\n')
        {
            parts.push_back(anchor.substr(start, i - start));
            start = i + 1;
        }
    lead = trail = 0;
    while (size_t(lead) < parts.size() && trimmed(parts[size_t(lead)]).empty())
        ++lead;
    while (size_t(trail) + size_t(lead) < parts.size() && trimmed(parts[parts.size() - 1 - size_t(trail)]).empty())
        ++trail;
}

// moves a line end past up to n blank lines that follow it in the file
inline size_t takeBlankLinesAfter(const std::string& t, size_t le, int n)
{
    while (n-- > 0 && le < t.size())
    {
        size_t i = le;
        while (i < t.size() && (t[i] == ' ' || t[i] == '\t' || t[i] == '\r'))
            ++i;
        if (i < t.size() && t[i] != '\n')
            break;
        le = i < t.size() ? i + 1 : i;
    }
    return le;
}

// moves a line start back over up to n blank lines that precede it in the file
inline size_t takeBlankLinesBefore(const std::string& t, size_t ls, int n)
{
    while (n-- > 0 && ls > 0)
    {
        size_t j = ls - 1; // the line break that ends the line above
        while (j > 0 && (t[j - 1] == ' ' || t[j - 1] == '\t' || t[j - 1] == '\r'))
            --j;
        if (j > 0 && t[j - 1] != '\n')
            break;
        ls = j;
    }
    return ls;
}

inline bool applyOne(std::string& w, const Edit& e, const std::string& nl, bool& present, std::string& why)
{
    present = false;
    const std::string T = squeeze(e.text);
    const std::string body = withNewlines(e.text, nl);
    const Compact c = compact(w);
    if (e.kind == EditKind::Prepend || e.kind == EditKind::Append)
    {
        if (T.empty())
        {
            why = "nothing to insert";
            return false;
        }
        if (c.s.find(T) != std::string::npos)
        {
            present = true;
            return true;
        }
        if (e.kind == EditKind::Prepend)
            w.insert(w.compare(0, 3, "\xEF\xBB\xBF") == 0 ? 3 : 0, body + nl);
        else
        {
            if (!w.empty() && w.back() != '\n')
                w += nl;
            w += body + nl;
        }
        return true;
    }
    const std::string A = squeeze(e.anchor);
    if (A.empty())
    {
        why = "anchor is empty";
        return false;
    }
    const std::vector<size_t> hits = findAll(c.s, A);
    if (e.kind == EditKind::Replace)
    {
        if (!T.empty() && c.s.find(T) != std::string::npos && (hits.empty() || T.find(A) != std::string::npos))
        {
            present = true;
            return true;
        }
    }
    else if (!T.empty())
    {
        const std::string both = e.kind == EditKind::After ? A + T : T + A;
        if (c.s.find(both) != std::string::npos)
        {
            present = true;
            return true;
        }
    }
    if (hits.size() != size_t(e.expected))
    {
        why = "anchor found " + std::to_string(hits.size()) + " times, expected " + std::to_string(e.expected);
        return false;
    }
    int lead = 0, trail = 0;
    blankEdges(e.anchor, lead, trail);
    for (size_t k = hits.size(); k-- > 0;) // last first, so earlier offsets stay valid
    {
        const size_t s = c.pos[hits[k]];
        const size_t end = c.pos[hits[k] + A.size() - 1] + 1;
        size_t ls = 0, le = 0;
        const bool whole = lineStart(w, s, ls) && lineEnd(w, end, le);
        if (whole)
        {
            // blank lines that open or close the anchor belong to it where the file has them
            ls = takeBlankLinesBefore(w, ls, lead);
            le = takeBlankLinesAfter(w, le, trail);
        }
        if (e.kind == EditKind::Replace)
        {
            if (whole)
            {
                const bool brokeLine = le > 0 && w[le - 1] == '\n';
                w.replace(ls, le - ls, T.empty() && e.text.empty() ? std::string() : body + (brokeLine ? nl : std::string()));
            }
            else
                w.replace(s, end - s, body);
        }
        else if (e.kind == EditKind::After)
        {
            if (whole)
            {
                if (le == w.size() && (le == 0 || w[le - 1] != '\n'))
                    w.insert(le, nl + body);
                else
                    w.insert(le, body + nl);
            }
            else
                w.insert(end, body);
        }
        else
        {
            if (whole)
                w.insert(ls, body + nl);
            else
                w.insert(s, body);
        }
    }
    return true;
}

inline Result applyAll(const FileEdits& fe, const std::string& original)
{
    Result r;
    std::string w = original;
    const std::string nl = newlineOf(original);
    for (size_t i = 0; i < fe.edits.size(); ++i)
    {
        bool present = false;
        std::string why;
        if (!applyOne(w, fe.edits[i], nl, present, why))
        {
            r.status = Status::Failed;
            r.message = "edit " + std::to_string(i + 1) + " (" + kindName(fe.edits[i].kind) + ", " + fe.origin + " line " +
                std::to_string(fe.edits[i].line) + "): " + why;
            r.text.clear();
            return r;
        }
        if (present)
            ++r.present;
        else
            ++r.applied;
    }
    r.status = r.applied ? Status::Applied : Status::Present;
    if (r.applied)
        r.text = std::move(w);
    return r;
}

// Sections with the same path are joined in list order (first section's fallback wins unless empty).
inline std::vector<FileEdits> mergeByPath(const std::vector<FileEdits>& sections)
{
    std::vector<FileEdits> merged;
    for (const FileEdits& s : sections)
    {
        FileEdits* into = nullptr;
        for (FileEdits& m : merged)
            if (m.path == s.path)
                into = &m;
        if (!into)
        {
            merged.push_back(s);
            continue;
        }
        if (into->fallback.empty())
            into->fallback = s.fallback;
        into->edits.insert(into->edits.end(), s.edits.begin(), s.edits.end());
        if (into->origin != s.origin)
            into->origin += "+" + s.origin;
    }
    return merged;
}
} // namespace painter_source_edits
