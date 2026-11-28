// Source edits, engine side (included at the end of ShaderSourceCRC.cpp). The lists are read once from
// gamedata/configs/painter_source_edits/*.txt; every file the engine reads through painterSourceEdits() is edited once per
// session and the result is kept for the rest of it. One log line per edited file, so the log shows what applied.
#include <algorithm>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include "PainterSourceEditsCore.h"

namespace
{
namespace pse = painter_source_edits;

struct SourceEditCacheEntry
{
    bool replaced = false; // false: the engine keeps the bytes it read
    std::string text;
};

std::mutex g_sourceEditsLock;
bool g_sourceEditsLoaded = false;
std::vector<pse::FileEdits> g_sourceEdits;
std::map<std::string, std::vector<size_t>> g_sourceEditsByName; // file name -> sections in g_sourceEdits
std::map<std::pair<std::string, u32>, SourceEditCacheEntry> g_sourceEditCache;

const char* const kSourceEditFolder = "painter_source_edits\\";

void loadSourceEditsLocked()
{
    g_sourceEditsLoaded = true;
    xr_vector<LPSTR>* list = FS.file_list_open("$game_config$", kSourceEditFolder, FS_ListFiles | FS_RootOnly);
    if (!list)
        return;
    std::vector<std::string> names;
    for (LPSTR n : *list)
    {
        std::string name = pse::normalizePath(n);
        // GOYU: lists named "*upscaler*" edit BloodEngine's own DX11 DLSS/FSR3 shaders. This exe has its own
        // DLSS / frame generation path and does not use those shaders, so those lists are not read.
        if (name.find("upscaler") != std::string::npos)
        {
            Msg("* [painter edits] %s skipped (BloodEngine's own upscaler; this exe uses its own)", name.c_str());
            continue;
        }
        if (name.size() > 4 && name.compare(name.size() - 4, 4, ".txt") == 0 && name.find('/') == std::string::npos)
            names.push_back(name);
    }
    FS.file_list_close(list);
    std::sort(names.begin(), names.end());

    std::vector<pse::FileEdits> sections;
    for (const std::string& name : names)
    {
        string_path rel;
        strconcat(sizeof(rel), rel, kSourceEditFolder, name.c_str());
        IReader* reader = FS.r_open("$game_config$", rel);
        if (!reader)
            continue;
        const std::string text(static_cast<const char*>(reader->pointer()), size_t(reader->length()));
        FS.r_close(reader);
        std::vector<std::string> errors;
        if (!pse::parse(text, name, sections, errors))
            Msg("! [painter edits] list %s has an error, its later edits are not used", name.c_str());
        for (const std::string& e : errors)
            Msg("! [painter edits] %s", e.c_str());
    }
    g_sourceEdits = pse::mergeByPath(sections);
    u32 edits = 0;
    for (size_t i = 0; i < g_sourceEdits.size(); ++i)
    {
        g_sourceEditsByName[pse::lastComponent(g_sourceEdits[i].path)].push_back(i);
        edits += u32(g_sourceEdits[i].edits.size());
    }
    Msg("* [painter edits] %u edits for %u files from %u lists", edits, u32(g_sourceEdits.size()), u32(names.size()));
}

bool readSourceEditFallback(const std::string& fallback, std::string& out)
{
    string_path rel;
    xr_strcpy(rel, kSourceEditFolder);
    xr_strcat(rel, fallback.c_str());
    for (char* c = rel; *c; ++c)
        if (*c == '/')
            *c = '\\';
    IReader* reader = FS.r_open("$game_config$", rel);
    if (!reader)
        return false;
    out.assign(static_cast<const char*>(reader->pointer()), size_t(reader->length()));
    FS.r_close(reader);
    return true;
}
} // namespace

bool painterSourceEdits(LPCSTR path, const void* data, u32 size, const void*& editedData, u32& editedSize)
{
    if (!path || !data)
        return false;
    const std::string given = pse::normalizePath(path);

    std::lock_guard<std::mutex> guard(g_sourceEditsLock);
    if (!g_sourceEditsLoaded)
        loadSourceEditsLocked();

    const auto byName = g_sourceEditsByName.find(pse::lastComponent(given));
    if (byName == g_sourceEditsByName.end())
        return false;
    const pse::FileEdits* fe = nullptr;
    for (size_t index : byName->second)
        if (pse::pathMatches(given, g_sourceEdits[index].path))
            fe = &g_sourceEdits[index];
    if (!fe)
        return false;

    const auto key = std::make_pair(fe->path, crc32(data, size));
    auto cached = g_sourceEditCache.find(key);
    if (cached == g_sourceEditCache.end())
    {
        SourceEditCacheEntry entry;
        const pse::Result r = pse::applyAll(*fe, std::string(static_cast<const char*>(data), size_t(size)));
        if (r.status == pse::Status::Applied)
        {
            entry.replaced = true;
            entry.text = r.text;
            Msg("* [painter edits] %s: %d edits applied%s", fe->path.c_str(), r.applied,
                r.present ? " (the others were already in the file)" : "");
        }
        else if (r.status == pse::Status::Present)
            Msg("* [painter edits] %s: already edited (%d edits found in the file)", fe->path.c_str(), r.present);
        else if (!fe->fallback.empty() && readSourceEditFallback(fe->fallback, entry.text))
        {
            entry.replaced = true;
            Msg("! [painter edits] %s: %s. Using the fallback copy %s", fe->path.c_str(), r.message.c_str(), fe->fallback.c_str());
        }
        else
            Msg("! [painter edits] %s: %s. Using the installed file unchanged%s", fe->path.c_str(), r.message.c_str(),
                fe->fallback.empty() ? "" : " (fallback copy missing)");
        cached = g_sourceEditCache.emplace(key, std::move(entry)).first;
    }
    if (!cached->second.replaced)
        return false;
    editedData = cached->second.text.data();
    editedSize = u32(cached->second.text.size());
    return true;
}
