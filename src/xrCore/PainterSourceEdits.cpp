#include "stdafx.h"
#include "PainterSourceEdits.h"
#include "FS_internal.h"

#include <mutex>
#include <string>
#include <vector>

// Painter V13 "source edits" (painter_source_edits\*.txt in $game_config$).
// Painter V13 (2026-11-19) no longer ships copies of other mods' shaders and scripts. It ships small text edits
// instead, and BloodEngine applies them to whatever version of the file is installed as it is loaded.
// This is a reimplementation of that loader from the documented file format:
//   @file <path>                    following blocks apply to that file (path relative to gamedata)
//   @replace <lines> @with <lines> @end
//   @after <lines> @insert <lines> @end
//   @prepend <lines> @end
// Matching ignores spaces, tabs and line endings. When an edit does not match the installed file, the file is
// used unchanged and the log says so ("[painter edits]").
// GOYU: files whose name contains "upscaler" are skipped: they target BloodEngine's own DX11 DLSS/FSR3 shaders,
// which this exe does not use (it has its own DLSS/FG path).

namespace
{
enum class EditKind
{
	Replace,
	After,
	Prepend
};

struct Edit
{
	EditKind kind;
	std::string match; // anchor or text to replace (empty for prepend)
	std::string text;  // replacement / inserted text
};

struct FileEdits
{
	std::string key; // lowercase, '\\' separators, relative to gamedata (e.g. "scripts\\grok_bo.script")
	std::vector<Edit> edits;
	std::string source; // edits file it came from
};

std::mutex g_lock;
volatile bool g_loaded = false;
std::vector<FileEdits> g_files;
thread_local int g_inside = 0;

bool is_ws(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

std::string normalize(const std::string& s)
{
	std::string out;
	out.reserve(s.size());
	for (char c : s)
		if (!is_ws(c))
			out.push_back(c);
	return out;
}

// Normalized view of a buffer, with each normalized char's index in the original.
struct NormView
{
	std::string text;
	std::vector<size_t> pos;

	explicit NormView(const std::string& src)
	{
		text.reserve(src.size());
		pos.reserve(src.size());
		for (size_t i = 0; i < src.size(); ++i)
			if (!is_ws(src[i]))
			{
				text.push_back(src[i]);
				pos.push_back(i);
			}
	}
};

std::string lower_key(std::string s)
{
	for (char& c : s)
	{
		if (c == '/')
			c = '\\';
		else if (c >= 'A' && c <= 'Z')
			c = char(c - 'A' + 'a');
	}
	while (!s.empty() && (s.front() == '\\' || is_ws(s.front())))
		s.erase(s.begin());
	while (!s.empty() && is_ws(s.back()))
		s.pop_back();
	if (s.compare(0, 9, "gamedata\\") == 0)
		s.erase(0, 9);
	return s;
}

std::string trim(const std::string& s)
{
	size_t b = 0, e = s.size();
	while (b < e && is_ws(s[b]))
		++b;
	while (e > b && is_ws(s[e - 1]))
		--e;
	return s.substr(b, e - b);
}

void parse_file(const char* data, size_t size, const char* source_name)
{
	enum
	{
		Top,
		InMatch,
		InText
	} state = Top;
	EditKind kind = EditKind::Replace;
	std::string a, b;
	FileEdits* cur = nullptr;
	int edits = 0;

	size_t i = 0;
	while (i < size)
	{
		size_t e = i;
		while (e < size && data[e] != '\n')
			++e;
		std::string line(data + i, e - i);
		if (!line.empty() && line.back() == '\r')
			line.pop_back();
		i = e + 1;

		const std::string t = trim(line);
		const bool directive = !t.empty() && t[0] == '@';

		if (state == Top)
		{
			if (t.empty() || t[0] == '#')
				continue;
			if (t.compare(0, 6, "@file ") == 0)
			{
				g_files.push_back(FileEdits());
				cur = &g_files.back();
				cur->key = lower_key(t.substr(6));
				cur->source = source_name;
				continue;
			}
			if (!cur)
				continue;
			if (t == "@replace")
			{
				kind = EditKind::Replace;
				state = InMatch;
			}
			else if (t == "@after")
			{
				kind = EditKind::After;
				state = InMatch;
			}
			else if (t == "@prepend")
			{
				kind = EditKind::Prepend;
				state = InText;
			}
			a.clear();
			b.clear();
			continue;
		}

		if (directive && state == InMatch && ((kind == EditKind::Replace && t == "@with") || (kind == EditKind::After && t == "@insert")))
		{
			state = InText;
			continue;
		}
		if (directive && state == InText && t == "@end")
		{
			Edit ed;
			ed.kind = kind;
			ed.match = a;
			ed.text = b;
			cur->edits.push_back(ed);
			++edits;
			state = Top;
			continue;
		}

		std::string& dst = state == InMatch ? a : b;
		if (!dst.empty())
			dst += "\n";
		dst += line;
	}

	Msg("[painter edits] %s: %d edit(s) for %d file(s) so far", source_name, edits, int(g_files.size()));
}

void load_all()
{
	if (!FS.path_exist("$game_config$"))
		return; // too early; try again on a later open

	++g_inside;
	xr_vector<LPSTR>* list = FS.file_list_open("$game_config$", "painter_source_edits\\", FS_ListFiles | FS_RootOnly);
	if (list)
	{
		for (LPSTR name : *list)
		{
			string_path lname;
			xr_strcpy(lname, name);
			xr_strlwr(lname);
			const size_t n = xr_strlen(lname);
			if (n < 4 || 0 != xr_strcmp(lname + n - 4, ".txt"))
				continue;
			if (strstr(lname, "upscaler"))
			{
				Msg("[painter edits] %s skipped (BloodEngine upscaler edits, not used by this exe)", name);
				continue;
			}
			string_path rel;
			strconcat(sizeof(rel), rel, "painter_source_edits\\", name);
			IReader* R = FS.r_open("$game_config$", rel);
			if (!R)
				continue;
			parse_file(static_cast<const char*>(R->pointer()), R->length(), name);
			FS.r_close(R);
		}
		FS.file_list_close(list);
	}
	--g_inside;
	g_loaded = true;
}

bool apply_edits(std::string& src, const FileEdits& fe, const char* fname)
{
	const std::string nl = src.find("\r\n") != std::string::npos ? "\r\n" : "\n";
	int applied = 0;

	for (size_t k = 0; k < fe.edits.size(); ++k)
	{
		const Edit& ed = fe.edits[k];
		std::string text = ed.text;
		if (nl == "\r\n")
		{
			std::string t;
			for (char c : text)
			{
				if (c == '\n')
					t += "\r\n";
				else
					t.push_back(c);
			}
			text.swap(t);
		}

		NormView view(src);
		const std::string ntext = normalize(ed.text);

		if (ed.kind == EditKind::Prepend)
		{
			if (!ntext.empty() && view.text.compare(0, ntext.size(), ntext) == 0)
				continue; // already there
			src.insert(0, text + nl);
			++applied;
			continue;
		}

		const std::string nmatch = normalize(ed.match);
		if (nmatch.empty())
			continue;
		const size_t at = view.text.find(nmatch);

		if (ed.kind == EditKind::Replace)
		{
			if (at == std::string::npos)
			{
				if (!ntext.empty() && view.text.find(ntext) != std::string::npos)
					continue; // this version already has the change
				Msg("! [painter edits] %s: edit %u (replace) does not match, file used as it is", fname, u32(k + 1));
				return false;
			}
			const size_t start = view.pos[at];
			const size_t end = view.pos[at + nmatch.size() - 1] + 1;
			// keep the indentation of the first matched line: replacement text carries its own
			size_t line_start = start;
			while (line_start > 0 && src[line_start - 1] != '\n')
				--line_start;
			bool only_ws = true;
			for (size_t p = line_start; p < start; ++p)
				if (!is_ws(src[p]))
					only_ws = false;
			const size_t from = only_ws ? line_start : start;
			src.replace(from, end - from, text);
			++applied;
		}
		else // After
		{
			if (at == std::string::npos)
			{
				Msg("! [painter edits] %s: edit %u (after) does not match, file used as it is", fname, u32(k + 1));
				return false;
			}
			if (!ntext.empty() && view.text.compare(at + nmatch.size(), ntext.size(), ntext) == 0)
				continue; // already inserted right after the anchor (or this version has it)
			size_t end = view.pos[at + nmatch.size() - 1] + 1;
			while (end < src.size() && src[end] != '\n')
				++end;
			if (end < src.size())
			{
				src.insert(end + 1, text + nl);
			}
			else
			{
				src += nl + text + nl;
			}
			++applied;
		}
	}

	Msg("[painter edits] %s: %d edit(s) applied (%s)", fname, applied, fe.source.c_str());
	return true;
}
} // namespace

IReader* painter_source_edits_apply(IReader* R, LPCSTR fname)
{
	if (!R || !fname || g_inside)
		return R;

	// Only shader and script sources are edited.
	if (!strstr(fname, "\\shaders\\") && !strstr(fname, "\\scripts\\"))
		return R;

	if (!g_loaded)
	{
		std::lock_guard<std::mutex> guard(g_lock);
		if (!g_loaded)
			load_all();
		if (!g_loaded)
			return R;
	}
	if (g_files.empty())
		return R;

	const size_t flen = xr_strlen(fname);
	std::vector<const FileEdits*> hits;
	for (const FileEdits& fe : g_files)
	{
		const size_t klen = fe.key.size();
		if (klen + 1 > flen)
			continue;
		const char* tail = fname + flen - klen;
		if (tail[-1] != '\\')
			continue;
		if (0 == _stricmp(tail, fe.key.c_str()))
			hits.push_back(&fe);
	}
	if (hits.empty())
		return R;

	std::string src(static_cast<const char*>(R->pointer()), R->length());
	const std::string original = src;
	bool changed = false;
	for (const FileEdits* fe : hits)
	{
		std::string attempt = src;
		if (apply_edits(attempt, *fe, fname))
		{
			changed = changed || attempt != src;
			src.swap(attempt);
		}
	}
	if (!changed || src == original)
		return R;

	void* data = xr_malloc(src.size());
	CopyMemory(data, src.data(), src.size());
	IReader* edited = xr_new<CTempReader>(data, int(src.size()), 0);
	FS.r_close(R);
	return edited;
}
