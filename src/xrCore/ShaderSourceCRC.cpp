#include "stdafx.h"
#include "ShaderSourceCRC.h"
#include <map>
#include <string>
#include <vector>

namespace
{
void addShaderSourceCrc32(const void* sourceData, u32 sourceSize, LPCSTR shaderPath, u32& crc);

IReader* openShaderInclude(LPCSTR shaderPath, LPCSTR includeName, string_path& editPath)
{
	string_path includePath;
	strconcat(sizeof(includePath), includePath, shaderPath ? shaderPath : "", includeName);

	IReader* reader = FS.r_open("$game_shaders$", includePath);
	if (reader)
	{
		strconcat(sizeof(editPath), editPath, "shaders\\", includePath);
		return reader;
	}

	strconcat(sizeof(editPath), editPath, "shaders\\", includeName);
	return FS.r_open("$game_shaders$", includeName);
}

bool getIncludeName(LPCSTR sourceLine, string_path& includeName)
{
	string4096 line;
	xr_strcpy(line, sourceLine);
	_Trim(line);

	if (!line[0] || line[0] != '#' || !strstr(line, "#include"))
		return false;

	if (!_GetItem(line, 1, includeName, '"'))
		return false;

	xr_strlwr(includeName);
	return true;
}

void addIncludedShaderCrc32(LPCSTR shaderPath, LPCSTR includeName, u32& crc)
{
    string_path editPath;
    IReader* includeReader = openShaderInclude(shaderPath, includeName, editPath);
    if (!includeReader)
    {
        Msg("! Shader source CRC: can't find include '%s', skipping it for cache validation", includeName);
        return;
    }

    // the CRC covers the include as it is compiled, with its source edits
    const void* source = includeReader->pointer();
    u32 size = includeReader->length();
    painterSourceEdits(editPath, includeReader->pointer(), includeReader->length(), source, size);
    addShaderSourceCrc32(source, size, shaderPath, crc);
    FS.r_close(includeReader);
}

template<class Visit>
void visitIncludes(const char* sourceData, u32 sourceSize, Visit visit)
{
	const char* cursor = sourceData;
	const char* const sourceEnd = sourceData + sourceSize;

	while (cursor < sourceEnd)
	{
		const char* const lineBegin = cursor;
		while (cursor < sourceEnd && *cursor != '\r' && *cursor != '\n')
			++cursor;

		const size_t lineLength = cursor - lineBegin;

		while (cursor < sourceEnd && (*cursor == '\r' || *cursor == '\n'))
			++cursor;

		if (lineLength >= sizeof(string4096))
			continue;

		string4096 line;
		CopyMemory(line, lineBegin, lineLength);
		line[lineLength] = 0;

		string_path includeName;
		if (getIncludeName(line, includeName))
			visit(includeName);
	}
}

void addShaderSourceCrc32(const void* sourceData, u32 sourceSize, LPCSTR shaderPath, u32& crc)
{
	crc = crc32(sourceData, sourceSize, crc);
	visitIncludes(static_cast<const char*>(sourceData), sourceSize,
		[&](const char* name) { addIncludedShaderCrc32(shaderPath,name,crc); });
}

// Preserve the original root and include CRC calls in their original order, including
// repeated include edges. Both CPU CRC implementations use these exact chunk boundaries.
struct IncludeText {
    std::string bytes;
    std::vector<std::string> children;
    bool present = false;
    bool onStack = false;
};

struct ClosureLoad {
    using Key = std::pair<std::string,std::string>;
    std::map<Key,IncludeText> includes;
    std::map<Key,u32> roots;
    unsigned users = 0;
    unsigned lookups = 0;

    void append(const std::string& directory,const std::string& name,u32& crc) {
        ++lookups;
        auto added = includes.try_emplace({directory,name});
        auto& node = added.first->second;
        if (added.second) {
            string_path editPath;
            IReader* file = openShaderInclude(directory.c_str(),name.c_str(),editPath);
            if (!file) {
                Msg("! Shader source CRC: can't find include '%s', skipping it for cache validation",name.c_str());
                return;
            }
            const void* source = file->pointer();
            u32 size = file->length();
            painterSourceEdits(editPath,file->pointer(),size,source,size);
            if (size) node.bytes.assign(static_cast<const char*>(source),size);
            FS.r_close(file);
            visitIncludes(node.bytes.data(),u32(node.bytes.size()),
                [&](const char* child){node.children.emplace_back(child);});
            node.present = true;
        }
        if (!node.present || node.onStack) return;
        node.onStack = true;
        crc = crc32(node.bytes.data(),u32(node.bytes.size()),crc);
        for (const auto& child : node.children) append(directory,child,crc);
        node.onStack = false;
    }

    u32 root(const void* data,u32 size,const char* path) {
        Key key{path ? path : "",std::string(static_cast<const char*>(data),size)};
        auto inserted = roots.try_emplace(std::move(key),0);
        auto& row = *inserted.first;
        if (inserted.second) {
            row.second = crc32(data,size,0);
            visitIncludes(row.first.second.data(),size,
                [&](const char* name){append(row.first.first,name,row.second);});
        }
        return row.second;
    }
};
thread_local ClosureLoad* closureLoad = nullptr;

} // namespace

ShaderClosureLoadScope::ShaderClosureLoadScope() {
    if (!closureLoad) closureLoad = new ClosureLoad;
    ++closureLoad->users;
}
ShaderClosureLoadScope::~ShaderClosureLoadScope() {
    if (!--closureLoad->users) {
        Msg("[painter_shader_load] phase=closure_snapshot roots=%u include_names=%u include_visits=%u",
            unsigned(closureLoad->roots.size()),unsigned(closureLoad->includes.size()),closureLoad->lookups);
        delete closureLoad;
        closureLoad = nullptr;
    }
}


u32 getShaderSourceCrc32(const void* sourceData, u32 sourceSize, LPCSTR shaderPath)
{
	if (closureLoad) return closureLoad->root(sourceData,sourceSize,shaderPath);
	u32 crc = 0;
	addShaderSourceCrc32(sourceData, sourceSize, shaderPath, crc);
	return crc;
}

#include "PainterSourceEdits.inl"
