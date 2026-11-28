#pragma once
// Owner-thread shader resources. Cache only reflected layouts. Every publication
// supplies its current textures, including the actor's live blood table.
namespace PainterPassLayouts {
enum class Input : unsigned char {
    Pages, Bits, Planes, Regions, AnatomyPages, AnatomyData, TearParams,
    TearMap, TearBake, Dents, Slits, Blood, Original, Tissue
};
struct Binding {
    u16 destination;
    Input source;
    u16 sourceSlot;
};
struct Key {
    ref_ps ps;
    ref_vs vs;
    ref_ctable original, tissue;
    R_constant_setup *clock, *target;
    unsigned flags;
    bool same(const Key& k) const {
        return ps._get()==k.ps._get() && vs._get()==k.vs._get() &&
            original._get()==k.original._get() && tissue._get()==k.tissue._get() &&
            clock==k.clock && target==k.target && flags==k.flags;
    }
};
struct Layout {
    Key key;
    ref_ctable constants;
    std::vector<Binding> bindings;

    bool bind(const SPass& source, const SPass* tissue, const ref_texture* const* fields,
              STextureList& textures, xr_string& why) const {
        textures.reserve(bindings.size());
        for (const Binding& b:bindings) {
            if (b.source==Input::Original || b.source==Input::Tissue) {
                const SPass* from=b.source==Input::Original ? &source : tissue;
                if (from && from->T) {
                    const auto& list=*from->T;
                    const auto it=std::find_if(list.begin(),list.end(),[&](const auto& t){return t.first==b.sourceSlot;});
                    if (it!=list.end()) { textures.emplace_back(b.destination,it->second); continue; }
                }
                why="missing_original_texture";
                return false;
            }
            const auto& texture=*fields[static_cast<unsigned>(b.source)];
            if (!texture) { why="mask_resource_type"; return false; }
            textures.emplace_back(b.destination,texture);
        }
        return true;
    }
};
struct Cache {
    std::vector<Layout> entries;
    size_t next=0;
    const Layout* find(const Key& key) const {
        for (const auto& item:entries) if (item.key.same(key)) return &item;
        return nullptr;
    }
    void remember(Layout&& item) {
        constexpr size_t limit=192;
        if (entries.size()<limit) entries.push_back(std::move(item));
        else { entries[next]=std::move(item); next=(next+1)%limit; }
    }
    void clear() { entries.clear(); next=0; }
};
inline Cache& cache() { static Cache value; return value; }
}
