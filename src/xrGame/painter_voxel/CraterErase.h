#pragma once

// Classify each sorted/unique native cell against the same read-only input.
// Commit per-tile words serially through the existing journal. The full list
// of fresh cells stays in original lexicographic order for the native solver.
template<class Counters>
inline void eraseCraterParallel(Grid& g,const std::vector<Cell>& cells,std::vector<Cell>& out,std::set<Cell>& touched,
                               std::size_t& solid,Counters& bySlot){
    struct Part{
        std::vector<Cell> fresh;
        std::unordered_map<Cell,core::Tile,core::CellHash> bits;
        std::vector<std::pair<Cell,core::Tile*>> firstVisit;
        Counters counts{};
    };
    std::vector<Part> parts(core::parallelWidth());
    const std::size_t grain=std::max<std::size_t>(768,cells.size()/parts.size());
    core::parallelChunks(cells.size(),grain,[&](std::size_t first,std::size_t last,std::size_t worker){
        auto& p=parts[worker];p.fresh.reserve(last-first);
        CraterMaterialRow material;
        Cell lastTile{-1,-1,-1};const core::Tile* original=nullptr;core::Tile* write=nullptr;
        for(std::size_t i=first;i<last;++i){
            const Cell c=cells[i];if(!g.valid(c)||!g.ownerCuttable(c))continue;
            if(i&&c==cells[i-1])continue;
            const Cell tile=Grid::tileKey(c);
            if(!(lastTile==tile)){
                lastTile=tile;original=g.findTile(tile);
                auto item=p.bits.try_emplace(tile);write=&item.first->second;
                if(item.second)p.firstVisit.push_back({tile,write});
            }
            const unsigned bit=Grid::local(c);const auto word=std::uint64_t(1)<<(bit&63);
            if(original&&(original->removed[bit>>6]&word))continue;
            write->removed[bit>>6]|=word;p.fresh.push_back(c);
            if(const auto label=material.get(g,c)){
                const auto slot=g.material_slots&&label<g.material_slots->size()?unsigned((*g.material_slots)[label]&63u):1u;
                ++p.counts[slot];
            }
        }
    });
    out.reserve(cells.size());
    for(auto& p:parts){
        for(const auto& entry:p.firstVisit){
            auto& tile=core::journaledTile(g,entry.first);
            bool any=false;for(int j=0;j<8;++j){const auto bits=entry.second->removed[j];tile.removed[j]|=bits;any|=bits!=0;}
            if(any)touched.insert(entry.first);
        }
        for(unsigned i=0;i<p.counts.size();++i){bySlot[i]+=p.counts[i];solid+=p.counts[i];}
        out.insert(out.end(),p.fresh.begin(),p.fresh.end());
    }
}
