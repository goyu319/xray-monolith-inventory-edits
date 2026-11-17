#pragma once
// Included inside separation_detail. Semantic ownership only: connectivity
// labels and the split verdict continue to use the unmodified raw topology.
using IslandGroups=std::map<std::uint32_t,std::uint32_t>;
inline bool unchangedPristineIsland(const Grid&grid,const gfy_connectivity::Topology&raw,
    const gfy_connectivity::Topology&pristine,std::uint32_t id){
 if(!id||id>=raw.components.size()||raw.size!=pristine.size||raw.origin!=pristine.origin||raw.pitch!=pristine.pitch)return false;
 using SpanKey=std::array<int,4>;std::vector<SpanKey>live,original;std::uint32_t ancestor=0;std::uint64_t cells=0;
 for(const auto&col:raw.columns)for(auto n=col.first;n<col.first+col.count;++n){const auto&s=raw.spans[n];if(s.component!=id)continue;
  const auto*source=pristine.column(col.x,col.y);if(!source)return false;
  const auto first=pristine.spans.begin()+source->first,last=first+source->count;
  auto p=std::upper_bound(first,last,s.lo,[](int z,const gfy_connectivity::Span&r){return z<r.lo;});if(p==first)return false;--p;
  if(s.lo<p->lo||s.hi>p->hi||!p->component||(ancestor&&ancestor!=p->component))return false;
  ancestor=p->component;cells+=std::uint64_t(s.hi-s.lo);live.push_back({col.x,col.y,s.lo,s.hi});}
 if(!ancestor||cells!=raw.components[id].cells||cells!=pristine.components[ancestor].cells)return false;
 for(const auto&col:pristine.columns)for(auto n=col.first;n<col.first+col.count;++n){const auto&s=pristine.spans[n];if(s.component==ancestor)original.push_back({col.x,col.y,s.lo,s.hi});}
 if(live!=original)return false; // Full spans, not just count or bounding box.
 for(const auto&s:live)for(int z=s[2];z<s[3];++z){const Cell c{s[0],s[1],z};
  for(int a=0;a<3;++a)for(int sign:{-1,1}){Cell q=c;(&q.x)[a]+=sign;
   if(grid.valid(q)&&grid.removed(q))return false;}}
 return true;
}
template<class Elements>
inline bool chooseIslandGroups(const Grid&grid,const gfy_connectivity::Topology&raw,
    const gfy_connectivity::Topology&pristine,const std::vector<std::uint32_t>&ids,
    const std::vector<std::uint32_t>&empty,const Elements&elements,IslandGroups&groups){
 IslandGroups chosen;const std::set<std::uint32_t> missing(empty.begin(),empty.end());
 if(empty.empty()||missing.size()!=empty.size())return false;
 for(const auto id:empty){const auto e=elements.find(id);
  if(std::find(ids.begin(),ids.end(),id)==ids.end()||e==elements.end()||e->second.size()!=1||!unchangedPristineIsland(grid,raw,pristine,id))return false;
  const auto bone=*e->second.begin();std::uint32_t recipient=0;
  for(const auto other:ids)if(other!=id&&!missing.count(other)){const auto candidate=elements.find(other);
   if(candidate!=elements.end()&&candidate->second.count(bone)){if(recipient)return false;recipient=other;}}
  if(!recipient)return false;chosen.emplace(id,recipient);
 }
 if(chosen.size()!=empty.size())return false;groups=std::move(chosen);return true;
}
inline gfy_connectivity::Topology semanticIslandVolume(const gfy_connectivity::Topology&raw,const IslandGroups&groups){
 auto out=raw;std::uint64_t carried=0;
 for(const auto&g:groups){need(g.first&&g.second&&g.first<raw.components.size()&&g.second<raw.components.size()&&g.first!=g.second&&!groups.count(g.second),"island_group_domain");
  const auto&from=raw.components[g.first];auto&to=out.components[g.second];need(from.cells&&to.cells,"island_group_empty");
  to.cells+=from.cells;carried+=from.cells;for(unsigned a=0;a<3;++a){to.lo[a]=std::min(to.lo[a],from.lo[a]);to.hi[a]=std::max(to.hi[a],from.hi[a]);}
  out.components[g.first]={};}
 std::uint64_t relabeled=0;for(auto&s:out.spans){const auto g=groups.find(s.component);if(g!=groups.end()){relabeled+=std::uint64_t(s.hi-s.lo);s.component=g->second;}}
 need(relabeled==carried&&out.cells==raw.cells,"island_group_cell_conservation");
 return out;
}
