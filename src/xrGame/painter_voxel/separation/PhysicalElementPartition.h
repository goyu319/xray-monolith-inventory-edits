#pragma once
// Exact intersection of an immutable, source-hashed pre-damage physical-
// element map with a selected surviving anatomy component. Equal pitch with
// integer-cell origin translation only; resampling needs a verified adapter.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <map>
#include <stdexcept>
#include <vector>
#include "PieceCollider.h"
#include "ThreadBudget.h"
namespace gfy_piece_collider { namespace element_partition {
using gfy_connectivity::Topology;
inline void require(bool ok,const char*why){if(!ok)throw std::runtime_error(why);}
#pragma pack(push,1)
struct Run {std::uint32_t column;std::uint16_t lo,hi,element;};
#pragma pack(pop)
static_assert(sizeof(Run)==10,"physics_map_record_size");
struct Map {
 std::array<std::uint8_t,32> source_sha256{},volume_sha256{};
 std::array<std::uint32_t,3> size{};
 std::array<double,3> origin{};
 double pitch{};
 std::vector<Run> runs;
 std::vector<std::uint32_t> offsets;
 std::map<std::uint16_t,std::uint64_t> original_cells;
};
inline Map decode(const std::uint8_t*data,std::size_t length,
                  const std::array<std::uint8_t,32>&expected_source,
                  const std::array<std::uint8_t,32>&expected_volume){
 require(data&&length>=124,"physics_map_short");std::size_t cursor=0;
 auto read=[&](auto&v){require(sizeof(v)<=length-cursor,"physics_map_truncated");
                      std::memcpy(&v,data+cursor,sizeof(v));cursor+=sizeof(v);};
 std::array<char,8>magic{};read(magic);require(std::memcmp(magic.data(),"PHYSCOL1",8)==0,"physics_map_magic");
 Map out;read(out.source_sha256);read(out.volume_sha256);
 require(out.source_sha256==expected_source&&out.volume_sha256==expected_volume,"physics_map_source_identity");
 read(out.size);read(out.origin);read(out.pitch);
 for(int a=0;a<3;++a)require(out.size[a]>0&&out.size[a]<=65535&&std::isfinite(out.origin[a]),"physics_map_domain");
 require(std::isfinite(out.pitch)&&out.pitch>=.0005&&out.pitch<=.01,"physics_map_pitch");
 std::uint64_t count;read(count);
 const auto columns=std::uint64_t(out.size[0])*out.size[1];
 require(columns<=30000000&&count<=64000000&&
         length==124+count*sizeof(Run),"physics_map_size");
 out.runs.resize(std::size_t(count));
 std::memcpy(out.runs.data(),data+cursor,std::size_t(count*sizeof(Run)));
 out.offsets.resize(std::size_t(columns)+1);
 std::uint32_t previous=0;std::uint16_t previousHi=0;std::size_t filled=0;
 for(std::size_t i=0;i<out.runs.size();++i){const auto&r=out.runs[i];
  require(r.column<columns&&r.element&&r.lo<r.hi&&r.hi<=out.size[2],"physics_map_run");
  require(!i||r.column>=previous,"physics_map_column_order");
  if(!i||r.column!=previous){while(filled<=r.column)out.offsets[filled++]=std::uint32_t(i);previousHi=0;}
  require(r.lo>=previousHi,"physics_map_run_overlap");previousHi=r.hi;previous=r.column;}
 for(const auto&r:out.runs)out.original_cells[r.element]+=std::uint64_t(r.hi-r.lo);
 while(filled<out.offsets.size())out.offsets[filled++]=std::uint32_t(out.runs.size());
 return out;
}
inline Map decode(const std::vector<std::uint8_t>&bytes,
                  const std::array<std::uint8_t,32>&source,
                  const std::array<std::uint8_t,32>&volume){
 return decode(bytes.data(),bytes.size(),source,volume);
}
inline Map load(const char*path,const std::array<std::uint8_t,32>&source,
                const std::array<std::uint8_t,32>&volume){
 std::ifstream f(path,std::ios::binary|std::ios::ate);require(bool(f),"physics_map_open");
 const auto length=f.tellg();require(length>=0&&length<=std::streamoff(640000000),"physics_map_file_budget");
 f.seekg(0);std::vector<std::uint8_t> bytes(std::size_t(length),std::uint8_t{});
 f.read(reinterpret_cast<char*>(bytes.data()),std::streamsize(bytes.size()));
 require(bool(f),"physics_map_read");return decode(bytes,source,volume);
}
struct Element {std::uint16_t bone{};Topology volume;bool original_shape_complete{};};
// Include even pre-existing one-cell atlas islands in this accounting. Those
// islands are not a fresh bullet cut and must not force a refit of both calves
// when a completely unrelated forearm is severed.
inline std::map<std::uint16_t,std::uint64_t> remainingCounts(const Map&map,const Topology&top){
 require(std::abs(map.pitch-top.pitch)<=1e-12,"physics_count_pitch");
 std::array<int,3>shift{};
 for(unsigned a=0;a<3;++a){const auto d=(top.origin[a]-map.origin[a])/map.pitch;
  require(std::isfinite(d)&&std::abs(d)<1000000&&std::abs(d-std::nearbyint(d))<1e-6,"physics_count_lattice");shift[a]=int(std::nearbyint(d));}
 std::map<std::uint16_t,std::uint64_t>out;std::uint64_t total=0;
 for(const auto&column:top.columns){const int x=column.x+shift[0],y=column.y+shift[1];
  require(x>=0&&y>=0&&x<int(map.size[0])&&y<int(map.size[1]),"physics_count_outside");
  const auto key=std::size_t(y)*map.size[0]+x;auto r=map.offsets[key];const auto end=map.offsets[key+1];
  for(auto i=column.first;i<column.first+column.count;++i){const auto&s=top.spans[i];int lo=s.lo+shift[2];const int hi=s.hi+shift[2];
   while(r<end&&map.runs[r].hi<=lo)++r;
   while(lo<hi){require(r<end&&map.runs[r].lo<=lo,"physics_count_gap");const auto stop=std::min(hi,int(map.runs[r].hi));
    require(stop>lo,"physics_count_run");out[map.runs[r].element]+=stop-lo;total+=stop-lo;lo=stop;if(lo>=map.runs[r].hi)++r;}
  }
 }
 require(total==top.cells,"physics_count_conservation");return out;
}
inline std::vector<Element> intersect(const Map&map,const Topology&remaining,std::uint32_t component){
 require(component>0&&component<remaining.components.size(),"physics_partition_component");
 require(std::isfinite(map.pitch)&&std::isfinite(remaining.pitch)&&
         std::abs(map.pitch-remaining.pitch)<=1e-12,"physics_partition_pitch");
 // Asset expands the damage Grid around source geometry. Its original immutable
 // anatomy remains on this source lattice, so an integer origin shift is exact.
 std::array<int,3> shift{};
 for(int a=0;a<3;++a){
  const double cells=(remaining.origin[a]-map.origin[a])/map.pitch;
  require(std::isfinite(cells)&&std::abs(cells)<=1000000&&
          std::abs(cells-std::nearbyint(cells))<=1e-6,"physics_partition_lattice");
  shift[a]=int(std::nearbyint(cells));
 }
 std::map<std::uint16_t,Element> elements;std::uint64_t covered=0;
 for(const auto&c:remaining.columns){
  const int mx=c.x+shift[0],my=c.y+shift[1];
  // Empty expanded columns are skipped by the sparse topology; a selected
  // anatomy column outside the original map is a damaged-source mismatch.
  if(mx<0||my<0||mx>=int(map.size[0])||my>=int(map.size[1])){
   for(auto i=c.first;i<c.first+c.count;++i)
    require(remaining.spans[i].component!=component,"physics_partition_outside_original_anatomy");
   continue;
  }
  const std::size_t col=std::size_t(my)*map.size[0]+mx;
  auto begin=map.offsets[col],end=map.offsets[col+1];
  for(auto i=c.first;i<c.first+c.count;++i){const auto&s=remaining.spans[i];if(s.component!=component)continue;
   int cursor=s.lo;auto r=begin;
   while(r<end&&int(map.runs[r].hi)-shift[2]<=cursor)++r;
   while(cursor<s.hi){require(r<end&&int(map.runs[r].lo)-shift[2]<=cursor,
                                  "physics_partition_unmapped_anatomy");
    const auto&source=map.runs[r];const int hi=std::min<int>(s.hi,int(source.hi)-shift[2]);
    require(hi>cursor,"physics_partition_run_order");
    auto&entry=elements[source.element];entry.bone=source.element;
    auto&volume=entry.volume;
    if(volume.components.size()==1){volume.size=remaining.size;volume.origin=remaining.origin;
        volume.pitch=remaining.pitch;volume.components.push_back({});}
    if(volume.columns.empty()||volume.columns.back().x!=c.x||volume.columns.back().y!=c.y)
       volume.columns.push_back({c.x,c.y,std::uint32_t(volume.spans.size()),0});
    auto&column=volume.columns.back();
    if(column.count&&volume.spans.back().hi==cursor)volume.spans.back().hi=hi;
    else{require(volume.spans.size()<4000000,"physics_partition_span_budget");
      volume.spans.push_back({cursor,hi,1});++column.count;}
    const auto n=std::uint64_t(hi-cursor);volume.cells+=n;volume.components[1].cells+=n;covered+=n;
    for(int axis=0;axis<3;++axis){const int lo=axis==0?c.x:(axis==1?c.y:cursor);
      const int upper=axis==0?c.x+1:(axis==1?c.y+1:hi);
      volume.components[1].lo[axis]=std::min(volume.components[1].lo[axis],lo);
      volume.components[1].hi[axis]=std::max(volume.components[1].hi[axis],upper);}
    cursor=hi;if(cursor>=int(source.hi)-shift[2])++r;}
  }
 }
 require(covered==remaining.components[component].cells,"physics_partition_cell_conservation");
 std::vector<Element> out;out.reserve(elements.size());
 for(auto&entry:elements){
  const auto total=map.original_cells.find(entry.first);
  require(total!=map.original_cells.end()&&total->second>=entry.second.volume.cells,
          "physics_partition_original_element_count");
  entry.second.original_shape_complete=(entry.second.volume.cells==total->second);
  out.push_back(std::move(entry.second));}
 return out;
}
// ---- Fused element partition (separation-track P1.6) ------------------------
// partitionFused(map,top,ids,remaining,parts) returns exactly what
// remainingCounts(map,top) returns in `remaining`, and what intersect(map,top,id)
// returns in parts[id] for every id of `ids`, from ONE pass over the topology's
// columns (contiguous column ranges on <= maxThreads threads, concatenated in
// column order). It returns false (outputs unspecified) whenever a check of
// remainingCounts() or of one of those intersect() calls would fail, or when the
// map runs of a visited column are not sorted and disjoint (never for a decoded or
// built map); the caller then runs the original functions, which fail (or not)
// with their own messages in their own order.
// Why the outputs are equal:
//  * both originals walk the topology columns in order and split every span at
//    the map runs of its column; remainingCounts keeps its run cursor across the
//    spans of a column and intersect restarts it per span, and with sorted,
//    disjoint runs both skip to the first run that ends after the span start, so
//    the pieces (element, cells) are the same;
//  * an element volume receives its pieces in the same order under the column-push
//    and same-column merge rules of intersect(); a topology column lies in one
//    column range, so concatenating ranges never merges across them; cell counts
//    are sums and component bounds are min/max;
//  * the per-element span budget (< 4,000,000 spans before each push) is checked on
//    the final span count, which only grows.
// Only types the originals already use (Element, Topology, std::map, std::vector)
// are instantiated: the kernel translation unit is close to the COFF section limit
// of a build without /bigobj.
struct FusedRange {bool ok{true},ran{};std::uint64_t total{};std::vector<std::uint64_t>counts,covered;std::vector<std::vector<Element>>volumes;};
inline void fusedScan(const Map&map,const Topology&top,const std::vector<int>&slot,const std::array<int,3>&shift,
                      std::size_t first,std::size_t last,FusedRange&range){
 const std::size_t slots=range.covered.size();
 int cachedSlot=-1;Element*cached=nullptr;
 for(std::size_t ci=first;ci<last;++ci){const auto&c=top.columns[ci];
  if(((ci-first)&8191)==0)gfy_live::sep::throwIfCanceled(); // P1.5: a canceled job only aborts
  const int x=c.x+shift[0],y=c.y+shift[1];
  if(x<0||y<0||x>=int(map.size[0])||y>=int(map.size[1])){range.ok=false;return;}
  const auto key=std::size_t(y)*map.size[0]+std::size_t(x);const auto begin=map.offsets[key],end=map.offsets[key+1];
  if(begin>end||end>map.runs.size()){range.ok=false;return;}
  for(auto r=begin;r<end;++r)if(!(map.runs[r].lo<map.runs[r].hi&&(r+1==end||map.runs[r].hi<=map.runs[r+1].lo))){range.ok=false;return;}
  auto r=begin;
  for(auto i=c.first;i<c.first+c.count;++i){const auto&s=top.spans[i];
   const int k=s.component<slot.size()?slot[s.component]:-1;
   int lo=s.lo+shift[2];const int hi=s.hi+shift[2];
   while(r<end&&map.runs[r].hi<=lo)++r;
   while(lo<hi){
    if(!(r<end&&map.runs[r].lo<=lo)){range.ok=false;return;}
    const int stop=std::min(hi,int(map.runs[r].hi));
    if(!(stop>lo)){range.ok=false;return;}
    const auto element=map.runs[r].element;const auto n=std::uint64_t(stop-lo);
    range.counts[element]+=n;range.total+=n;
    if(k>=0&&std::size_t(k)<slots){
     if(k!=cachedSlot||!cached||cached->bone!=element){auto&list=range.volumes[std::size_t(k)];cached=nullptr;
      for(auto&e:list)if(e.bone==element){cached=&e;break;}
      if(!cached){list.emplace_back();cached=&list.back();cached->bone=element;auto&v=cached->volume;
       v.size=top.size;v.origin=top.origin;v.pitch=top.pitch;v.components.push_back({});}
      cachedSlot=k;}
     auto&v=cached->volume;const int zlo=lo-shift[2],zhi=stop-shift[2];
     if(v.columns.empty()||v.columns.back().x!=c.x||v.columns.back().y!=c.y)v.columns.push_back({c.x,c.y,std::uint32_t(v.spans.size()),0});
     auto&column=v.columns.back();
     if(column.count&&v.spans.back().hi==zlo)v.spans.back().hi=zhi;
     else{v.spans.push_back({zlo,zhi,1});++column.count;}
     v.cells+=n;range.covered[std::size_t(k)]+=n;auto&b=v.components[1];b.cells+=n;
     b.lo[0]=std::min(b.lo[0],c.x);b.hi[0]=std::max(b.hi[0],c.x+1);
     b.lo[1]=std::min(b.lo[1],c.y);b.hi[1]=std::max(b.hi[1],c.y+1);
     b.lo[2]=std::min(b.lo[2],zlo);b.hi[2]=std::max(b.hi[2],zhi);
    }
    lo=stop;if(lo>=int(map.runs[r].hi))++r;}
  }
 }
}
inline bool partitionFused(const Map&map,const Topology&top,const std::vector<std::uint32_t>&ids,
    std::map<std::uint16_t,std::uint64_t>&remaining,std::map<std::uint32_t,std::vector<Element>>&parts,unsigned maxThreads=4){
 remaining.clear();parts.clear();
 if(!(std::isfinite(map.pitch)&&std::isfinite(top.pitch)&&std::abs(map.pitch-top.pitch)<=1e-12))return false;
 std::array<int,3>shift{};
 for(unsigned a=0;a<3;++a){const auto d=(top.origin[a]-map.origin[a])/map.pitch;
  if(!(std::isfinite(d)&&std::abs(d)<1000000&&std::abs(d-std::nearbyint(d))<1e-6))return false;shift[a]=int(std::nearbyint(d));}
 std::vector<int>slot(top.components.size(),-1);
 for(std::size_t k=0;k<ids.size();++k){const auto id=ids[k];if(!(id>0&&id<top.components.size())||slot[id]>=0)return false;slot[id]=int(k);}
 if(map.offsets.size()!=std::size_t(map.size[0])*map.size[1]+1)return false;
 constexpr std::size_t maxRanges=4;std::array<FusedRange,maxRanges>ranges;
 gfy_live::sep::parallel(top.columns.size(),std::size_t(1)<<15,std::min<unsigned>(maxThreads,unsigned(maxRanges)),[&](std::size_t first,std::size_t last,std::size_t chunk){
  auto&range=ranges[chunk];range.ran=true;range.counts.assign(65536,0);range.covered.assign(ids.size(),0);range.volumes.resize(ids.size());
  fusedScan(map,top,slot,shift,first,last,range);});
 std::size_t used=0;while(used<maxRanges&&ranges[used].ran)++used; // chunks 0..used-1 ran
 std::uint64_t total=0;
 for(std::size_t c=0;c<used;++c){if(!ranges[c].ok)return false;total+=ranges[c].total;}
 if(total!=top.cells)return false;
 for(std::size_t k=0;k<ids.size();++k){std::uint64_t covered=0;for(std::size_t c=0;c<used;++c)covered+=ranges[c].covered[k];
  if(covered!=top.components[ids[k]].cells)return false;}
 for(std::size_t e=0;e<65536;++e){std::uint64_t n=0;for(std::size_t c=0;c<used;++c)n+=ranges[c].counts[e];if(n)remaining[std::uint16_t(e)]=n;}
 // split2 part 10 (2026-11-12): the loop below put every element's volume together on one thread, growing the vectors as it
 // went. Here the elements are listed first (same order, same output slots), then each one is put together by the former
 // statements with its final sizes reserved, the elements side by side (each thread takes the next element; an element
 // writes only its own slot). A failed check makes the function return false, as before. GFY_SEP_PARTITION_MERGE=0: former.
 static const bool mergeSideBySide=[]{const char*e=std::getenv("GFY_SEP_PARTITION_MERGE");return !e||std::atoi(e)!=0;}();
 if(mergeSideBySide){
  struct MergeTask {std::size_t k;Element*element;};std::vector<MergeTask> tasks;
  for(std::size_t k=0;k<ids.size();++k){
   std::vector<std::uint32_t>bones;
   for(std::size_t c=0;c<used;++c)for(const auto&e:ranges[c].volumes[k])bones.push_back(e.bone);
   std::sort(bones.begin(),bones.end());bones.erase(std::unique(bones.begin(),bones.end()),bones.end());
   auto&out=parts[ids[k]];out.resize(bones.size());
   for(std::size_t j=0;j<bones.size();++j){out[j].bone=std::uint16_t(bones[j]);tasks.push_back({k,&out[j]});}}
  std::atomic<std::size_t> next{0};std::atomic<bool> refused{false};
  const unsigned threads=std::min<unsigned>(std::min<unsigned>(maxThreads,4u),unsigned(std::max<std::size_t>(1,tasks.size())));
  gfy_live::sep::parallel(threads,1,threads,[&](std::size_t,std::size_t,std::size_t){
   for(std::size_t n;(n=next.fetch_add(1,std::memory_order_relaxed))<tasks.size();){const std::size_t k=tasks[n].k;auto&element=*tasks[n].element;auto&volume=element.volume;
    volume.size=top.size;volume.origin=top.origin;volume.pitch=top.pitch;volume.components.push_back({});
    {std::size_t columns=0,spans=0;for(std::size_t c=0;c<used;++c)for(const auto&e:ranges[c].volumes[k])if(e.bone==element.bone){columns+=e.volume.columns.size();spans+=e.volume.spans.size();}
     volume.columns.reserve(columns);volume.spans.reserve(spans);}
    bool ok=true;
    for(std::size_t c=0;c<used&&ok;++c)for(auto&e:ranges[c].volumes[k]){if(e.bone!=element.bone)continue;auto&v=e.volume;
     const auto offset=std::uint32_t(volume.spans.size());
     for(auto column:v.columns){column.first+=offset;volume.columns.push_back(column);}
     volume.spans.insert(volume.spans.end(),v.spans.begin(),v.spans.end());
     if(volume.spans.size()>4000000){ok=false;break;}
     volume.cells+=v.cells;auto&b=volume.components[1];b.cells+=v.components[1].cells;
     for(int axis=0;axis<3;++axis){b.lo[axis]=std::min(b.lo[axis],v.components[1].lo[axis]);b.hi[axis]=std::max(b.hi[axis],v.components[1].hi[axis]);}
     v=Topology{};}
    if(ok){const auto original=map.original_cells.find(element.bone);
     if(original==map.original_cells.end()||original->second<volume.cells)ok=false;
     else element.original_shape_complete=(volume.cells==original->second);}
    if(!ok)refused.store(true,std::memory_order_relaxed);}});
  return !refused.load(std::memory_order_relaxed);
 }
 for(std::size_t k=0;k<ids.size();++k){
  // Element ids of this component, ascending (the std::map order of intersect()).
  std::vector<std::uint32_t>bones;
  for(std::size_t c=0;c<used;++c)for(const auto&e:ranges[c].volumes[k])bones.push_back(e.bone);
  std::sort(bones.begin(),bones.end());bones.erase(std::unique(bones.begin(),bones.end()),bones.end());
  auto&out=parts[ids[k]];out.resize(bones.size());
  for(std::size_t j=0;j<bones.size();++j){auto&element=out[j];element.bone=std::uint16_t(bones[j]);auto&volume=element.volume;
   volume.size=top.size;volume.origin=top.origin;volume.pitch=top.pitch;volume.components.push_back({});
   for(std::size_t c=0;c<used;++c)for(auto&e:ranges[c].volumes[k]){if(e.bone!=element.bone)continue;auto&v=e.volume;
    const auto offset=std::uint32_t(volume.spans.size());
    for(auto column:v.columns){column.first+=offset;volume.columns.push_back(column);}
    volume.spans.insert(volume.spans.end(),v.spans.begin(),v.spans.end());
    if(volume.spans.size()>4000000)return false;
    volume.cells+=v.cells;auto&b=volume.components[1];b.cells+=v.components[1].cells;
    for(int axis=0;axis<3;++axis){b.lo[axis]=std::min(b.lo[axis],v.components[1].lo[axis]);b.hi[axis]=std::max(b.hi[axis],v.components[1].hi[axis]);}
    v=Topology{};}
   const auto original=map.original_cells.find(element.bone);
   if(original==map.original_cells.end()||original->second<volume.cells)return false;
   element.original_shape_complete=(volume.cells==original->second);}
 }
 return true;
}
}}
