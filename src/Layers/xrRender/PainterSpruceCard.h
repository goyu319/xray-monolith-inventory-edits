#pragma once
#include "TreeCardTopology.h"
#include <cstdint>
#include <cstring>
#include <tuple>

// Scope the registry to real opaque tree visuals in the loaded level. The original level
// tables own the GPU objects. A missed source, unsupported declaration or allocation failure
// simply leaves the original draw in place.
namespace PainterSpruceCard
{
struct Window { u32 offset, vertices, triangles; };
using SectionKey=std::tuple<u32,u32,u32,u32,u32,u32,uintptr_t>;
using DrawKey=std::tuple<uintptr_t,uintptr_t,uintptr_t,u32,u32,u32,u32>;
struct Section {
    std::vector<Window> windows;
    std::vector<TreeCardTopology::Point> positions;
    ID3DVertexBuffer* vertices=nullptr;
};
struct Draw { ID3DIndexBuffer* indices; u32 first,triangles; };
inline std::map<SectionKey,Section> sections;
inline std::map<DrawKey,Draw> burned;
inline void forget(){sections.clear();burned.clear();}
inline bool isStage(const char* stage){
    return stage && (!_stricmp(stage,"flora\\trunk_wave") || !_stricmp(stage,"flora\\trunk"));
}
inline DrawKey drawKey(const void* shader,const void* vb,const void* ib,u32 vbase,u32 nv,u32 start,u32 tris){
    return {uintptr_t(shader),uintptr_t(vb),uintptr_t(ib),vbase,nv,start,tris};
}
inline bool readExact(IReader* r,u32 id,void* out,u32 bytes){if(r->find_chunk(id)!=bytes)return false;r->r(out,bytes);return true;}
inline void registerVisual(IReader* visual,const std::vector<const void*>& barkShaders,const xr_vector<FSlideWindowItem>& swis) {
    ogf_header h;
    if(!readExact(visual,OGF_HEADER,&h,sizeof(h)) || (h.type!=MT_TREE_ST && h.type!=MT_TREE_PM) ||
       h.shader_id>=barkShaders.size() || !barkShaders[h.shader_id])return;
    u32 f[6];
    if(!readExact(visual,OGF_GCONTAINER,f,sizeof(f))||f[2]>65536||!f[2]||f[5]<42||f[5]%3)return;
    auto key=SectionKey(f[0],f[1],f[2],f[3],f[4],f[5],uintptr_t(barkShaders[h.shader_id]));
    // A geometry section can be referenced with distinct windows by different progressive visuals.
    auto& sec=sections[key];
    auto addWindow=[&](Window w){
        if(!w.triangles || !w.vertices || w.vertices>f[2] || w.offset>f[5] || w.triangles>(f[5]-w.offset)/3)return;
        auto it=std::find_if(sec.windows.begin(),sec.windows.end(),[&](const Window& a){
            return a.offset==w.offset && a.vertices==w.vertices && a.triangles==w.triangles;});
        if(it==sec.windows.end())sec.windows.push_back(w);
    };
    if(h.type==MT_TREE_ST)addWindow({0,f[2],f[5]/3});
    else {
        u32 id;
        if(!readExact(visual,OGF_SWICONTAINER,&id,sizeof(id))||id>=swis.size())return;
        const auto& s=swis[id];
        for(u32 i=0;i<s.count;i++)addWindow({s.sw[i].offset,s.sw[i].num_verts,s.sw[i].num_tris});
    }
}
inline void vertices(u32 pool,u32 count,u32 stride,const D3DVERTEXELEMENT9* decl,
                     ID3DVertexBuffer* vb,const BYTE* bytes) {
    if(!vb)return;
    unsigned off=stride;
    for(const auto* e=decl;e->Stream!=0xff;++e)
        if(e->Stream==0 && e->Usage==D3DDECLUSAGE_POSITION && e->UsageIndex==0 && e->Type==D3DDECLTYPE_FLOAT3)
            {off=e->Offset;break;}
    if(off>stride || stride-off<sizeof(float)*3)return;
    for(auto& rec:sections){
        const auto& [vp,base,n,ip,ib,ni,sh]=rec.first;auto& sec=rec.second;
        if(vp!=pool || base>count || n>count-base || sec.windows.empty())continue;
        sec.vertices=vb;sec.positions.resize(n);
        const BYTE* data=bytes+size_t(base)*stride+off;
        for(u32 i=0;i<n;++i)std::memcpy(sec.positions[i].data(),data+size_t(i)*stride,sizeof(float)*3);
    }
}
template<typename CreateBuffer>
inline void indices(u32 pool,u32 count,ID3DIndexBuffer* ib,const BYTE* bytes,CreateBuffer create) {
    if(!ib)return;
    std::vector<u16> packed;
    std::map<DrawKey,Draw> ready;
    for(auto& rec:sections){
        const auto& [vp,vBase,nv,ip,iBase,ni,sh]=rec.first;auto& sec=rec.second;
        if(ip!=pool || !sec.vertices || sec.windows.empty() || iBase>count || ni>count-iBase)continue;
        // The finest window owns eligibility. A degenerating low-LOD branch cannot emerge as a new card.
        std::stable_sort(sec.windows.begin(),sec.windows.end(),[](const Window& a,const Window& b){return a.triangles>b.triangles;});
        TreeCardTopology::Cards original,found;
        std::vector<u16> compact;
        for(size_t k=0;k<sec.windows.size();k++){
            const auto& w=sec.windows[k];const u16* first=reinterpret_cast<const u16*>(bytes)+iBase+w.offset;
            if(!TreeCardTopology::classify(sec.positions,w.vertices,first,w.triangles*3,k?&original:nullptr,found,compact)){
                if(!k)break;
                continue;
            }
            if(!k)original=found;
            if(compact.size()+packed.size()>std::numeric_limits<u32>::max()/sizeof(u16))break;
            auto key=drawKey(reinterpret_cast<const void*>(sh),sec.vertices,ib,vBase,w.vertices,iBase+w.offset,w.triangles);
            ready[key]={nullptr,(u32)packed.size(),(u32)compact.size()/3};
            packed.insert(packed.end(),compact.begin(),compact.end());
        }
    }
    if(packed.empty())return;
    ID3DIndexBuffer* compact=create(packed.data(),(u32)(packed.size()*sizeof(u16)));
    if(!compact)return;
    for(auto& item:ready){item.second.indices=compact;burned.insert(item);}
}
inline const Draw* query(const void* shader,const void* vb,const void* ib,u32 base,u32 nv,u32 first,u32 tris){
    auto it=burned.find(drawKey(shader,vb,ib,base,nv,first,tris));
    return it==burned.end()?nullptr:&it->second;
}
}
