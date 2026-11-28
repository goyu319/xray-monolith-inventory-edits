#pragma once
#ifdef USE_DX11
#include "PainterStaticBloodOutputs.h"
#include "PainterSoftShader.h"
#include "BloodThermalWallmark.h"
#include "ResourceManager.h"
#include "dxRenderDeviceRender.h"
#include <algorithm>
#include <memory>
#include <cstdint>
#include <limits>

namespace painter_static_blood_gpu {
constexpr unsigned StampCount=512, StampBytes=32;
inline bool valid_extent(unsigned w,unsigned h){return w && h && w<=D3D11_REQ_TEXTURE2D_U_OR_V_DIMENSION && h<=D3D11_REQ_TEXTURE2D_U_OR_V_DIMENSION;}
inline bool valid_rows(unsigned first,unsigned count){return first<=StampCount && count<=StampCount-first;}
inline std::size_t charged(std::size_t n){return n?((n+65535u)/65536u)*65536u:0;}
inline std::size_t bytes(unsigned nv,unsigned w,unsigned h){
    return charged(std::size_t(nv)*sizeof(FVF::LIT))+charged(std::size_t(w)*h*4)+charged(std::size_t(w)*h*2)+charged(StampCount*StampBytes);
}
inline std::size_t dirty_bytes(unsigned w,unsigned h,unsigned stamps){
    return charged(std::size_t(w)*h*4)+charged(std::size_t(w)*h*2)+charged(std::size_t(stamps)*StampBytes);
}
struct Plane {
    ID3D11Texture2D* resource=nullptr;
    ref_texture binding;
    Plane()=default;Plane(const Plane&)=delete;Plane& operator=(const Plane&)=delete;
    ~Plane(){binding.destroy();_RELEASE(resource);}
    bool create(unsigned w,unsigned h,DXGI_FORMAT format,const void* initial,unsigned pitch,bool shader_input) {
        D3D11_TEXTURE2D_DESC d={};d.Width=w;d.Height=h;d.MipLevels=1;d.ArraySize=1;
        d.Format=format;d.SampleDesc.Count=1;d.Usage=D3D11_USAGE_DEFAULT;d.BindFlags=shader_input?D3D11_BIND_SHADER_RESOURCE:0;
        D3D11_SUBRESOURCE_DATA seed={};seed.pSysMem=initial;seed.SysMemPitch=pitch;
        if(FAILED(HW.pDevice->CreateTexture2D(&d,initial?&seed:nullptr,&resource)))return false;
        if(!shader_input)return true;
        ID3D11ShaderResourceView* srv=nullptr;
        if(FAILED(HW.pDevice->CreateShaderResourceView(resource,nullptr,&srv)))return false;
        binding=xr_new<CTexture>();string128 n;xr_sprintf(n,"$user$static_blood_%p",this);binding->set_name(n);
        const bool ok=binding->surface_set_prepared(resource,srv);_RELEASE(srv);
        return ok;
    }
};
struct Patch {
    Plane color,ids,stamps;
    ID3D11Device* device=nullptr;ID3D11Buffer* vb=nullptr;
    ref_geom geometry;ref_shader material;
    unsigned width=0,height=0,count=0;
    ~Patch(){material.destroy();geometry.destroy();_RELEASE(vb);}
};
struct Clock : R_constant_setup {
    void setup(R_constant* c) override{
        const u32 n=Device.dwTimeGlobal;
        RCache.set_c(c,float(n&65535u),float(n>>16),heat_vision_cooldown_time,ps_r2_heatvision<=0?0.f:heat_vision_cooldown?1.f:2.f);
    }
};
inline Clock& clock(){static Clock s;return s;}
inline unsigned supported(const ref_shader& src){
    if(!src||!src->E[0]||src->E[0]->passes.size()!=1||!BloodThermalWallmark::Compatible(src))return 0;
    const auto& p=src->E[0]->passes[0];
    if(!p->vs||!p->vs->vs||!p->ps||!p->ps->ps||!p->constants||
        (p->gs&&p->gs->gs)||(p->hs&&p->hs->sh)||(p->ds&&p->ds->sh)||(p->cs&&p->cs->sh))return 0;
    const char* vs=*p->vs->cName;const char* name="effects_wallmark_blood";
    if(xr_strcmp(vs,name) && xr_strcmp(vs,"effects_wallmark_blood_0"))return 0;
    // A replacement program with additional texture inputs can implement a different appearance.
    for(const auto& c:p->ps->constants.table){
        if(c->type==RC_dx10texture && xr_strcmp(*c->name,"s_base"))return 0;
    }
    if(!p->ps->constants.get("ssfx_blood_decals"))return 0;
    const unsigned outputs=painter_static_blood_outputs::mask(p->ps->ps);
    return (outputs==1u||outputs==3u)?outputs:0u;
}
struct Programs {
    ID3D11Device* device=nullptr;unsigned samples=0;bool attempted=false;
    ref_ps table[2][9];
};
inline Programs& programs(){static Programs* p=new Programs;return *p;}
inline void clear_programs(){programs()=Programs();}
inline void warm_programs(){
    if(!HW.pDevice||!DEV)return;
    auto& cache=programs();
    const unsigned samples=RImplementation.o.dx10_msaa?std::min<unsigned>(8,RImplementation.o.dx10_msaa_samples):0;
    if(cache.attempted && cache.device==HW.pDevice && cache.samples==samples)return;
    clear_programs();cache.device=HW.pDevice;cache.samples=samples;
    if(!FS.exist("$game_shaders$","r3\\painter_static_blood.h")){cache.attempted=true;return;}
    struct Restore {int ms=::Render->m_MSAASample;~Restore(){::Render->m_MSAASample=ms;}} restore;
    for(unsigned mode=0;mode<2;++mode){
        const char* program=mode?"painter_static_blood":"painter_static_blood_color";
        const char* filename=mode?"r3\\painter_static_blood.ps":"r3\\painter_static_blood_color.ps";
        if(!FS.exist("$game_shaders$",filename))continue;
        for(int sample=-1;sample<int(samples);++sample){
            ::Render->m_MSAASample=sample;
            PainterSoftShaderGuard soft;ref_ps program_ref=DEV->_CreatePS(program);
            if(program_ref && program_ref->ps)cache.table[mode][sample+1]=program_ref;
        }
    }
    cache.attempted=true;
}
inline const ref_ps* find_program(const ref_shader& src,unsigned outputs){
    const auto& cache=programs();if(!outputs||cache.device!=HW.pDevice||!cache.attempted)return nullptr;
    const char* name=*src->E[0]->passes[0]->ps->cName;const auto len=strlen(name);
    const unsigned sample=(len>=2 && name[len-2]=='_' && name[len-1]>='0' && name[len-1]<='7')?unsigned(name[len-1]-'0')+1:0;
    const ref_ps& p=cache.table[outputs==3u][sample];
    return p && p->ps?&p:nullptr;
}
inline bool compatible(const ref_shader& src){return find_program(src,supported(src))!=nullptr;}
inline bool clone(Patch& slot,const ref_shader& src,unsigned outputs){
    const auto& op=src->E[0]->passes[0];
    const ref_ps* cached=find_program(src,outputs);if(!cached)return false;
    xrCriticalSectionTryGuard guard(DEV->PainterBloodCreationGuard());if(!guard.owns_lock())return false;
    SPass pass;pass.state=op->state;pass.vs=op->vs;pass.C=op->C;pass.gs=op->gs;pass.hs=op->hs;pass.ds=op->ds;pass.cs=op->cs;
    pass.ps=*cached;
    if(!pass.ps||!pass.ps->ps)return false;
    R_constant_table ct;ct.merge(&pass.ps->constants);ct.merge(&pass.vs->constants);
    STextureList textures;unsigned bound=0;bool liveClock=false;
    for(auto& c:ct.table){
        const char* n=*c->name;const ref_texture* binding=nullptr;
        if(!xr_strcmp(n,"s_static_blood_color"))binding=&slot.color.binding;
        else if(!xr_strcmp(n,"s_static_blood_owner"))binding=&slot.ids.binding;
        else if(!xr_strcmp(n,"s_static_blood_stamps"))binding=&slot.stamps.binding;
        if(binding){
            if(c->type!=RC_dx10texture)return false;
            textures.emplace_back(c->samp.index,*binding);++bound;continue;
        }
        if(!xr_strcmp(n,"painter_static_blood_clock")){if(c->type!=RC_float)return false;c->handler=&clock();liveClock=true;continue;}
        auto* prior=op->constants->get(n);
        if(!prior||prior->type!=c->type)return false;
        c->handler=prior->handler;
        if(c->type==RC_sampler&&c->samp.index!=prior->samp.index)return false;
        if(c->type==RC_dx10texture)return false;
    }
    if(bound!=3||!liveClock)return false;
    pass.constants=DEV->_CreateConstantTable(ct);pass.T=DEV->_CreateTextureList(textures);
    ShaderElement element;element._copy(*src->E[0]);element.passes.clear();element.passes.push_back(DEV->_CreatePass(pass));
    Shader assembled;assembled.E[0]=DEV->_CreateElement(element);slot.material=DEV->_CreateShader(&assembled);
    return !!slot.material;
}
inline std::shared_ptr<Patch> allocate(const ref_shader& src,const void* vertices,unsigned nv,unsigned w,unsigned h,
        const void* rgba,const void* ids,const void* paddedStamps) {
    static_assert(sizeof(FVF::LIT)==24,"wallmark vertex layout");
    if(!HW.pDevice||!vertices||!nv||nv%3||nv>UINT_MAX/sizeof(FVF::LIT)||!valid_extent(w,h))return {};
    const unsigned outputs=supported(src);if(!find_program(src,outputs))return {};
    auto p=std::make_shared<Patch>();p->device=HW.pDevice;p->width=w;p->height=h;p->count=nv;
    D3D11_BUFFER_DESC d={};d.ByteWidth=nv*sizeof(FVF::LIT);d.Usage=D3D11_USAGE_IMMUTABLE;d.BindFlags=D3D11_BIND_VERTEX_BUFFER;
    D3D11_SUBRESOURCE_DATA seed={};seed.pSysMem=vertices;
    if(FAILED(HW.pDevice->CreateBuffer(&d,&seed,&p->vb)))return {};
    p->geometry.create(FVF::F_LIT,p->vb,nullptr);
    if(!p->geometry||!p->color.create(w,h,DXGI_FORMAT_R8G8B8A8_UNORM,rgba,w*4,true)||
        !p->ids.create(w,h,DXGI_FORMAT_R16_UINT,ids,w*2,true)||
        !p->stamps.create(2,StampCount,DXGI_FORMAT_R32G32B32A32_UINT,paddedStamps,StampBytes,true)||!clone(*p,src,outputs))return {};
    return p;
}
inline std::shared_ptr<Patch> create_blank(const ref_shader& src,const void* vertices,unsigned nv,unsigned w,unsigned h){
    return allocate(src,vertices,nv,w,h,nullptr,nullptr,nullptr);
}
inline std::shared_ptr<Patch> create(const ref_shader& src,const void* vertices,unsigned nv,unsigned w,unsigned h,
        const void* rgba,const void* ids,const void* stamps,unsigned live_stamp_count){
    if(!rgba||!ids||!stamps||!live_stamp_count||live_stamp_count>StampCount)return {};
    std::uint32_t padded[StampCount*StampBytes/sizeof(std::uint32_t)]={};
    memcpy(padded,stamps,std::size_t(live_stamp_count)*StampBytes);
    return allocate(src,vertices,nv,w,h,rgba,ids,padded);
}
inline bool ready(const Patch& p){return p.device==HW.pDevice && p.vb && p.geometry && p.material;}
inline void draw(const Patch& p,BOOL suppress_cull=FALSE){
    // Called in the original slot after its depth bias and optional heat MRT are established.
    RCache.set_Shader(p.material._get());RCache.set_Geometry(p.geometry._get());
    if(suppress_cull)RCache.set_CullMode(CULL_NONE);
    RCache.Render(D3DPT_TRIANGLELIST,0,p.count/3);
    if(suppress_cull)RCache.set_CullMode(CULL_CCW);
    Device.Statistic->RenderDUMP_WMT_Count+=p.count/3;
}
struct Pending {
    Plane color,ids,stamps;
    const Patch* owner=nullptr;ID3D11Device* device=nullptr;
    unsigned x=0,y=0,w=0,h=0,first=0,n=0,row=0,stampRow=0;
    bool committed=false;
};
inline std::unique_ptr<Pending> begin_update(Patch& p,unsigned x,unsigned y,unsigned w,unsigned h,unsigned first,unsigned n){
    if(!ready(p)||!valid_extent(w,h)||x>p.width||w>p.width-x||y>p.height||h>p.height-y||!valid_rows(first,n))return {};
    auto q=std::make_unique<Pending>();q->owner=&p;q->device=p.device;q->x=x;q->y=y;q->w=w;q->h=h;q->first=first;q->n=n;
    if(!q->color.create(w,h,DXGI_FORMAT_R8G8B8A8_UNORM,nullptr,w*4,false)||!q->ids.create(w,h,DXGI_FORMAT_R16_UINT,nullptr,w*2,false)||
       (n&&!q->stamps.create(2,n,DXGI_FORMAT_R32G32B32A32_UINT,nullptr,StampBytes,false)))return {};
    return q;
}
inline void write_rows(Plane& target,unsigned x,unsigned y,unsigned width,unsigned rows,const void* data,unsigned pitch){
    D3D11_BOX box={x,y,0,x+width,y+rows,1};HW.pContext->UpdateSubresource(target.resource,0,&box,data,pitch,0);
}
// The owner keeps this patch private until all live clock rows and both full images have arrived.
inline bool pump_initial(Patch& p,unsigned& row,unsigned& stampRow,const void* rgba,const void* ids,
        const void* stamps,unsigned live_stamp_count,unsigned& byteBudget){
    if(!ready(p)||!rgba||!ids||!stamps||!live_stamp_count||live_stamp_count>StampCount||
       row>p.height||stampRow>live_stamp_count)return false;
    unsigned available=(std::min)(byteBudget,256u*1024u);
    const unsigned startBudget=available;
    const unsigned colorPitch=p.width*4,idPitch=p.width*2,rowBytes=colorPitch+idPitch;
    const unsigned rows=(std::min)(p.height-row,available/rowBytes);
    if(rows){
        write_rows(p.color,0,row,p.width,rows,(const u8*)rgba+std::size_t(row)*colorPitch,colorPitch);
        write_rows(p.ids,0,row,p.width,rows,(const u8*)ids+std::size_t(row)*idPitch,idPitch);
        row+=rows;available-=rows*rowBytes;
    }
    const unsigned sr=(std::min)(live_stamp_count-stampRow,available/StampBytes);
    if(sr){
        write_rows(p.stamps,0,stampRow,2,sr,(const u8*)stamps+std::size_t(stampRow)*StampBytes,StampBytes);
        stampRow+=sr;available-=sr*StampBytes;
    }
    byteBudget-=startBudget-available;
    return row==p.height && stampRow==live_stamp_count;
}
inline bool pump(Pending& q,const void* rgba,unsigned color_pitch,const void* ids,unsigned id_pitch,const void* stamps,unsigned& budget){
    if(q.committed||q.device!=HW.pDevice||!rgba||!ids||(q.n&&!stamps)||color_pitch<q.w*4||id_pitch<q.w*2)return false;
    const unsigned stride=q.w*6,rows=(std::min)(q.h-q.row,budget/stride);
    if(rows){
        write_rows(q.color,0,q.row,q.w,rows,(const u8*)rgba+std::size_t(q.row)*color_pitch,color_pitch);
        write_rows(q.ids,0,q.row,q.w,rows,(const u8*)ids+std::size_t(q.row)*id_pitch,id_pitch);
        q.row+=rows;budget-=rows*stride;
    }
    const unsigned sr=(std::min)(q.n-q.stampRow,budget/StampBytes);
    if(sr){write_rows(q.stamps,0,q.stampRow,2,sr,(const u8*)stamps+std::size_t(q.stampRow)*StampBytes,StampBytes);q.stampRow+=sr;budget-=sr*StampBytes;}
    return q.row==q.h&&q.stampRow==q.n;
}
inline bool commit(Patch& p,Pending& q){
    if(!ready(p)||q.owner!=&p||q.device!=p.device||q.committed||q.row!=q.h||q.stampRow!=q.n)return false;
    HW.pContext->CopySubresourceRegion(p.color.resource,0,q.x,q.y,0,q.color.resource,0,nullptr);
    HW.pContext->CopySubresourceRegion(p.ids.resource,0,q.x,q.y,0,q.ids.resource,0,nullptr);
    if(q.n)HW.pContext->CopySubresourceRegion(p.stamps.resource,0,0,q.first,0,q.stamps.resource,0,nullptr);
    q.committed=true;return true;
}
inline bool update(Patch& p,unsigned x,unsigned y,unsigned w,unsigned h,const void* rgba,unsigned color_pitch,const void* ids,unsigned id_pitch,
        const void* stamps,unsigned first,unsigned n){
    if(!ready(p)||!valid_extent(w,h)||x>p.width||w>p.width-x||y>p.height||h>p.height-y||!valid_rows(first,n)||
       !rgba||!ids||(n&&!stamps)||color_pitch<w*4||id_pitch<w*2)return false;
    write_rows(p.color,x,y,w,h,rgba,color_pitch);write_rows(p.ids,x,y,w,h,ids,id_pitch);
    if(n)write_rows(p.stamps,0,first,2,n,stamps,StampBytes);
    return true;
}
}
#endif
