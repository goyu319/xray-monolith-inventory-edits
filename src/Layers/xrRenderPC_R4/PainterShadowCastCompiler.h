#pragma once
#include "PainterBurnTearCompiler.h"
#include "PainterShadowCastDepth.h"
namespace painter_shadow_cast {
using Microsoft::WRL::ComPtr;
using painter_tear_modules::Cache;
using painter_tear_modules::Module;
using painter_tear_modules::SharedLibrary;
using painter_burn_tear_link::Pin;
using painter_burn_tear_link::Wire;
using painter_burn_tear_link::Bound;
struct Recipe {
    std::array<SharedLibrary,2> parts{};
    const char* certified=nullptr;
    bool ready() const {return certified && parts[0] && parts[1];}
};
static constexpr const char* names[]={"fittedShadow","residualShadow"};
static constexpr int parameterCounts[]={14,16};
static constexpr Pin inputPins[]={{"painter_rest","PAINTER_REST",3,false},{"painter_K0","PAINTER_SROW0",3,false},{"painter_K1","PAINTER_SROW1",3,false},{"painter_K2","PAINTER_SROW2",3,false},{"painter_K3","PAINTER_SROW3",3,false},{"painter_clip","PAINTER_CLIP",4,false},{"painter_box","PAINTER_BOXU",1,false},{"hpos","SV_Position",4,false},{"primitive","SV_PrimitiveID",1,true}};
static constexpr Pin outputPins[]={{"res_c","SV_Target0",4,false},{"res_depth","SV_DepthGreaterEqual",1,false}};
static constexpr Wire wires[]={{-1,0,0,0},{-1,1,0,1},{-1,2,0,2},{-1,3,0,3},{-1,4,0,4},{-1,5,0,5},{-1,6,0,6},{-1,7,0,7},{-1,8,0,8},{-1,0,1,0},{-1,1,1,1},{-1,2,1,2},{-1,3,1,3},{-1,4,1,4},{-1,5,1,5},{-1,6,1,6},{-1,7,1,7},{-1,8,1,8},{0,9,1,9},{0,10,1,10},{0,11,1,11},{0,12,1,12},{0,13,1,13},{1,14,2,0},{1,15,2,1}};
static constexpr Bound slots[]={{"s_painter_removal_pages",0,1,D3D_SIT_TEXTURE},{"s_painter_removal_data",1,1,D3D_SIT_TEXTURE},{"s_painter_slits",2,1,D3D_SIT_TEXTURE},{"s_anatomy_pages",3,1,D3D_SIT_TEXTURE},{"s_anatomy_data",4,1,D3D_SIT_TEXTURE},{"s_tear_params",5,1,D3D_SIT_TEXTURE},{"s_tear_map",6,1,D3D_SIT_TEXTURE},{"s_tear_bake",7,1,D3D_SIT_TEXTURE},{"s_painter_burn_field",8,1,D3D_SIT_TEXTURE},{"s_painter_burn_faces",9,1,D3D_SIT_TEXTURE},{"s_painter_burn_baked",10,1,D3D_SIT_TEXTURE},{"PainterTissueState",0,1,D3D_SIT_CBUFFER}};
inline const char* certifiedProfile(unsigned samples,const std::string& semantic){
    if(samples==1 && semantic=="680328e7f87674235a053f2392d5413ddcc097d8350a8b985d586beb622594fb")return "7f77adf4e342b3fca4b683accf18d6cffff17ec8c837ed0c8667bcbbf88cc5f4";
    if(samples==2 && semantic=="6ad91cc27b47816b5b9d2020ca7aada581d8fe41b9d9a2d5c84546de5eee892c")return "7f77adf4e342b3fca4b683accf18d6cffff17ec8c837ed0c8667bcbbf88cc5f4";
    if(samples==4 && semantic=="1dc9225392d54fc26a970d8e7d81828569739dac3ef9976a94a28e4ce1d86a81")return "7f77adf4e342b3fca4b683accf18d6cffff17ec8c837ed0c8667bcbbf88cc5f4";
    return nullptr;
}
inline bool replace(std::string& text, const char* from, const char* to) {
    const auto at=text.find(from);
    if(at==text.npos || text.find(from,at+std::strlen(from))!=text.npos)return false;
    text.replace(at,std::strlen(from),to);
    return true;
}
inline Recipe capture(Cache& cache,const char* name,const char* entry,const char* target,UINT flags,
    const void* source,size_t bytes,const D3D_SHADER_MACRO* defs,ID3DInclude* includes){
    if(!name||!entry||!target||!source||!includes||strcmp(name,"painter_burn_painter_vmi_shadow_tear")||
        strcmp(entry,"main")||strcmp(target,"ps_5_0")||flags!=D3DCOMPILE_PACK_MATRIX_ROW_MAJOR)return {};
    std::string active=painter_tear_link::strippedLines(painter_tear_modules::preprocess(source,bytes,defs,includes,"PAINTER_SHADOW_PARTS"));
    const std::string marker="bool pb_baked_caster (", entryText="painter_vms_out main (";
    auto a=active.find(marker),b=active.find(entryText);
    if(a==active.npos||b==active.npos||b<=a||active.find(marker,a+1)!=active.npos||active.find(entryText,b+1)!=active.npos)return {};
    auto prefix=active.substr(0,a),caster=active.substr(a,b-a),last=active.substr(b);
    std::string fields;
    if(!painter_tear_link::declareState(prefix,"painter_burn_bake",12,fields)||
       !painter_tear_link::declareState(prefix,"painter_burn_meta",13,fields)||
       !painter_tear_link::declareState(prefix,"painter_burn_draw",14,fields))return {};
    prefix="cbuffer PainterTissueState:register(b0){"+fields+"}\n"+prefix;
    if(!replace(caster,"bool found = painter_vmi_find ( rest , ray , box , P , N , label ) ;","P=priorP;N=priorN;label=priorLabel;")||
       !replace(caster,"float box , out float3 P","float box , uint found, float3 priorP, float3 priorN, uint priorLabel, out float3 P"))return {};
    a=last.find("painter_vms_out o ;");b=last.find("o . depth = painter_vms_found_depth");
    if(a==last.npos||b==last.npos||b<=a)return {};
    b=last.find("return o ;",b);if(b==last.npos)return {};
    last=last.substr(a,b-a);
    if(!replace(last,"const float3 ray = painter_vms_ray ( I ) ;","")||
       !replace(last,"float3 P , N ; uint label ;","float3 finalP,finalN;uint finalLabel;")||
       !replace(last,"painter_box , P , N , label","painter_box , found,P,N,label,finalP,finalN,finalLabel")||
       !replace(last,"I , P , ray","I , finalP , ray"))return {};
    for(int i=0;i!=2;++i){auto at=last.find("discard ; return o ;");if(at==last.npos)return {};last.replace(at,20,"res_c=o.c;res_depth=o.depth;discard;return;");}
    if(last.find("discard ; return o ;")!=last.npos)return {};
    Recipe r;
    r.parts[0]=cache.intern(prefix+"\n"+R"VSH(export void fittedShadow(float3 painter_rest,float3 painter_K0,float3 painter_K1,float3 painter_K2,float3 painter_K3,float4 painter_clip,float painter_box,float4 hpos,uint primitive,out precise float3 ray,out precise float3 P,out precise float3 N,out uint label,out uint found){painter_vms_v2p I;I.painter_rest=painter_rest;I.painter_K0=painter_K0;I.painter_K1=painter_K1;I.painter_K2=painter_K2;I.painter_K3=painter_K3;I.painter_clip=painter_clip;I.painter_box=painter_box;I.hpos=hpos;ray=painter_vms_ray(I);P=I.painter_rest;N=-ray;label=0u;found=0u;[branch]if(I.painter_box>=-500.)found=painter_vmi_find(I.painter_rest,ray,I.painter_box,P,N,label);})VSH",flags);
    r.parts[1]=cache.intern(prefix+caster+"\n"+R"VSH(export void residualShadow(float3 painter_rest,float3 painter_K0,float3 painter_K1,float3 painter_K2,float3 painter_K3,float4 painter_clip,float painter_box,float4 hpos,uint primitive,float3 ray,float3 P,float3 N,uint label,uint found,out float4 res_c,out float res_depth){painter_vms_v2p I;I.painter_rest=painter_rest;I.painter_K0=painter_K0;I.painter_K1=painter_K1;I.painter_K2=painter_K2;I.painter_K3=painter_K3;I.painter_clip=painter_clip;I.painter_box=painter_box;I.hpos=hpos;)VSH"+last+"res_c=o.c;res_depth=o.depth;}",flags);
    return r;
}
inline bool bind(Module& m) {
    for(UINT i=0;i<m.desc.BoundResources;++i) {
        D3D11_SHADER_INPUT_BIND_DESC actual{};
        if(FAILED(m.function->GetResourceBindingDesc(i,&actual))||!actual.Name)return false;
        const Bound* slot=nullptr;
        for(const auto& s:slots)if(!strcmp(s.name,actual.Name)){slot=&s;break;}
        if(!slot||slot->type!=actual.Type||slot->count!=actual.BindCount)return false;
        HRESULT hr=E_FAIL;
        switch(slot->type) {
            case D3D_SIT_CBUFFER: hr=m.instance->BindConstantBufferByName(actual.Name,slot->slot,0);break;
            case D3D_SIT_TEXTURE: hr=m.instance->BindResourceByName(actual.Name,slot->slot,slot->count);break;
            case D3D_SIT_SAMPLER: hr=m.instance->BindSamplerByName(actual.Name,slot->slot,slot->count);break;
            default:return false;
        }
        if(FAILED(hr))return false;
    }
    return true;
}
inline bool make(Recipe& r,UINT flags,ID3DBlob** dst) {
    if(!dst || !r.ready() || flags!=D3DCOMPILE_PACK_MATRIX_ROW_MAJOR)return false;
    std::array<Module,2> mod;
    std::array<ComPtr<ID3D11LinkingNode>,2> node;
    ComPtr<ID3D11FunctionLinkingGraph> graph;
    ComPtr<ID3D11LinkingNode> input,output;
    if(FAILED(D3DCreateFunctionLinkingGraph(0,&graph)))return false;
    std::vector<D3D11_PARAMETER_DESC> pin,pout;
    for(const auto& p:inputPins){
        auto v=painter_tear_link::param(p.name,p.semantic,p.width,false,p.integer);
        if(!strcmp(p.semantic,"SV_PrimitiveID"))v.InterpolationMode=D3D_INTERPOLATION_CONSTANT;
        pin.push_back(v);
    }
    for(const auto& p:outputPins)pout.push_back(painter_tear_link::param(p.name,p.semantic,p.width,true,p.integer));
    if(FAILED(graph->SetInputSignature(pin.data(),UINT(pin.size()),&input)))return false;
    for(unsigned i=0;i<2;++i){
        if(!r.parts[i]->compile(flags) || !mod[i].open(*r.parts[i]->binary.Get())||
           mod[i].desc.FunctionParameterCount!=parameterCounts[i]||!bind(mod[i]))return false;
        if(FAILED(graph->CallFunction("",mod[i].program.Get(),names[i],&node[i])))return false;
    }
    if(FAILED(graph->SetOutputSignature(pout.data(),UINT(pout.size()),&output)))return false;
    for(const auto& w:wires){
        auto* from=w.from<0?input.Get():node[w.from].Get();
        auto* to=w.to==2?output.Get():node[w.to].Get();
        if(FAILED(graph->PassValue(from,w.output,to,w.input)))return false;
    }
    ComPtr<ID3D11ModuleInstance> entry;
    ComPtr<ID3D11Linker> linker;
    ComPtr<ID3DBlob> out;
    if(FAILED(graph->CreateModuleInstance(&entry,nullptr))||FAILED(D3DCreateLinker(&linker)))return false;
    for (auto& m:mod)if(FAILED(linker->UseLibrary(m.instance.Get())))return false;
    if(FAILED(linker->Link(entry.Get(),"main","ps_5_0",0,&out,nullptr)))return false;
    if(!painter_shadow_depth::normalize(static_cast<unsigned char*>(out->GetBufferPointer()),out->GetBufferSize(),3))return false;
    if(!painter_burn_tear_link::certifiedPayload(out->GetBufferPointer(),out->GetBufferSize(),r.certified))return false;
    *dst=out.Detach();return true;
}
}
