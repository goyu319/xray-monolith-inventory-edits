#pragma once
#include "PainterTearModules.h"
#include "PainterTearDepth.h"
#include "PainterPortableRecord.h"

// One deferred pass draws the exact ray result. The linker joins the two independently
// optimized functions before CreatePixelShader. Missing or incompatible inputs use the
// ordinary compiler through the caller's existing fallback.
namespace painter_tear_link {
using painter_tear_modules::Cache;
using painter_tear_modules::Module;
using painter_tear_modules::SharedLibrary;
using painter_tear_modules::BindingLayout;
using Microsoft::WRL::ComPtr;
struct Recipe {
    std::array<SharedLibrary,2> parts;
    bool coverage=false;
    bool ready() const {return parts[0] && parts[1];}
};
inline bool eligible(const char* name,const char* entry,const char* target,UINT flags) {
    return !strcmp(name,"painter_blood_painter_vmi_tear") && !strcmp(entry,"main") &&
           !strcmp(target,"ps_5_0") && flags==D3DCOMPILE_PACK_MATRIX_ROW_MAJOR;
}
inline std::string strippedLines(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for(size_t i=0;i<text.size();) {
        auto at=text.find_first_not_of(" \t",i);
        auto next=text.find('\n',i);
        if(next==std::string::npos)next=text.size();
        if(at>=next || text.compare(at,5,"#line")!=0)out.append(text,i,next-i+size_t(next<text.size()));
        i=next+1;
    }
    return out;
}
inline std::string compact(const std::string& text) {
    std::string out;out.reserve(text.size());
    for(char ch:text)if(static_cast<unsigned char>(ch)>32)out.push_back(ch);
    return out;
}
inline std::string membersOf(const std::string& text,const char* name) {
    auto pos=text.find(std::string("struct ")+name+" ");
    auto a=text.find('{',pos),b=text.find('}',a);
    if(a==std::string::npos || b==std::string::npos)return {};
    return compact(text.substr(a+1,b-a-1));
}
inline bool declareState(std::string& source,const char* name,int slot,std::string& fields) {
    std::string declaration=std::string("float4 ")+name+" ;";
    size_t at=source.find(declaration);
    if(at==std::string::npos || source.find(declaration,at+1)!=std::string::npos)return false;
    size_t len=declaration.size();
    if(at>=8 && source.compare(at-8,8,"uniform ")==0){at-=8;len+=8;}
    source.erase(at,len);
    fields+="float4 "+std::string(name)+":packoffset(c"+std::to_string(slot)+");";
    return true;
}
inline Recipe capture(Cache& cache,const char* name,const char* entry,const char* target,UINT flags,
                      const void* root,size_t size,const D3D_SHADER_MACRO* defines,ID3DInclude* includes) {
    Recipe recipe;
    if(!eligible(name,entry,target,flags))return recipe;
    // Multisample draw and coverage are qualified separately, even if a local override disables those macros inside its HLSL.
    for (auto d=defines; d && d->Name; ++d)
        if (!std::strcmp(d->Name,"USE_MSAA") || !std::strcmp(d->Name,"MSAA_SAMPLES")) return recipe;
    painter_portable_shader::Digest semantic{};
    if (!includes || !painter_portable_shader::key(name, entry, target, flags, root, size, defines, *includes, semantic)) return recipe;
    // This exact compiler and preprocessed input have matched every hardware render target on the three supported packs.
    if (painter_portable_shader::hex(semantic) != "cd4b7b8901d1d805d6929db66c1b5093261de582cca37cf47361aec80a9ec000") return recipe;
    auto text=strippedLines(painter_tear_modules::preprocess(root,size,defines,includes,"PAINTER_LINK_DEFERRED_TEAR"));
    const auto main=text.rfind("painter_vm_out main (");
    const auto ray=text.find("const float3 ray =",main);
    const auto point=text.find("float3 P , N ;",ray);
    const auto tail=text.find("const float3 d = P - I . painter_rest",point);
    const auto depth=text.find("o . depth = saturate",tail);
    const auto end=text.find("return o ;",depth);
    if(main==std::string::npos || ray==std::string::npos || point==std::string::npos ||
       tail==std::string::npos || depth==std::string::npos || end==std::string::npos)return {};
    const std::string expected="painter_vm_outmain(painter_vmi_v2pI){painter_vm_outo=(painter_vm_out)0;o.depth=I.painter_hpos.z;"
        "constfloat3dpx=ddx(I.painter_rest),dpy=ddy(I.painter_rest);constfloat3ray=painter_vm_rest_ray(I.painter_L0,I.painter_L1,I.painter_L2,I.position.xyz);"
        "float3P,N;uintlabel;[branch]if(!painter_vmi_find(I.painter_rest,ray,I.tcdh.x,P,N,label)){discard;returno;}";
    if(compact(text.substr(main,tail-main))!=expected || compact(text.substr(end))!="returno;}")return {};
    auto source=text.substr(0,main);
    if(membersOf(source,"painter_vm_out")!="f_defferg;floatdepth:SV_DepthGreaterEqual;")return {};
    const std::string inputs="float3painter_rest:PAINTER_REST;float3painter_L0:PAINTER_LROW0;float3painter_L1:PAINTER_LROW1;"
        "float3painter_L2:PAINTER_LROW2;float2tcdh:TEXCOORD0;float4position:TEXCOORD1;float4vel_curr:VEL_CURRENT;"
        "float4vel_prev:VEL_PREVIOUS;noperspectivecentroidfloat4painter_hpos:SV_Position;";
    if(membersOf(source,"painter_vmi_v2p")!=inputs)return {};
    const std::string baseOutputs="float4position:SV_Target0;float4C:SV_Target1;float4H:SV_Target2;float4velocity:SV_Target3;";
    auto outputs=membersOf(source,"f_deffer");
    recipe.coverage=outputs==baseOutputs+"uintmask:SV_COVERAGE;";
    if(!recipe.coverage && outputs!=baseOutputs)return {};
    std::string fields;
    if(!declareState(source,"L_hotness",11,fields) || !declareState(source,"ssfx_blood_decals",13,fields) ||
       !declareState(source,"painter_blood_clock",14,fields) || !declareState(source,"painter_blood_target",15,fields))return {};
    source="cbuffer PainterTissueState : register(b0){"+fields+"}\n"+source;
    const std::string args="float3 rest,float3 row0,float3 row1,float3 row2,float2 uv,float4 pos,float4 vel,float4 prev,float4 hpos";
    const std::string init="painter_vmi_v2p I;I.painter_rest=rest;I.painter_L0=row0;I.painter_L1=row1;I.painter_L2=row2;I.tcdh=uv;I.position=pos;I.vel_curr=vel;I.vel_prev=prev;I.painter_hpos=hpos;";
    const std::string cargo="float3 P,float3 N,uint label,uint4 state,float3 cut,float3 dpx,float3 dpy";
    const std::string geom=source+"\nexport void findTorn("+args+",out float3 P,out float3 N,out uint label,out uint4 state,out float3 cut,out float3 dpx,out float3 dpy){"+init+
        "dpx=dpy=0;"+text.substr(ray,point-ray)+
        "bool valid=painter_vmi_find(rest,ray,uv.x,P,N,label);"
        "state=uint4(painter_lod_has_crossing,painter_lod_wall_finish,painter_lod_detached_record,painter_lod_body_burst_face);cut=painter_lod_cut_position;state.w|=valid?2u:0u;}";
    const std::string color=source+"\nexport void shadeTorn("+args+","+cargo+
        ",out float4 oPos,out float4 oColor,out float4 oHeat,out float4 oVelocity,"+
        (recipe.coverage?"out uint oMask,":"")+"out float oDepth){"+init+
        "painter_lod_has_crossing=state.x!=0u;painter_lod_wall_finish=state.y!=0u;painter_lod_detached_record=state.z!=0u;painter_lod_body_burst_face=(state.w&1u)!=0u;painter_lod_cut_position=cut;"
        "[branch]if((state.w&2u)==0u){oPos=oColor=oHeat=oVelocity=0;oDepth=hpos.z;"+std::string(recipe.coverage?"oMask=0;":"")+"discard;return;}"
        "dpx=ddx(rest);dpy=ddy(rest);painter_vm_out o=(painter_vm_out)0;"+text.substr(tail,end-tail)+
        "oPos=o.g.position;oColor=o.g.C;oHeat=o.g.H;oVelocity=o.g.velocity;"+(recipe.coverage?"oMask=o.g.mask;":"")+"oDepth=o.depth;}";
    recipe.parts[0]=cache.intern(geom,flags);recipe.parts[1]=cache.intern(color,flags);
    return recipe;
}
inline D3D11_PARAMETER_DESC param(const char* name,const char* semantic,UINT width,bool outgoing,bool integer=false) {
    D3D11_PARAMETER_DESC p{};
    p.Name=name;p.SemanticName=semantic;p.Type=integer?D3D_SVT_UINT:D3D_SVT_FLOAT;
    p.Class=width==1?D3D_SVC_SCALAR:D3D_SVC_VECTOR;p.Rows=1;p.Columns=width;
    p.Flags=outgoing?D3D_PF_OUT:D3D_PF_IN;
    if(!outgoing)p.InterpolationMode=!strcmp(semantic,"SV_Position")?D3D_INTERPOLATION_LINEAR_NOPERSPECTIVE_CENTROID:D3D_INTERPOLATION_LINEAR;
    return p;
}
inline bool make(Recipe& recipe,UINT flags,ID3DBlob** bytecode) {
    if(!bytecode || !recipe.ready())return false;
    std::array<Module,2> modules;
    BindingLayout layout;
    for(int i=0;i<2;++i){
        auto& part=*recipe.parts[i];
        if(!part.compile(flags) || !modules[i].open(*part.binary.Get()))return false;
    }
    // Bind the material first so its samplers and constants retain native ownership.
    if(!layout.bind(modules[1],true) || !layout.bind(modules[0],false))return false;
    const int inputs=9,cargo=7;
    if(modules[0].desc.FunctionParameterCount!=inputs+cargo)return false;
    D3D11_PARAMETER_DESC in[]={
        param("rest","PAINTER_REST",3,false),param("row0","PAINTER_LROW0",3,false),
        param("row1","PAINTER_LROW1",3,false),param("row2","PAINTER_LROW2",3,false),
        param("uv","TEXCOORD0",2,false),param("position","TEXCOORD1",4,false),
        param("vel","VEL_CURRENT",4,false),param("prev","VEL_PREVIOUS",4,false),param("hpos","SV_Position",4,false)};
    std::vector<D3D11_PARAMETER_DESC> outputs{
        param("out_pos","SV_Target0",4,true),param("out_color","SV_Target1",4,true),
        param("out_heat","SV_Target2",4,true),param("out_velocity","SV_Target3",4,true)};
    if(recipe.coverage)outputs.push_back(param("out_mask","SV_Coverage",1,true,true));
    outputs.push_back(param("out_depth","SV_DepthGreaterEqual",1,true));
    if(modules[1].desc.FunctionParameterCount!=inputs+cargo+int(outputs.size()))return false;
    ComPtr<ID3D11FunctionLinkingGraph> graph;
    ComPtr<ID3D11LinkingNode> source,geometry,material,dest;
    if(FAILED(D3DCreateFunctionLinkingGraph(0,&graph)) ||
       FAILED(graph->SetInputSignature(in,inputs,&source)) ||
       FAILED(graph->CallFunction("",modules[0].program.Get(),"findTorn",&geometry)) ||
       FAILED(graph->CallFunction("",modules[1].program.Get(),"shadeTorn",&material)) ||
       FAILED(graph->SetOutputSignature(outputs.data(),UINT(outputs.size()),&dest)))return false;
    for(int i=0;i<inputs;++i)
        if(FAILED(graph->PassValue(source.Get(),i,geometry.Get(),i)) ||
           FAILED(graph->PassValue(source.Get(),i,material.Get(),i)))return false;
    for(int i=0;i<cargo;++i)
        if(FAILED(graph->PassValue(geometry.Get(),inputs+i,material.Get(),inputs+i)))return false;
    for(int i=0;i<int(outputs.size());++i)
        if(FAILED(graph->PassValue(material.Get(),inputs+cargo+i,dest.Get(),i)))return false;
    ComPtr<ID3D11ModuleInstance> entry;
    ComPtr<ID3D11Linker> linker;
    ComPtr<ID3DBlob> result;
    if(FAILED(graph->CreateModuleInstance(&entry,nullptr)) || FAILED(D3DCreateLinker(&linker)))return false;
    for(auto& m:modules)if(FAILED(linker->UseLibrary(m.instance.Get())))return false;
    if(FAILED(linker->Link(entry.Get(),"main","ps_5_0",0,&result,nullptr)) || !layout.accepts(*result.Get()))return false;
    if(!normalizeLinkedDepth(static_cast<unsigned char*>(result->GetBufferPointer()),result->GetBufferSize(),2))return false;
    static const painter_portable_shader::Digest expected = {0x31,0xa3,0x31,0x0c,0xd6,0x26,0xb3,0xde,0x90,0x0c,0x33,0xc2,0xff,0x47,0xaf,0x13,0x5c,0x6a,0x19,0x1f,0xf2,0xd5,0x3e,0x5a,0x84,0x26,0x08,0xb7,0x24,0xd7,0x0f,0x65};
    if (!painter_portable_shader::payload(result->GetBufferPointer(),result->GetBufferSize(),expected)) return false;
    *bytecode=result.Detach();
    return true;
}
}
