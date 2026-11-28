#pragma once
#include "PainterTearCompiler.h"
#include "PainterSurfaceSelector.h"
#include "PainterBurnTearDepth.h"

namespace painter_burn_tear_link {
using Microsoft::WRL::ComPtr;
using painter_tear_modules::Cache;
using painter_tear_modules::Module;
using painter_tear_modules::SharedLibrary;

struct Bound;
struct Recipe {
    const Bound* slotTable=nullptr;
    size_t slotCount=0;
    const char* certified=nullptr;
    std::array<SharedLibrary,7> parts{};
    bool ready() const {
        for (const auto& part : parts) if (!part) return false;
        return true;
    }
};
struct Pin {const char* name; const char* semantic; unsigned width; bool integer;};
struct Wire {int from, output, to, input;};
struct Bound {const char* name; unsigned slot, count; D3D_SHADER_INPUT_TYPE type;};
#include "PainterBurnTearRecipe.inc"
#include "PainterVolumeBindings.inc"

// Effective source, compiler, result and render-target format are qualified together.
inline const char* certifiedProfile(unsigned samples,const std::string& semantic) {
    if(const auto* result=volumeProfile(samples,semantic))return result;
    if(samples==1 && semantic=="a4bdf9ec4748e0b118f63b0e339d350a6e3fcd4438efcd78f52d458c2ddd5d92")return "e86847c1bad803f4838124efab528cae7f09e2f1dcdc74e68ea782f58ca85998"; // skinfast
    if(samples==2 && semantic=="7b0e7942c71fd6a8c1ea3b7bebddbf1fa351d548443b0244c7614ab676e947d3")return "782a9af3b8aa76bdbbb753aa56baeb1e938a8db0a0d72afd4927f9950cb943ca"; // skinfast
    if(samples==4 && semantic=="abbb8b338d5de6320ab37db4366cb4081d73580e27dc08b0c4e0aec1b99eb56b")return "782a9af3b8aa76bdbbb753aa56baeb1e938a8db0a0d72afd4927f9950cb943ca"; // skinfast
    if(samples==1 && semantic=="0ec2825b9bfaaa6c5dfc43c625ccc45313aeaf99999e32980b2dbef03841bdf1")return "d6fae75fed875916eef2fd6d2eb97c225e003d22944a33b87dd8fec646a80fed";
    if(samples==2 && semantic=="0a05cbd7f481169e24f8b32567cea46e05c6c0a33ff51c1762132f0b8d757c1e")return "d6fae75fed875916eef2fd6d2eb97c225e003d22944a33b87dd8fec646a80fed";
    if(samples==4 && semantic=="c1de2fdd23ca30b50d5cf6eb5af428b2f17af3ddb50c4ac65e687339e43a8f6d")return "ffe67470f1bdee46e9bfd2cab26255968bf9c41ed5c1072705b4603eef9a373e";
    return nullptr;
}
inline bool certifiedPayload(const void* bytes,size_t n,const char* digest) {
    return digest && painter_surface_selector::hash(bytes,n)==digest;
}
inline Recipe capture(Cache& cache,const char* name,const char* entry,const char* target,UINT flags,
                      const void* source,size_t bytes,const D3D_SHADER_MACRO* defs,ID3DInclude* includes) {
    const bool wall=name && !strcmp(name,"painter_burn_painter_blood_painter_vmi_wall");
    const bool piece=name && !strcmp(name,"painter_burn_painter_blood_painter_vmi_piece");
    if(!name||!entry||!target||!source||!bytes||!includes||
       (!wall && !piece && strcmp(name,"painter_burn_painter_blood_painter_vmi_tear"))||strcmp(entry,"main")||
       strcmp(target,"ps_5_0")||flags!=D3DCOMPILE_PACK_MATRIX_ROW_MAJOR)return {};
    auto text=painter_tear_link::strippedLines(painter_tear_modules::preprocess(source,bytes,defs,includes,"PAINTER_LINK_BURN_TEAR"));
    const auto main=text.rfind("painter_vm_out main (");
    if(main==std::string::npos || text.find("uint primitive : SV_PrimitiveID",main)==std::string::npos)return {};
    if(painter_tear_link::membersOf(text,"f_deffer")!="float4position:SV_Target0;float4C:SV_Target1;float4H:SV_Target2;float4velocity:SV_Target3;")return {};
    const bool volume=wall||piece;
    const auto materialSource=volume?text.substr(main):std::string{};
    text.resize(main);
    std::string fields;
    const char* nativeNames[]={"L_hotness","ssfx_blood_decals","painter_blood_clock","painter_blood_target","painter_burn_bake","painter_burn_meta","painter_burn_draw"};
    const int registers[]={11,13,14,15,16,17,18};
    for(unsigned i=0;i<7;++i)if(!painter_tear_link::declareState(text,nativeNames[i],registers[i],fields))return {};
    text="cbuffer PainterTissueState : register(b0){"+fields+"}\n"+text;
    const auto scan=text.find("bool pb_trace_baked (");
    const auto end=scan==std::string::npos?std::string::npos:text.find("bool pb_burn_find_shadow (",scan);
    if(end==std::string::npos || text.find("bool pb_trace_baked (",scan+1)!=std::string::npos)return {};
    Recipe r;
    if(wall){r.slotTable=wallSlots;r.slotCount=sizeof(wallSlots)/sizeof(*wallSlots);}
    if(piece){r.slotTable=pieceSlots;r.slotCount=sizeof(pieceSlots)/sizeof(*pieceSlots);}
    for (unsigned i=0;i<7;++i) {
        std::string head=text;
        if(i==4 && !volume)head.replace(scan,end-scan,std::string(bakedScan)+"\n");
        std::string stage=wrappers[i];
        // skinfast (2026-11-24): the first skin under burnt-open cloth takes the outfit's normal (painter_burn_game_floor.h:
        // pb_outfit_shaded_normal) right after the hit is known. The wall's and the piece's last part is main()'s own text;
        // the torn program's is the recipe's, so it gets the same statement - only for a source that has that function.
        if(!volume && i==6 && text.find("pb_outfit_shaded_normal (")!=std::string::npos){
            const std::string hit="const float3 d = P - I . painter_rest ;";
            const auto at=stage.find(hit);
            if(at==stage.npos)return {};
            stage.insert(at+hit.size()," N = pb_outfit_shaded_normal ( N , float3 ( I . tcdh . y , I . vel_curr . z , I . vel_prev . z ) , I . painter_L0 , I . painter_L1 , I . painter_L2 , I . position . xyz ) ;");
        }
        if(volume && i==6){
            // Carry arithmetic precision through the material store at the native function boundary.
            const std::string rayEnd="const float3 d = P -", ret="res_position=o.g.position;";
            const auto a=stage.find(rayEnd),b=stage.find(ret,a),u=materialSource.find(rayEnd);
            auto finish=materialSource.find("o . depth = saturate",u);
            const auto v=materialSource.find("return o ;",finish);
            if(a==stage.npos||b==stage.npos||u==materialSource.npos||finish==materialSource.npos||v==materialSource.npos)return {};
            auto tail=materialSource.substr(u,v-u);
            auto shade=tail.find("o . g = painter_vm_shade"),write=tail.find(';',shade);
            if(shade==tail.npos || write==tail.npos)return {};
            tail.insert(write+1,"precise float4 strictColor=o.g.C;o.g.C=strictColor;");
            stage.replace(a,b-a,tail);
        }
        head+='\n';head+=stage;
        r.parts[i]=cache.intern(std::move(head),flags);
    }
    return r;
}

inline bool bind(Module& m,const Recipe& r) {
    const Bound* bindings=r.slotTable?r.slotTable:slots;
    const size_t count=r.slotTable?r.slotCount:sizeof(slots)/sizeof(*slots);
    for(UINT i=0;i<m.desc.BoundResources;++i) {
        D3D11_SHADER_INPUT_BIND_DESC actual{};
        if(FAILED(m.function->GetResourceBindingDesc(i,&actual))||!actual.Name)return false;
        const Bound* slot=nullptr;
        for(size_t b=0;b<count;++b)if(!strcmp(bindings[b].name,actual.Name)){slot=bindings+b;break;}
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
    std::array<Module,7> mod;
    std::array<ComPtr<ID3D11LinkingNode>,7> node;
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
    for(unsigned i=0;i<7;++i){
        if(!r.parts[i]->compile(flags) || !mod[i].open(*r.parts[i]->binary.Get())||
           mod[i].desc.FunctionParameterCount!=parameterCounts[i]||!bind(mod[i],r))return false;
        if(FAILED(graph->CallFunction("",mod[i].program.Get(),names[i],&node[i])))return false;
    }
    if(FAILED(graph->SetOutputSignature(pout.data(),UINT(pout.size()),&output)))return false;
    for(const auto& w:wires){
        auto* from=w.from<0?input.Get():node[w.from].Get();
        auto* to=w.to==7?output.Get():node[w.to].Get();
        if(FAILED(graph->PassValue(from,w.output,to,w.input)))return false;
    }
    ComPtr<ID3D11ModuleInstance> entry;
    ComPtr<ID3D11Linker> linker;
    ComPtr<ID3DBlob> out;
    if(FAILED(graph->CreateModuleInstance(&entry,nullptr))||FAILED(D3DCreateLinker(&linker)))return false;
    for (auto& m:mod)if(FAILED(linker->UseLibrary(m.instance.Get())))return false;
    if(FAILED(linker->Link(entry.Get(),"main","ps_5_0",0,&out,nullptr)))return false;
    if(!painter_burn_depth::normalize(static_cast<unsigned char*>(out->GetBufferPointer()),out->GetBufferSize(),2))return false;
    if(!certifiedPayload(out->GetBufferPointer(),out->GetBufferSize(),r.certified))return false;
    *dst=out.Detach();return true;
}
}
