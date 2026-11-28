#pragma once
#include <d3dcompiler.h>
#include <d3d11shader.h>
#include <wrl/client.h>
#include <algorithm>
#include <array>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <sstream>
#include <vector>

// Build the deferred material and its two shared surface functions with the active, edited sources.
// Keys live only for this load. Workers share immutable bytes, never the renderer or its filesystem.
namespace painter_tear_modules {
using Microsoft::WRL::ComPtr;

struct Library {
    explicit Library(std::string text):source(std::move(text)) {}
    std::string source;
    std::once_flag once;
    ComPtr<ID3DBlob> binary;
    void adopt(const std::vector<unsigned char>& bytes) {
        ComPtr<ID3DBlob> verified;
        if(FAILED(D3DCreateBlob(bytes.size(),&verified)))return;
        std::memcpy(verified->GetBufferPointer(),bytes.data(),bytes.size());
        std::call_once(once,[&]{binary=std::move(verified);});
    }
    bool compile(UINT flags) {
        std::call_once(once,[&] {
            D3DCompile(source.data(),source.size(),nullptr,nullptr,nullptr,nullptr,"lib_5_0",flags,0,binary.GetAddressOf(),nullptr);
        });
        return binary.Get()!=nullptr;
    }
};

using SharedLibrary=std::shared_ptr<Library>;
struct Cache {
    // Only the load owner modifies this map. Individual compilation is synchronized on each entry.
    std::map<std::pair<UINT,std::string>,SharedLibrary> sources;
    SharedLibrary intern(std::string text,UINT flags) {
        auto key=std::make_pair(flags,std::move(text));
        auto it=sources.find(key);
        if(it!=sources.end()) return it->second;
        auto result=std::make_shared<Library>(key.second);
        sources.emplace(std::move(key),result);
        return result;
    }
};
inline std::string preprocess(const void* source,size_t size,const D3D_SHADER_MACRO* defines,
                              ID3DInclude* includes,const char* mode)
{
    std::vector<D3D_SHADER_MACRO> options;
    for(auto d=defines;d && d->Name;++d) options.push_back(*d);
    options.push_back({mode,"1"});
    options.push_back({nullptr,nullptr});
    ComPtr<ID3DBlob> text;
    if(FAILED(D3DPreprocess(source,size,"",options.data(),includes,&text,nullptr))) return {};
    const auto* p=static_cast<const char*>(text->GetBufferPointer());
    size_t n=text->GetBufferSize();
    while(n && !p[n-1]) --n;
    return std::string(p,n);
}
struct Module {
    ComPtr<ID3D11Module> program;
    ComPtr<ID3D11ModuleInstance> instance;
    ComPtr<ID3D11LibraryReflection> reflection;
    ID3D11FunctionReflection* function=nullptr;
    D3D11_FUNCTION_DESC desc{};
    bool open(ID3DBlob& bytecode) {
        void* data=bytecode.GetBufferPointer();
        size_t n=bytecode.GetBufferSize();
        if(FAILED(D3DLoadModule(data,n,&program)) || FAILED(program->CreateInstance("",&instance)) ||
           FAILED(D3DReflectLibrary(data,n,IID_PPV_ARGS(&reflection)))) return false;
        D3D11_LIBRARY_DESC info{};
        if(FAILED(reflection->GetDesc(&info)) || info.FunctionCount!=1) return false;
        function=reflection->GetFunctionByIndex(0);
        return function && SUCCEEDED(function->GetDesc(&desc));
    }
};

// The engine binds constant buffers by reflection ordinal. Build dense slots and reject a linker
// result that enumerates them differently. The body's material owns its original sampler slots.
struct BindingLayout {
    struct Binding {UINT slot,count;D3D_SHADER_INPUT_TYPE type;};
    std::map<std::string,Binding> names;
    UINT cb=0,tex=0;
    bool bind(Module& m,bool material) {
        for(UINT i=0;i<m.desc.BoundResources;++i) {
            D3D11_SHADER_INPUT_BIND_DESC d{};
            if(FAILED(m.function->GetResourceBindingDesc(i,&d)) || !d.BindCount || !d.Name) return false;
            UINT next=0;
            auto it=names.find(d.Name);
            if(it!=names.end()) {
                if(it->second.count!=d.BindCount || it->second.type!=d.Type) return false;
                next=it->second.slot;
            } else {
                switch(d.Type){
                case D3D_SIT_SAMPLER:
                    if(!material) return false;
                    next=d.BindPoint;break;
                case D3D_SIT_CBUFFER:next=cb++;break;
                case D3D_SIT_TEXTURE:case D3D_SIT_STRUCTURED:case D3D_SIT_BYTEADDRESS:
                    next=tex;tex+=d.BindCount;break;
                default:return false;
                }
                names.emplace(d.Name,Binding{next,d.BindCount,d.Type});
            }
            HRESULT hr=E_FAIL;
            switch(d.Type){
            case D3D_SIT_CBUFFER: hr=m.instance->BindConstantBufferByName(d.Name,next,0);break;
            case D3D_SIT_SAMPLER: hr=m.instance->BindSamplerByName(d.Name,next,d.BindCount);break;
            default:hr=m.instance->BindResourceByName(d.Name,next,d.BindCount);break;
            }
            if(FAILED(hr)) return false;
        }
        return true;
    }
    bool accepts(ID3DBlob& shader) const {
        ComPtr<ID3D11ShaderReflection> result;
        if(FAILED(D3DReflect(shader.GetBufferPointer(),shader.GetBufferSize(),IID_PPV_ARGS(&result)))) return false;
        D3D11_SHADER_DESC desc{};
        if(FAILED(result->GetDesc(&desc))) return false;
        for(UINT i=0;i<desc.ConstantBuffers;++i){
            D3D11_SHADER_BUFFER_DESC cbDesc{};
            auto* buffer=result->GetConstantBufferByIndex(i);
            D3D11_SHADER_INPUT_BIND_DESC d{};
            if(!buffer || FAILED(buffer->GetDesc(&cbDesc))) return false;
            const bool bound=cbDesc.Name && SUCCEEDED(result->GetResourceBindingDescByName(cbDesc.Name,&d));
            if(!bound && cbDesc.Size==0 && cbDesc.Variables==0) continue;
            if(!bound || d.BindPoint!=i) return false;
        }
        for(UINT i=0;i<desc.BoundResources;++i){
            D3D11_SHADER_INPUT_BIND_DESC d{};
            if(FAILED(result->GetResourceBindingDesc(i,&d))) return false;
            auto it=names.find(d.Name);
            if(it==names.end()||it->second.slot!=d.BindPoint||it->second.count!=d.BindCount||it->second.type!=d.Type) return false;
        }
        return true;
    }
};

}
