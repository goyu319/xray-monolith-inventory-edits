#pragma once
#ifdef USE_DX11
namespace painter_static_blood_outputs {
// Store the compiled program's outputs on its own device object. Cached bytecode and source compiles share this path.
inline const GUID& key() {
    static const GUID id={0x8c27bd13,0x63e7,0x4e5c,{0x9e,0xac,0x25,0x07,0x4d,0x18,0xe1,0x3b}};
    return id;
}
inline void remember(ID3D11PixelShader* ps,ID3D11ShaderReflection* reflection) {
    D3D11_SHADER_DESC desc={};
    if(!ps||FAILED(reflection->GetDesc(&desc)))return;
    unsigned mask=0;
    for(unsigned i=0;i<desc.OutputParameters;++i){
        D3D11_SIGNATURE_PARAMETER_DESC param={};
        if(FAILED(reflection->GetOutputParameterDesc(i,&param)))return;
        if(param.SystemValueType==D3D_NAME_TARGET && param.SemanticIndex<8)mask|=1u<<param.SemanticIndex;
    }
    ps->SetPrivateData(key(),sizeof(mask),&mask);
}
inline unsigned mask(ID3D11PixelShader* ps){
    unsigned result=0;UINT size=sizeof(result);
    return ps && SUCCEEDED(ps->GetPrivateData(key(),&size,&result)) ? result : 0;
}
}
#endif