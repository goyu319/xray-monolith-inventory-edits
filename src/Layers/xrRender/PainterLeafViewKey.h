#pragma once
namespace PainterLeafContent {
static const GUID woodViewKey = {0xc341557e,0xff85,0x4dad,{0x90,0xac,0x4b,0x10,0x3c,0xde,0x6b,0x7a}};
inline ID3D11ShaderResourceView* viewOf(ID3D11Resource* surface) {
    ID3D11ShaderResourceView* view=nullptr;
    UINT bytes=sizeof(view);
    if (surface) surface->GetPrivateData(woodViewKey,&bytes,&view);
    return view;
}
}