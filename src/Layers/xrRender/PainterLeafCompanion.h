#pragma once
#include "PainterLeafContent.h"
#include "PainterLeafViewKey.h"

namespace PainterLeafContent {
// Private data lives with the source resource, including during an evict/reload.
inline void attach(ID3D11Resource* texture, const void* bits, unsigned size) {
    if (!texture) return;
    const auto* entry=identify(bits,size);
    if (!entry) return;
    string_path path;
    if (!FS.exist(path,"$game_textures$",entry->maskName,".dds")) return;
    IReader* mask=FS.r_open(path);
    if (!mask) return;
    unsigned char actual[32];
    if (mask->length()==entry->maskBytes &&
        digest(mask->pointer(),mask->length(),actual) && !memcmp(actual,entry->mask,sizeof(actual))) {
        ID3D11ShaderResourceView* view=nullptr;
        D3DX11_IMAGE_LOAD_INFO info;
        info.BindFlags=D3D11_BIND_SHADER_RESOURCE;
        info.Usage=D3D11_USAGE_IMMUTABLE;
        info.CpuAccessFlags=0;
        info.Filter=D3DX11_FILTER_NONE;
        info.MipFilter=D3DX11_FILTER_NONE;
        if (SUCCEEDED(D3DX11CreateShaderResourceViewFromMemory(HW.pDevice,mask->pointer(),
                              mask->length(),&info,nullptr,&view,nullptr)) && view) {
            texture->SetPrivateDataInterface(woodViewKey,view);
            view->Release();
        }
    }
    FS.r_close(mask);
}
}
