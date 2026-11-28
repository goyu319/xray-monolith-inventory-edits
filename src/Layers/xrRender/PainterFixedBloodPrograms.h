#pragma once
#include <d3d11.h>

namespace PainterFixedBloodPrograms
{
enum Program {Expand, Skin, Project, Scan, Compact, ProjectSkin, ScanRender, SkinBatch, Relocate, Count};
HRESULT acquire(ID3D11Device* device, Program program, ID3D11ComputeShader** out) noexcept;
void warm(ID3D11Device* device) noexcept;
void prepareTextureReaders(ID3D11Device* device) noexcept;
void releaseTextureReaders() noexcept;
unsigned directObjectCount() noexcept;
void clear() noexcept;
}
