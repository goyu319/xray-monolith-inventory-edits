#pragma once
// The immutable, actually selected decal's texture. No synthetic stain fallback.
// Bilinear WRAP sampling matches smp_base at fixed bake LOD; screen-dependent
// anisotropy cannot be retained after a decal is reduced to rest-space cells.
#include <array>
#include <vector>
#include <cstdint>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <limits>
namespace PainterBloodTextureCodec {
using Pixel=std::array<float,4>;
enum class Format { RGBA8, BGRA8, BGRX8, BC1, BC2, BC3 };
inline float linear(float c) { return c<=0.04045f?c/12.92f:std::pow((c+0.055f)/1.055f,2.4f); }
inline std::uint16_t u16(const std::uint8_t*p){return std::uint16_t(p[0])|(std::uint16_t(p[1])<<8);}
inline std::uint32_t u32(const std::uint8_t*p){return std::uint32_t(p[0])|(std::uint32_t(p[1])<<8)|(std::uint32_t(p[2])<<16)|(std::uint32_t(p[3])<<24);}
inline Pixel rgb565(std::uint16_t c){const unsigned r=(c>>11)&31,g=(c>>5)&63,b=c&31;return {float((r<<3)|(r>>2))/255.f,float((g<<2)|(g>>4))/255.f,float((b<<3)|(b>>2))/255.f,1.f};}
inline Pixel block(const std::uint8_t*p,Format f,unsigned i){
 const auto*c=p+(f==Format::BC1?0:8);const auto a=u16(c),b=u16(c+2);Pixel palette[4]={rgb565(a),rgb565(b),{},{}},out{};
 if(a>b||f!=Format::BC1){for(unsigned k=0;k<3;++k){palette[2][k]=(2*palette[0][k]+palette[1][k])/3;palette[3][k]=(palette[0][k]+2*palette[1][k])/3;}palette[2][3]=palette[3][3]=1;}
 else {for(unsigned k=0;k<3;++k)palette[2][k]=(palette[0][k]+palette[1][k])/2;palette[2][3]=1;}
 out=palette[(u32(c+4)>>(i*2))&3];
 if(f==Format::BC2)out[3]=float((p[i/2]>>((i%2)*4))&15)/15.f;
 if(f==Format::BC3){float alpha[8]={float(p[0])/255.f,float(p[1])/255.f};
  if(p[0]>p[1])for(unsigned j=2;j<8;++j)alpha[j]=((8-j)*alpha[0]+(j-1)*alpha[1])/7;
  else {for(unsigned j=2;j<6;++j)alpha[j]=((6-j)*alpha[0]+(j-1)*alpha[1])/5;alpha[6]=0;alpha[7]=1;}
  std::uint64_t bits=0;for(unsigned k=0;k<6;++k)bits|=std::uint64_t(p[k+2])<<(8*k);out[3]=alpha[(bits>>(i*3))&7];}
 return out;
}
struct Image {
 unsigned width=0,height=0;std::vector<Pixel> pixels;
 Pixel sample(float u,float v)const {
  if(!std::isfinite(u)||!std::isfinite(v)||!width||!height||pixels.size()!=std::size_t(width)*height)throw std::runtime_error("blood_texture_sample_invalid");
  // Projection itself clips outside [0,1]; WRAP still matters at its border.
  const float x=(u-std::floor(u))*width-.5f,y=(v-std::floor(v))*height-.5f;
  const int ix=int(std::floor(x)),iy=int(std::floor(y));
  // D3D11 filtering addresses use at least eight subtexel fractional bits;
  // the deployed hardware uses nearest 1/256 fractions.
  const float tx=std::floor((x-ix)*256.f+.5f)/256.f,ty=std::floor((y-iy)*256.f+.5f)/256.f;
  auto wrap=[](int q,unsigned n){return unsigned((q%int(n)+int(n))%int(n));};
  const Pixel&a=pixels[std::size_t(wrap(iy,height))*width+wrap(ix,width)],&b=pixels[std::size_t(wrap(iy,height))*width+wrap(ix+1,width)];
  const Pixel&c=pixels[std::size_t(wrap(iy+1,height))*width+wrap(ix,width)],&d=pixels[std::size_t(wrap(iy+1,height))*width+wrap(ix+1,width)];
  Pixel out{};for(unsigned k=0;k<4;++k)out[k]=(a[k]+(b[k]-a[k])*tx)*(1-ty)+(c[k]+(d[k]-c[k])*tx)*ty;return out;
 }
};
inline Image decode(const void*data,std::size_t rowPitch,unsigned width,unsigned height,Format format,bool srgb){
 if(!data||!width||!height||width>16384||height>16384||std::size_t(width)*height>16u*1024u*1024u)throw std::runtime_error("blood_texture_dimensions");
 const bool compressed=format==Format::BC1||format==Format::BC2||format==Format::BC3;
 const unsigned blockBytes=format==Format::BC1?8:16;
 const std::size_t minRow=compressed?std::size_t((width+3)/4)*blockBytes:std::size_t(width)*4;
 if(rowPitch<minRow)throw std::runtime_error("blood_texture_row_pitch");
 Image out;out.width=width;out.height=height;out.pixels.resize(std::size_t(width)*height);
 const auto*bytes=static_cast<const std::uint8_t*>(data);
 for(unsigned y=0;y<height;++y)for(unsigned x=0;x<width;++x){Pixel c{};
  if(compressed)c=block(bytes+std::size_t(y/4)*rowPitch+std::size_t(x/4)*blockBytes,format,(y%4)*4+x%4);
  else {const auto*p=bytes+std::size_t(y)*rowPitch+x*4;const bool bgra=format!=Format::RGBA8;c={float(p[bgra?2:0])/255.f,float(p[1])/255.f,float(p[bgra?0:2])/255.f,format==Format::BGRX8?1.f:float(p[3])/255.f};}
  if(srgb)for(unsigned k=0;k<3;++k)c[k]=linear(c[k]);out.pixels[std::size_t(y)*width+x]=c;
 }
 return out;
}
}
