#pragma once
// Uses the engine's existing miniz implementation. No external DLL/runtime.
#ifndef MINIZ_NO_ZLIB_COMPATIBLE_NAMES
#define MINIZ_NO_ZLIB_COMPATIBLE_NAMES
#endif
#include "../../3rd party/optick-git/src/optick_miniz.h"
#include <cstdint>
#include <stdexcept>
#include <vector>
namespace gfy_asset_codec {
inline std::vector<std::uint8_t> gunzip(const std::vector<std::uint8_t>& bytes){
 auto require=[](bool ok,const char*why){if(!ok)throw std::runtime_error(why);};
 require(bytes.size()>=18&&bytes[0]==31&&bytes[1]==139&&bytes[2]==8,"asset_gzip_header");
 const unsigned flags=bytes[3];require(!(flags&0xe0),"asset_gzip_flags");
 const size_t end=bytes.size()-8;size_t p=10;
 auto word=[&](size_t n){return std::uint32_t(bytes[n])|(std::uint32_t(bytes[n+1])<<8)|(std::uint32_t(bytes[n+2])<<16)|(std::uint32_t(bytes[n+3])<<24);};
 if(flags&4){require(p+2<=end,"asset_gzip_extra");unsigned n=bytes[p]|(unsigned(bytes[p+1])<<8);p+=2;require(n<=end-p,"asset_gzip_extra");p+=n;}
 for(unsigned flag:{8u,16u})if(flags&flag){while(p<end&&bytes[p])++p;require(p<end,"asset_gzip_string");++p;}
 if(flags&2){require(p+2<=end,"asset_gzip_header_crc");unsigned actual=mz_crc32(MZ_CRC32_INIT,bytes.data(),p)&65535u;require(actual==(bytes[p]|unsigned(bytes[p+1])<<8),"asset_gzip_header_crc");p+=2;}
 const auto size=word(end+4);require(p<end&&size>0&&size<=256u*1024u*1024u,"asset_gzip_size");
 std::vector<std::uint8_t> out(size);tinfl_decompressor decoder;tinfl_init(&decoder);
 size_t input=end-p,output=out.size();auto status=tinfl_decompress(&decoder,bytes.data()+p,&input,out.data(),out.data(),&output,TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
 require(status==TINFL_STATUS_DONE&&input==end-p&&output==out.size(),"asset_gzip_deflate");
 require(mz_crc32(MZ_CRC32_INIT,out.data(),out.size())==word(end),"asset_gzip_crc");return out;
}
}
