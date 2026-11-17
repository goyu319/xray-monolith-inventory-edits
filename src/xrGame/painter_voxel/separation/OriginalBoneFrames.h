#pragma once
// Source- and fitted-rig-hashed neutral GFY original-bone frames. A captured
// animation/ragdoll anchor_to_world is still required at separation commit.
#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <vector>
#include "PieceCollider.h"
namespace gfy_piece_collider { namespace original_frames {
struct BoneFrame {
 std::int16_t physical_element{-1};
 Matrix rest_to_anchor,anchor_to_rest;
};
struct Frames {
 std::array<std::uint8_t,32> source_sha256{},fit_sha256{};
 std::vector<BoneFrame> bones;
};
inline Frames decode(const std::uint8_t*data,std::size_t length,
                     const std::array<std::uint8_t,32>&source,
                     const std::array<std::uint8_t,32>&fit){
 require(data&&length>=76,"bone_frames_short");std::size_t cursor=0;
 auto read=[&](auto&v){require(sizeof(v)<=length-cursor,"bone_frames_truncated");
                      std::memcpy(&v,data+cursor,sizeof(v));cursor+=sizeof(v);};
 std::array<char,8>magic{};read(magic);
 require(std::memcmp(magic.data(),"PHYBIND1",8)==0,"bone_frames_magic");
 Frames out;read(out.source_sha256);read(out.fit_sha256);
 require(out.source_sha256==source&&out.fit_sha256==fit,"bone_frames_source_identity");
 std::uint32_t count{};read(count);require(count>0&&count<=256,"bone_frames_count");
 require(length==76+std::uint64_t(count)*(2+2*16*8),"bone_frames_size");
 out.bones.resize(count);
 for(auto&bone:out.bones){read(bone.physical_element);read(bone.rest_to_anchor.row_major);
  read(bone.anchor_to_rest.row_major);
  require(bone.physical_element>=-1&&bone.physical_element<int(count),"bone_frames_element");
  validateRigid(bone.rest_to_anchor);validateRigid(bone.anchor_to_rest);
  const auto&a=bone.rest_to_anchor.row_major,&b=bone.anchor_to_rest.row_major;
  for(int row=0;row<4;++row)for(int col=0;col<4;++col){double value=0;
   for(int k=0;k<4;++k)value+=a[row*4+k]*b[k*4+col];
   require(std::abs(value-(row==col?1.:0.))<1e-5,"bone_frames_inverse");}}
 return out;
}
inline Frames decode(const std::vector<std::uint8_t>&bytes,
                     const std::array<std::uint8_t,32>&source,
                     const std::array<std::uint8_t,32>&fit){
 return decode(bytes.data(),bytes.size(),source,fit);
}
inline Frames load(const char*path,const std::array<std::uint8_t,32>&source,
                   const std::array<std::uint8_t,32>&fit){
 std::ifstream stream(path,std::ios::binary|std::ios::ate);
 require(bool(stream),"bone_frames_open");const auto length=stream.tellg();
 require(length>=0&&length<=std::streamoff(76+256*(2+2*16*8)),"bone_frames_file_budget");
 stream.seekg(0);std::vector<std::uint8_t> bytes(std::size_t(length),std::uint8_t{});
 stream.read(reinterpret_cast<char*>(bytes.data()),std::streamsize(bytes.size()));
 require(bool(stream),"bone_frames_read");return decode(bytes,source,fit);
}
}}
