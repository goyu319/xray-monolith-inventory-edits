#pragma once
#include <array>
#include <cstdint>
#include <vector>
#include <string>
#include <cstring>
#include <cstddef>
#include <cmath>
// Immutable original renderer source. Clones share this object; runtime wound
// publications, donor visuals and model-pool generations never mutate it.
namespace painter_skin_proxy {
enum SurfaceKind : std::uint8_t {skin=1,solid_vest=2,solid_shoe=3};
struct Vertex {std::array<float,3> position{};std::array<std::uint16_t,4> bones{};std::array<float,4> weights{};std::array<float,2> uv{};std::uint8_t surface_kind=skin;};
struct RigBone {std::array<float,16> model_to_bone{};std::array<std::uint8_t,112> shape{};std::uint32_t joint_type=0;std::uint16_t parent=65535,owner=65535;};
struct ModelSphere {std::array<double,3> center{};double radius=0;};
struct ModelBox {std::array<double,3> center{},halfsize{};std::array<double,9> axes{1,0,0,0,1,0,0,0,1};};
struct Triangle {std::uint16_t child=65535;std::uint32_t face=0;std::array<std::uint32_t,3> vertices{};};
struct PreparedGroup {std::uint16_t anchor=65535;std::vector<ModelSphere> spheres;std::vector<ModelBox> boxes;std::vector<std::uint32_t> source_vertices,source_triangles;unsigned source_points=0,required_points=0,uncovered_points=0;std::uint8_t surface_kind=0;bool complete=false;};
struct Source {bool complete_triangles=true;std::vector<Triangle> triangles;std::vector<std::uint16_t> opaque_children;std::vector<Vertex> vertices;std::string authored_materials;std::vector<RigBone> rig;std::vector<PreparedGroup> prepared;double prepare_ms=0;};
inline bool admitted_material(std::string texture){
 for(char& c:texture){if(c=='/')c='\\';if(c>='A'&&c<='Z')c+=32;}
 // GLOBAL declarations from reviewed authored skin atlases. This is deliberately
 // exact: face-mask, balaclava, hat, helmet and unknown atlases stay unchanged.
 return texture=="act\\newloner\\hhl_m_latino01_007_light_co"||texture=="act\\act_face_22"||texture=="act\\newloner\\rus_vest_dark"||texture=="act\\newloner\\joggingshoes_black_co";
}
inline std::uint8_t material_kind(std::string texture){
 for(char& c:texture){if(c=='/')c='\\';if(c>='A'&&c<='Z')c+=32;}
 if(texture=="act\\newloner\\rus_vest_dark")return solid_vest;
 if(texture=="act\\newloner\\joggingshoes_black_co")return solid_shoe;
 return admitted_material(texture)?skin:0;
}
// Shared by the actual hardware/software model loader and native contract
// witness. Reads the AUTHORED packed OGF bytes before hardware packing/freeing.
inline bool append_raw(Source& out,unsigned links,const void* raw,std::size_t count,std::size_t available,std::uint8_t kind=0){
 if(!kind)kind=material_kind(out.authored_materials);if(!kind)return false;
 const std::size_t stride=links==1?60:links==2?64:links==3?70:links==4?76:0;
 if(!stride||!raw||count>16384||out.vertices.size()+count>16384||count*stride>available)return false;
 auto f32=[](const unsigned char* p){float x;std::memcpy(&x,p,4);return x;};
 auto u16=[](const unsigned char* p){std::uint16_t x;std::memcpy(&x,p,2);return x;};
 for(std::size_t i=0;i<count;++i){const auto* p=static_cast<const unsigned char*>(raw)+i*stride;Vertex v;v.surface_kind=kind;const std::size_t offset=links==1?0:links*2;for(unsigned a=0;a<3;++a)v.position[a]=f32(p+offset+4*a);
  if(links==1){std::uint32_t bone;std::memcpy(&bone,p+56,4);if(bone>=65536)return false;v.bones[0]=std::uint16_t(bone);v.weights[0]=1;}
  else{for(unsigned j=0;j<links;++j)v.bones[j]=u16(p+2*j);if(links==2){v.weights[1]=f32(p+52);v.weights[0]=1-v.weights[1];}else{for(unsigned j=0;j<links-1;++j)v.weights[j]=f32(p+offset+48+4*j);v.weights[links-1]=links==3?1-v.weights[0]-v.weights[1]:1-v.weights[0]-v.weights[1]-v.weights[2];}}
  const std::size_t uv=links==1?48:offset+48+4*(links-1);for(unsigned a=0;a<2;++a){v.uv[a]=f32(p+uv+4*a);if(!std::isfinite(v.uv[a]))return false;}
  for(float p:v.position)if(!std::isfinite(p))return false;
  out.vertices.push_back(v);
 }
 return true;
}
}

