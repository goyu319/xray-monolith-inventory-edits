#pragma once
// Stock R4 four-link layout: three UNORM8 coefficients and a signed implicit
// fourth. This adds no shader, vertex-buffer format, or CPU skinning renderer.
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
namespace gfy_signed_hw {
inline unsigned quantize(float w){return unsigned(std::clamp(int(std::floor(w*255.f+.5f)),0,255));}
inline float decoded(unsigned q){return float(q)/255.f;}
template<class W>inline float residual(const W&w){return ((1.f-w[0])-w[1])-w[2];}
template<class W>inline bool valid4(const W&w,bool grid=true){
 for(unsigned k=0;k<4;++k)if(!std::isfinite(w[k]))return false;
 for(unsigned k=0;k<3;++k)if(w[k]<0.f||w[k]>1.f||(grid&&w[k]!=decoded(quantize(w[k]))))return false;
 return w[3]==residual(w)&&w[3]>=-2.f&&w[3]<=1.f;
}
template<class W>inline bool negative(const W&w){for(const auto&x:w)if(x<0)return true;return false;}
template<class W>inline bool dependency(const W&w,unsigned k,double floor){return w[k]>floor||w[k]<0||(w[k]!=0&&valid4(w,true));}
template<class B,class W,class M>inline bool remap_hardware(B&bones,const W&w,const M&map){if(!valid4(w,true))return false;for(auto&b:bones){if(b>=map.size())throw std::runtime_error("signed_remap_bone_range");b=map[b];}return true;}
struct Error {double l1{},negative_discarded{},positive_discarded{},sum_error{};bool approximated{};};
template<class B,class W>inline std::array<double,64> full(const B&b,const W&w){
 std::array<double,64> result{};for(unsigned k=0;k<4;++k){if(b[k]>=64)throw std::runtime_error("signed_bone_range");result[b[k]]+=w[k];}return result;
}
template<class B,class W>inline Error difference(const std::array<double,64>&original,const B&b,const W&w){
 const auto actual=full(b,w);Error e;double sum=0;
 for(unsigned k=0;k<64;++k){e.l1+=std::abs(actual[k]-original[k]);sum+=actual[k];
  e.negative_discarded+=std::max(0.,-original[k])-std::min(std::max(0.,-original[k]),std::max(0.,-actual[k]));
  e.positive_discarded+=std::max(0.,original[k])-std::min(std::max(0.,original[k]),std::max(0.,actual[k]));}
 e.sum_error=std::abs(sum-1);return e;
}
// Called ONLY for signed vectors. Representable <=4-link values retain their
// signed dependence. Nonrepresentable generated cut values use the existing
// strongest-four-positive reduction; its signed loss is measured explicitly.
// Authored corners bypass this function and retain original slots/UNORM bytes.
inline Error pack(const std::array<double,64>&input,std::array<std::uint16_t,4>&bones,std::array<float,4>&weights){
 unsigned negatives=0,negativeBone=0;double sum=0;
 for(unsigned b=0;b<64;++b){if(!std::isfinite(input[b])||input[b]<-2.000001||input[b]>3.000001)throw std::runtime_error("signed_skin_range");sum+=input[b];if(input[b]<0){++negatives;negativeBone=b;}}
 if(!negatives||std::abs(sum-1)>1e-6)throw std::runtime_error("signed_skin_sum");
 std::array<unsigned,3> explicitBones{};std::array<double,3> explicitWeights{};unsigned slots=0;bool fits=negatives==1;
 for(unsigned b=0;b<64&&fits;++b){double left=input[b];while(left>1e-12){if(slots==3){fits=false;break;}
   const double part=std::min(1.,left);explicitBones[slots]=b;explicitWeights[slots]=part;++slots;left-=part;}}
 bool approximate=!fits;
 if(fits){bones.fill(std::uint16_t(negativeBone));weights.fill(0);
  for(unsigned k=0;k<slots;++k){bones[k]=std::uint16_t(explicitBones[k]);weights[k]=decoded(quantize(float(explicitWeights[k])));}
  weights[3]=residual(weights);
 }else{
  std::array<unsigned,64> order{};unsigned count=0;for(unsigned b=0;b<64;++b)if(input[b]>1e-12)order[count++]=b;
  std::sort(order.begin(),order.begin()+count,[&](unsigned a,unsigned b){return input[a]==input[b]?a<b:input[a]>input[b];});count=std::min(count,4u);
  if(!count)throw std::runtime_error("signed_reduction_empty");
  std::sort(order.begin(),order.begin()+count);double retained=0;unsigned largest=0;
  for(unsigned k=0;k<count;++k){retained+=input[order[k]];if(input[order[k]]>input[order[largest]])largest=k;}
  bones.fill(std::uint16_t(order[largest]));weights.fill(0);unsigned next=0;
  for(unsigned k=0;k<count;++k)if(k!=largest){bones[next]=std::uint16_t(order[k]);weights[next]=decoded(quantize(float(input[order[k]]/retained)));++next;}
  weights[3]=residual(weights);
 }
 if(!valid4(weights,true))throw std::runtime_error("signed_4w_layout");
 auto e=difference(input,bones,weights);e.approximated=approximate;return e;
}
} // namespace gfy_signed_hw
