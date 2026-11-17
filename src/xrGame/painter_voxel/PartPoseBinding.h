#pragma once
// Numeric pose ownership for separated pieces. Original vertex weights and
// anatomical rest coordinates remain intact. Each piece supplies its own
// physical anchor transforms; missing source bones follow a retained anchor
// through the relationship captured at separation, never another live object.
// CPU preparation only; a native shell/render adapter must own the callbacks.
#include "LiveDamageWorker.h"
#include <cmath>
#include <stdexcept>
namespace gfy_live { namespace part_pose {
inline void require(bool ok,const char*why){if(!ok)throw std::runtime_error(why);}
inline void validate(const Matrix&m){
 for(double n:m.row_major)require(std::isfinite(n),"part_pose_nonfinite");
 const auto&a=m.row_major;require(std::abs(a[12])<1e-12&&std::abs(a[13])<1e-12&&std::abs(a[14])<1e-12&&std::abs(a[15]-1)<1e-12,"part_pose_nonaffine");
}
inline Matrix multiply(const Matrix&a,const Matrix&b){Matrix out;
 for(unsigned r=0;r<4;++r)for(unsigned c=0;c<4;++c)for(unsigned k=0;k<4;++k)out.row_major[r*4+c]+=a.row_major[r*4+k]*b.row_major[k*4+c];return out;}
inline Matrix inverse(const Matrix&m){validate(m);const auto&a=m.row_major;
 double det=a[0]*(a[5]*a[10]-a[6]*a[9])-a[1]*(a[4]*a[10]-a[6]*a[8])+a[2]*(a[4]*a[9]-a[5]*a[8]);
 require(std::isfinite(det)&&std::abs(det)>1e-8,"part_pose_singular_anchor");Matrix out=Matrix::identity();auto&b=out.row_major;
 b[0]=(a[5]*a[10]-a[6]*a[9])/det;b[1]=(a[2]*a[9]-a[1]*a[10])/det;b[2]=(a[1]*a[6]-a[2]*a[5])/det;
 b[4]=(a[6]*a[8]-a[4]*a[10])/det;b[5]=(a[0]*a[10]-a[2]*a[8])/det;b[6]=(a[2]*a[4]-a[0]*a[6])/det;
 b[8]=(a[4]*a[9]-a[5]*a[8])/det;b[9]=(a[1]*a[8]-a[0]*a[9])/det;b[10]=(a[0]*a[5]-a[1]*a[4])/det;
 for(unsigned r=0;r<3;++r)b[r*4+3]=-(b[r*4]*a[3]+b[r*4+1]*a[7]+b[r*4+2]*a[11]);validate(out);return out;}
struct Binding {
 std::uint64_t owned{};
 std::vector<std::uint16_t> anchor;
 std::vector<Matrix> relative;
};
inline Binding capture(const std::vector<Matrix>&source,std::uint64_t owned,const std::vector<std::uint16_t>&anchors){
 require(!source.empty()&&source.size()<=64&&anchors.size()==source.size(),"part_pose_palette_count");
 require(owned&&(source.size()==64||!(owned>>source.size())),"part_pose_owned_mask");
 Binding out;out.owned=owned;out.anchor=anchors;out.relative.reserve(source.size());std::vector<Matrix>inverses(source.size());
 for(unsigned b=0;b<source.size();++b){validate(source[b]);if(owned&(1ull<<b))inverses[b]=inverse(source[b]);}
 for(unsigned b=0;b<source.size();++b){auto a=anchors[b];require(a<source.size()&&(owned&(1ull<<a)),"part_pose_external_anchor");
  require(!(owned&(1ull<<b))||a==b,"part_pose_owned_anchor_remapped");
  out.relative.push_back(a==b?Matrix::identity():multiply(inverses[a],source[b]));}
 return out;
}
inline std::vector<Matrix> evaluate(const Binding&binding,const std::vector<Matrix>&current){
 require(current.size()==binding.anchor.size()&&binding.relative.size()==current.size()&&!current.empty()&&current.size()<=64,"part_pose_evaluate_count");
 require(binding.owned&&(current.size()==64||!(binding.owned>>current.size())),"part_pose_evaluate_mask");
 std::vector<Matrix>out;out.reserve(current.size());
 // Intentionally do not read unowned matrices. Callers may poison these to
 // prove no remaining dependency on the old body's palette after separation.
 for(unsigned b=0;b<current.size();++b){auto a=binding.anchor[b];require(a<current.size()&&(binding.owned&(1ull<<a)),"part_pose_evaluate_anchor");
  validate(current[a]);validate(binding.relative[b]);out.push_back(multiply(current[a],binding.relative[b]));validate(out.back());}
 return out;
}
}}
