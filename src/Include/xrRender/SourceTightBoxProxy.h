#pragma once
// Optional generic source geometry. The caller authenticates all points of one
// native physical owner and the original-solid-covered mask. No engine types,
// model identifiers, hardcoded positions, I/O, mass or collision policy.
#include "CompactSkinProxy.h"
#include <numeric>
#include <map>
#include <chrono>
namespace source_tight_box_proxy {
using compact_skin_proxy::Vec;
using compact_skin_proxy::Face;
using compact_skin_proxy::cross;
struct Box {Vec center,half;std::array<Vec,3> axes;};
struct Fit {
 std::vector<Box> boxes;
 unsigned points=0,required=0,hull_faces=0,frames=0,candidates=0,uncovered=0;
 double minimum_clearance=0,maximum_inner_plane_error=0,maximum_outer_offset=0;
 double hull_ms=0,generation_ms=0,selection_ms=0;
 std::vector<unsigned> uncovered_points;
 bool complete=false,budget_exhausted=false,certified=false;
};
inline double coord(Vec p,unsigned a){return a==0?p.x:a==1?p.y:p.z;}
inline Vec unit(Vec p){const double n=std::sqrt(p.length2());return n>1e-14?p*(1./n):Vec{};}
inline unsigned pop(std::uint64_t v){unsigned n=0;while(v){v&=v-1;++n;}return n;}
inline Fit fit(const std::vector<Vec>& points,const std::vector<unsigned char>& required,
               unsigned maximum_boxes=40,double allowed_overhang=.0101,double clearance=.0021,bool allow_partial=false){
 Fit out;out.points=unsigned(points.size());
 const auto begun=std::chrono::steady_clock::now();
 if(points.size()<4||points.size()>1024||required.size()!=points.size()||maximum_boxes==0||maximum_boxes>64||
    !std::isfinite(allowed_overhang)||!std::isfinite(clearance)||clearance<0||allowed_overhang<=clearance)return out;
 for(const auto& p:points)if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return out;
 for(auto q:required)out.required+=q?1:0;
 if(!out.required){out.complete=true;return out;}
 std::vector<Face> hull;std::vector<unsigned> vertices;
 if(!compact_skin_proxy::hull(points,hull,vertices))return out;
 if(hull.size()>512)return out;
 out.hull_faces=unsigned(hull.size());
 out.hull_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begun).count();
 const auto generation_begun=std::chrono::steady_clock::now();
 // Keep 0.1 mm of the declared support budget for native float conversion.
 // Moving any point of an expanded OBB back to its inner OBB costs at most
 // norm(growth). All inner corners pass every full-source hull plane.
 const double safety=1e-7,grow=(allowed_overhang-.0001)/std::sqrt(3.);
 if(grow<.005||grow<=clearance+2e-6)return out;
 Vec mean{};for(auto p:points)mean=mean+p;mean=mean*(1./points.size());
 std::vector<std::array<Vec,3>> frames{{Vec{1,0,0},Vec{0,1,0},Vec{0,0,1}}};
 std::vector<unsigned> face_order(hull.size());std::iota(face_order.begin(),face_order.end(),0);
 std::stable_sort(face_order.begin(),face_order.end(),[&](unsigned a,unsigned b){
  const auto& x=hull[a].v;const auto& y=hull[b].v;
  return cross(points[x[1]]-points[x[0]],points[x[2]]-points[x[0]]).length2()>cross(points[y[1]]-points[y[0]],points[y[2]]-points[y[0]]).length2();});
 for(unsigned id:face_order){const auto& f=hull[id];Vec u{};
  for(unsigned a=0;a<3;++a){const auto e=points[f.v[(a+1)%3]]-points[f.v[a]];if(e.length2()>u.length2())u=e;}
  u=unit(u);const Vec w=f.n,v=unit(cross(w,u));if(v.length2()<.99)continue;
  std::array<Vec,3> frame{u,v,w};bool duplicate=false;
  for(const auto& other:frames){bool same=true;for(unsigned a=0;a<3;++a){double nearest=0;for(unsigned b=0;b<3;++b)nearest=std::max(nearest,std::abs(frame[a].dot(other[b])));same&=nearest>.9995;}if(same){duplicate=true;break;}}
  if(!duplicate)frames.push_back(frame);if(frames.size()>=128)break;
 }
 // Eight deterministic, well-separated source frames, rather than eight
 // almost coplanar triangles of a sole. Axis signs/permutations are identical
 // OBB orientations and do not buy an additional candidate frame.
 const unsigned permutations[6][3]={{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
 auto separation=[&](const std::array<Vec,3>& a,const std::array<Vec,3>& b){double best=0;for(const auto& p:permutations){double score=0;for(unsigned j=0;j<3;++j)score+=std::abs(a[j].dot(b[p[j]]));best=std::max(best,score);}return 1.-best/3.;};
 std::vector<std::array<Vec,3>> chosen_frames{frames.front()};std::vector<unsigned char> used(frames.size());used[0]=1;
 while(chosen_frames.size()<8&&chosen_frames.size()<frames.size()){unsigned best=0;double score=-1.;for(unsigned i=1;i<frames.size();++i)if(!used[i]){double nearest=1.;for(const auto& f:chosen_frames)nearest=std::min(nearest,separation(frames[i],f));if(nearest>score){score=nearest;best=i;}}used[best]=1;chosen_frames.push_back(frames[best]);}
 frames.swap(chosen_frames);
 const unsigned words=unsigned((points.size()+63)/64);
 std::vector<std::uint64_t> need(words);for(unsigned i=0;i<points.size();++i)if(required[i])need[i/64]|=std::uint64_t(1)<<(i%64);
 struct Candidate {Box box;Vec growth;std::vector<std::uint64_t> cover;};
 std::vector<Candidate> candidates;std::map<std::vector<std::uint64_t>,unsigned> unique;
 std::vector<std::vector<unsigned>> neighbors(points.size());
 for(unsigned q=0;q<points.size();++q)if(required[q]){auto& order=neighbors[q];for(unsigned j=0;j<points.size();++j)if(required[j])order.push_back(j);
  const auto less=[&](unsigned a,unsigned b){const double x=(points[a]-points[q]).length2(),y=(points[b]-points[q]).length2();return x<y||(x==y&&a<b);};
  const auto limit=std::min<std::size_t>(64,order.size());std::partial_sort(order.begin(),order.begin()+limit,order.end(),less);order.resize(limit);}
 constexpr unsigned candidate_budget=32768;
 auto candidate=[&](const std::array<Vec,3>& axes,const std::vector<Vec>& local,const std::vector<Face>& planes,Vec c,Vec h,Vec growth){
  for(const auto& f:planes){const double err=f.n.dot(c)+f.d+std::abs(f.n.x)*h.x+std::abs(f.n.y)*h.y+std::abs(f.n.z)*h.z;if(err>1e-9)return;}
  const Vec expanded=h+growth;std::vector<std::uint64_t> mask(words);unsigned count=0;
  for(unsigned q=0;q<local.size();++q)if(required[q]){const Vec d=local[q]-c;if(std::abs(d.x)<=expanded.x-clearance-2e-6&&std::abs(d.y)<=expanded.y-clearance-2e-6&&std::abs(d.z)<=expanded.z-clearance-2e-6){mask[q/64]|=std::uint64_t(1)<<(q%64);++count;}}
  if(!count||unique.count(mask))return;
  if(candidates.size()>=candidate_budget){out.budget_exhausted=true;return;}
  Box b;b.center=mean+axes[0]*c.x+axes[1]*c.y+axes[2]*c.z;b.half=expanded;b.axes=axes;
  unique.emplace(mask,unsigned(candidates.size()));candidates.push_back({b,growth,std::move(mask)});
 };
 for(const auto& axes:frames){++out.frames;
  std::vector<Vec> local;local.reserve(points.size());Vec lo{},hi{};
  for(auto p:points){p=p-mean;Vec q{p.dot(axes[0]),p.dot(axes[1]),p.dot(axes[2])};if(local.empty())lo=hi=q;else{lo={std::min(lo.x,q.x),std::min(lo.y,q.y),std::min(lo.z,q.z)};hi={std::max(hi.x,q.x),std::max(hi.y,q.y),std::max(hi.z,q.z)};}local.push_back(q);}
  std::vector<Face> planes;for(const auto& f:hull){Face p=f;p.n={f.n.dot(axes[0]),f.n.dot(axes[1]),f.n.dot(axes[2])};p.d=f.n.dot(mean)+f.d;planes.push_back(p);}
  // The smallest inner interval needed to cover a source patch with the
  // declared outer clearance is its bbox shrunk by (growth-clearance).
  // Collapsed intervals may move toward the hull interior. Every candidate
  // still passes all inner-corner planes; source-neighbor guesses confer no
  // geometry eligibility by themselves.
  // Exactly two source-derived expansion choices. The second spends more of
  // the same Euclidean support budget along the source-face normal. Direct
  // atomic CBoxGeom allocation preserves these positive small halfsizes.
  for(const Vec growth:{Vec{grow,grow,grow},Vec{.003,.003,std::sqrt((allowed_overhang-.0001)*(allowed_overhang-.0001)-.000018)}}){
  const Vec band=growth-Vec{clearance+5e-5,clearance+5e-5,clearance+5e-5};
  auto patch=[&](Vec a,Vec b,Vec& center,Vec& half){
   for(unsigned axis=0;axis<3;++axis){const double l=coord(a,axis)+coord(band,axis),u=coord(b,axis)-coord(band,axis);
    const double c=l<=u?(l+u)*.5:std::max(u,std::min(0.,l));
    (&center.x)[axis]=c;(&half.x)[axis]=l<=u?(u-l)*.5:0.;}
   for(const auto& f:planes)if(f.n.dot(center)+f.d+std::abs(f.n.x)*half.x+std::abs(f.n.y)*half.y+std::abs(f.n.z)*half.z>-safety)return false;
   return true;
  };
  for(unsigned q=0;q<points.size();++q)if(required[q]){Vec a=local[q],b=a,c{},h{};unsigned accepted=0;
   for(unsigned j:neighbors[q]){const Vec p=local[j];Vec aa{std::min(a.x,p.x),std::min(a.y,p.y),std::min(a.z,p.z)},bb{std::max(b.x,p.x),std::max(b.y,p.y),std::max(b.z,p.z)},cc{},hh{};
    if(!patch(aa,bb,cc,hh))continue;a=aa;b=bb;c=cc;h=hh;++accepted;
    if((accepted&(accepted-1))==0)candidate(axes,local,planes,c,h,growth);
   }
   if(accepted)candidate(axes,local,planes,c,h,growth);
  }
  }
  if(out.budget_exhausted)break;
 }
 out.candidates=unsigned(candidates.size());
 out.generation_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-generation_begun).count();
 const auto selection_begun=std::chrono::steady_clock::now();
 std::vector<std::vector<unsigned>> inverted(points.size());
 for(unsigned q=0;q<points.size();++q)if(required[q])for(unsigned i=0;i<candidates.size();++i)if(candidates[i].cover[q/64]&(std::uint64_t(1)<<(q%64)))inverted[q].push_back(i);
 auto remaining=need;std::vector<unsigned> selected;unsigned best_uncovered=out.required;
 // Same certified candidates, several deterministic rarity weights. No extra
 // geometry/clearance/shape budget. Incremental inverted scores touch each
 // covered incidence once rather than rescanning vertices at every choice.
 for(double exponent:{0.,.5,1.,1.5,2.}){auto left=need;std::vector<unsigned> chosen;std::vector<double> scores(candidates.size()),weights(points.size());
  for(unsigned q=0;q<points.size();++q)if(required[q]&&!inverted[q].empty()){weights[q]=1./std::pow(double(inverted[q].size()),exponent);for(auto i:inverted[q])scores[i]+=weights[q];}
  for(unsigned step=0;step<maximum_boxes;++step){unsigned best=0;double score=0;
   for(unsigned i=0;i<scores.size();++i)if(scores[i]>score+1e-12){score=scores[i];best=i;}if(score<=1e-12)break;chosen.push_back(best);
   for(unsigned q=0;q<points.size();++q)if((left[q/64]&candidates[best].cover[q/64])&(std::uint64_t(1)<<(q%64))){left[q/64]&=~(std::uint64_t(1)<<(q%64));for(auto i:inverted[q])scores[i]-=weights[q];}
   bool any=false;for(auto w:left)any|=w!=0;if(!any)break;
  }
  unsigned missing=0;for(auto w:left)missing+=pop(w);
  if(missing<best_uncovered||(missing==0&&chosen.size()<selected.size())){best_uncovered=missing;remaining=left;selected=std::move(chosen);}
 }
 out.complete=best_uncovered==0;
 for(auto w:remaining)out.uncovered+=pop(w);
 for(unsigned q=0;q<points.size();++q)if(remaining[q/64]&(std::uint64_t(1)<<(q%64)))out.uncovered_points.push_back(q);
 out.selection_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-selection_begun).count();
 if(!out.complete&&!allow_partial)return out; // Partial groups require explicit caller admission; each emitted box remains source-certified.
 for(std::size_t q=0;q<selected.size();){bool all=true;for(unsigned w=0;w<words;++w){std::uint64_t mask=0;for(std::size_t j=0;j<selected.size();++j)if(j!=q)mask|=candidates[selected[j]].cover[w];if((mask&need[w])!=need[w]){all=false;break;}}if(all)selected.erase(selected.begin()+q);else++q;}
 for(auto q:selected)out.boxes.push_back(candidates[q].box);
 out.minimum_clearance=1e100;out.maximum_inner_plane_error=-1e100;out.maximum_outer_offset=0;
 for(unsigned q=0;q<points.size();++q)if(required[q]&&!(remaining[q/64]&(std::uint64_t(1)<<(q%64)))){double best=-1e100;for(const auto& b:out.boxes){const Vec d=points[q]-b.center;double inside=1e100;for(unsigned a=0;a<3;++a)inside=std::min(inside,coord(b.half,a)-std::abs(d.dot(b.axes[a])));best=std::max(best,inside);}out.minimum_clearance=std::min(out.minimum_clearance,best);}
 for(auto q:selected){const auto& b=candidates[q].box;const auto growth=candidates[q].growth;out.maximum_outer_offset=std::max(out.maximum_outer_offset,std::sqrt(growth.length2()));const Vec inner=b.half-growth;for(unsigned corner=0;corner<8;++corner){Vec p=b.center;for(unsigned a=0;a<3;++a)p=p+b.axes[a]*(coord(inner,a)*((corner&(1u<<a))?1.:-1.));for(const auto& f:hull)out.maximum_inner_plane_error=std::max(out.maximum_inner_plane_error,f.n.dot(p)+f.d);}}
 out.certified=!out.boxes.empty()&&out.minimum_clearance>=clearance&&out.maximum_inner_plane_error<=1e-9&&out.maximum_outer_offset<=allowed_overhang;if(!out.certified){out.complete=false;out.boxes.clear();}
 return out;
}
}
