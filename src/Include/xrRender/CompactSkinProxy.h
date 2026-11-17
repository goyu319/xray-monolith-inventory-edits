#pragma once
// Private generic geometry prototype. Input is authenticated exposed skin with
// one native physical owner; source classification is an explicit caller input.
// No model coordinates, model names, bone names, shader names or file I/O.
#include <vector>
#include <array>
#include <map>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
namespace compact_skin_proxy {
struct Vec { double x=0,y=0,z=0; Vec operator+(Vec b)const{return{x+b.x,y+b.y,z+b.z};} Vec operator-(Vec b)const{return{x-b.x,y-b.y,z-b.z};} Vec operator*(double s)const{return{x*s,y*s,z*s};} double dot(Vec b)const{return x*b.x+y*b.y+z*b.z;} double length2()const{return dot(*this);} };
inline Vec cross(Vec a,Vec b){return{a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline double distance(Vec a,Vec b){return std::sqrt((a-b).length2());}
struct Ball {Vec center;double radius=0,overhang=0;};
struct Face {std::array<unsigned,3> v;Vec n;double d=0;};
struct Fit {std::vector<Ball> balls;unsigned points=0,hull_vertices=0,hull_faces=0,candidates=0;double clearance=0,overhang=0,minimum_vertex_clearance=0;bool complete=false;};
inline bool hull(const std::vector<Vec>& p,std::vector<Face>& out,std::vector<unsigned>& vertices){
 if(p.size()<4||p.size()>65536)return false;
 unsigned a=0,b=0,c=0,e=0;
 for(unsigned i=1;i<p.size();++i)if(p[i].x<p[a].x)a=i;
 for(unsigned i=0;i<p.size();++i)if((p[i]-p[a]).length2()>(p[b]-p[a]).length2())b=i;
 if((p[b]-p[a]).length2()<1e-12)return false;
 for(unsigned i=0;i<p.size();++i)if(cross(p[i]-p[a],p[b]-p[a]).length2()>cross(p[c]-p[a],p[b]-p[a]).length2())c=i;
 Vec base=cross(p[b]-p[a],p[c]-p[a]);if(base.length2()<1e-18)return false;
 for(unsigned i=0;i<p.size();++i)if(std::abs(base.dot(p[i]-p[a]))>std::abs(base.dot(p[e]-p[a])))e=i;
 if(std::abs(base.dot(p[e]-p[a]))<1e-12)return false;
 const Vec inside=(p[a]+p[b]+p[c]+p[e])*.25;
 auto face=[&](unsigned x,unsigned y,unsigned z){Face f{{x,y,z},cross(p[y]-p[x],p[z]-p[x]),0};double size=std::sqrt(f.n.length2());if(size<1e-14)return;f.n=f.n*(1/size);f.d=-f.n.dot(p[x]);if(f.n.dot(inside)+f.d>0){std::swap(f.v[1],f.v[2]);f.n=f.n*(-1);f.d=-f.d;}out.push_back(f);};
 face(a,b,c);face(a,b,e);face(a,c,e);face(b,c,e);
 for(unsigned q=0;q<p.size();++q){
  std::map<std::pair<unsigned,unsigned>,std::pair<unsigned,unsigned>> edges;
  std::vector<Face> kept;kept.reserve(out.size()+8);bool outside=false;
  for(const auto& f:out){if(f.n.dot(p[q])+f.d<=1e-10){kept.push_back(f);continue;}outside=true;
   for(unsigned j=0;j<3;++j){unsigned x=f.v[j],y=f.v[(j+1)%3];auto reverse=edges.find({y,x});if(reverse!=edges.end())edges.erase(reverse);else edges.emplace(std::make_pair(x,y),std::make_pair(x,y));}}
  if(!outside)continue;out.swap(kept);for(const auto& edge:edges)face(edge.second.first,edge.second.second,q);
  if(out.size()>32768)return false;
 }
 // Explicit final half-space check; failed numeric/construction certification
 // refuses optional fitting rather than publishing a partial proxy envelope.
 for(const auto& f:out)for(const auto& q:p)if(f.n.dot(q)+f.d>5e-8)return false;
 vertices.clear();for(const auto& f:out)for(unsigned q:f.v)vertices.push_back(q);std::sort(vertices.begin(),vertices.end());vertices.erase(std::unique(vertices.begin(),vertices.end()),vertices.end());return true;
}
inline Fit fit(const std::vector<Vec>& points,double allowed_overhang=.0101,double clearance=.0021,unsigned maximum_balls=40){
 Fit result;result.points=unsigned(points.size());result.clearance=clearance;result.overhang=allowed_overhang;
 if(points.size()<4||points.size()>65536||allowed_overhang<=clearance||maximum_balls>64)return result;
 for(const auto& p:points)if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return result;
 std::vector<Face> faces;std::vector<unsigned> hv;if(!hull(points,faces,hv))return result;
 result.hull_vertices=unsigned(hv.size());result.hull_faces=unsigned(faces.size());
 Vec mean{},lo=points[0],hi=lo;for(const auto& p:points){if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return result;mean=mean+p;lo={std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z)};hi={std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z)};}mean=mean*(1./points.size());
 std::vector<Vec> centers{mean};
 // Candidate depth scales follow the authenticated source bounds, so a small
 // hand and a head do not share hardcoded metre positions or fitting radii.
 const double scale=std::max({hi.x-lo.x,hi.y-lo.y,hi.z-lo.z});
 for(unsigned q:hv){Vec inward{};for(const auto& f:faces)if(f.n.dot(points[q])+f.d>-1e-6)inward=inward-f.n;double len=std::sqrt(inward.length2());if(len<1e-12)continue;inward=inward*(1/len);for(double depth:{.06,.125,.2,.3,.4})centers.push_back(points[q]+inward*(scale*depth));}
 for(unsigned x=0;x<9;++x)for(unsigned y=0;y<13;++y)for(unsigned z=0;z<9;++z)centers.push_back({lo.x+(hi.x-lo.x)*x/8,lo.y+(hi.y-lo.y)*y/12,lo.z+(hi.z-lo.z)*z/8});
 const unsigned words=unsigned((points.size()+63)/64);std::vector<Ball> candidates;std::vector<std::vector<uint64_t>> cover;
 for(const auto& c:centers){double plane=-std::numeric_limits<double>::infinity();for(const auto& f:faces)plane=std::max(plane,f.n.dot(c)+f.d);if(plane>1e-8)continue;double r=-plane+allowed_overhang;std::vector<uint64_t> mask(words);bool any=false;for(unsigned q=0;q<points.size();++q)if(distance(c,points[q])<=r-clearance-1e-6){mask[q/64]|=uint64_t(1)<<(q%64);any=true;}if(any){candidates.push_back({c,r,r+plane});cover.push_back(std::move(mask));}}
 result.candidates=unsigned(candidates.size());std::vector<uint64_t> remaining(words,~uint64_t(0));if(points.size()%64)remaining.back()=(uint64_t(1)<<(points.size()%64))-1;
 auto pop=[](uint64_t v){unsigned n=0;while(v){v&=v-1;++n;}return n;};std::vector<unsigned> selected;
 for(unsigned step=0;step<maximum_balls;++step){unsigned best=0,score=0;for(unsigned b=0;b<candidates.size();++b){unsigned count=0;for(unsigned w=0;w<words;++w)count+=pop(cover[b][w]&remaining[w]);if(count>score){score=count;best=b;}}if(!score)break;selected.push_back(best);bool any=false;for(unsigned w=0;w<words;++w){remaining[w]&=~cover[best][w];any|=remaining[w]!=0;}if(!any){result.complete=true;break;}}
 if(!result.complete)return result;
 for(size_t index=0;index<selected.size();){bool all=true;for(unsigned w=0;w<words;++w){uint64_t combined=0;for(size_t j=0;j<selected.size();++j)if(j!=index)combined|=cover[selected[j]][w];const uint64_t required=(w+1==words&&points.size()%64)?((uint64_t(1)<<(points.size()%64))-1):~uint64_t(0);if(combined!=required){all=false;break;}}if(all)selected.erase(selected.begin()+index);else ++index;}
 result.minimum_vertex_clearance=std::numeric_limits<double>::infinity();for(unsigned b:selected)result.balls.push_back(candidates[b]);
 for(const auto& p:points){double inside=-std::numeric_limits<double>::infinity();for(const auto& b:result.balls)inside=std::max(inside,b.radius-distance(p,b.center));result.minimum_vertex_clearance=std::min(result.minimum_vertex_clearance,inside);}
 return result;
}
}
