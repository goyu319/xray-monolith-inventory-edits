#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace painter_fire {
using V=std::array<float,3>;
using H=std::uint16_t;
constexpr int X=16,Y=32,Z=8,Cells=X*Y*Z,Words=2+Cells/8,Slots=32;
constexpr std::uint32_t Magic=0x46495245u;
inline V add(V a,V b){return {a[0]+b[0],a[1]+b[1],a[2]+b[2]};}
inline V sub(V a,V b){return {a[0]-b[0],a[1]-b[1],a[2]-b[2]};}
inline float dot(V a,V b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
inline V mul(V a,float k){return {a[0]*k,a[1]*k,a[2]*k};}
inline V unit(V a){auto q=dot(a,a);return q>1e-12f?mul(a,1.f/std::sqrt(q)):V{};}
inline V cross(V a,V b){return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};}
inline std::uint32_t noise(std::uint32_t x){x^=x>>16;x*=0x7feb352d;x^=x>>15;x*=0x846ca68b;return x^(x>>16);}
inline V tangent(V v,V n){return sub(v,mul(n,dot(v,n)));}
struct Rig {std::array<V,12> joints{};unsigned valid=0;};
struct Pose {V torso{0,1.23f,0};V up{0,1,0};std::vector<V> boneUp;std::shared_ptr<const Rig> rig;};
struct Surface {V p{},normal{};float error=std::numeric_limits<float>::max();std::uint32_t links=0;H bone=0;};
struct Shape {
 V lo{},inv{};
 std::array<Surface,Cells> node{};
 std::array<H,Cells> nearest{};
 std::array<std::uint8_t,Cells> depth{};
 std::vector<H> active;
 void bounds(V low,V high){
   const int size[]={X,Y,Z};for(unsigned k=0;k<3;++k){float span=std::max(.045f,high[k]-low[k]);float pitch=span/(size[k]-2);lo[k]=low[k]-pitch*.5f;inv[k]=1.f/pitch;}
 }
 int cell(V p)const{
   int x=std::clamp(int(std::floor((p[0]-lo[0])*inv[0]+.5f)),0,X-1);
   int y=std::clamp(int(std::floor((p[1]-lo[1])*inv[1]+.5f)),0,Y-1);
   int z=std::clamp(int(std::floor((p[2]-lo[2])*inv[2]+.5f)),0,Z-1);return x+X*(y+Y*z);
 }
 V centre(int n)const{return {lo[0]+(n%X)/inv[0],lo[1]+((n/X)%Y)/inv[1],lo[2]+(n/(X*Y))/inv[2]};}
 int point(V p,V norm,H bone){
   const int n=cell(p);auto& s=node[n];auto delta=sub(p,centre(n));float err=dot(delta,delta);
   if(err<s.error){if(s.error==std::numeric_limits<float>::max())active.push_back(H(n));s.error=err;s.p=p;s.normal=norm;s.bone=bone;}
   return n;
 }
 void edge(int a,int b){
   if(a==b)return;
   const int dx=b%X-a%X,dy=(b/X)%Y-(a/X)%Y,dz=b/(X*Y)-a/(X*Y);
   if(std::abs(dx)>1||std::abs(dy)>1||std::abs(dz)>1)return;
   const int dir=(dz+1)*9+(dy+1)*3+dx+1;node[a].links|=1u<<dir;node[b].links|=1u<<(26-dir);
 }
 // Each link is traversed by this authenticated source triangle. Close but
 // unjoined garment panels have no edge across an empty lattice cell.
 void triangle(V a,V b,V c,H bone){
   V ab=sub(b,a),ac=sub(c,a),bc=sub(c,b),normal=unit(cross(ab,ac));
   if(dot(normal,normal)<.5f)return;
   float extent=0;for(int i=0;i<3;i++)extent=std::max({extent,std::abs(ab[i])*inv[i],std::abs(ac[i])*inv[i],std::abs(bc[i])*inv[i]});
   int steps=std::clamp(int(std::ceil(extent)),1,48);std::array<int,49>last{},current{};
   for(int j=0;j<=steps;++j){
    for(int i=0;i<=steps-j;++i){
      V p=add(a,add(mul(ab,float(i)/steps),mul(ac,float(j)/steps)));
      const int k=current[i]=point(p,normal,bone);
      if(i)edge(current[i-1],k);if(j){edge(last[i],k);edge(last[i+1],k);}
    }last.swap(current);
   }
 }
 template<class Solid> void finish(const Solid& inside){
    std::sort(active.begin(),active.end());
    for(int i=0;i<Cells;i++){
      auto p=centre(i);bool solid=inside(p);float best=std::numeric_limits<float>::max();int selected=-1;
      for(auto n:active){const auto& sample=node[n];auto d=sub(p,sample.p);float dd=dot(d,d);if(dd<best){best=dd;selected=n;}}
      nearest[i]=selected>=0?H(selected+1):0;if(selected<0)continue;
      // Leave air remote from any touched surface unsampled. Exterior
      // interpolation needs a single-cell margin through a silhouette.
      float maxPitch=std::max({1.f/inv[0],1.f/inv[1],1.f/inv[2]});
      if(!solid&&best>2.25f*maxPitch*maxPitch){nearest[i]=0;continue;}
      const auto& s=node[selected];
      const float inward=solid?std::min(std::sqrt(best),std::abs(dot(sub(p,s.p),s.normal))):0.f;
      depth[i]=std::uint8_t(std::clamp(int(std::lround(inward*125.f)),0,31));
    }
 }
 int closest(V p)const{
   int id=-1;float best=std::numeric_limits<float>::max();
   for(auto n:active){auto d=sub(p,node[n].p);float v=dot(d,d);if(v<best){id=n;best=v;}}
   return id;
 }
};
struct State {
 std::array<std::uint32_t,Cells> arrival{}; // first arrival at an exposed-time tick + 1
 std::array<H,Cells> progress{},display{};
 std::uint32_t heatTick{},heatStrength{};
 V lo{},inv{};std::uint32_t clock{},last{},pending{};bool initialized=false,needsSeeds=true,resumeGate=false;
 bool step(const Shape& g,const Pose& pose,const V* seeds,unsigned count,std::uint32_t tick,unsigned period,std::uint64_t salt){
   if(g.active.empty())return false;
   if(resumeGate){last=tick;pending=0;resumeGate=false;return false;}
   const auto prev=last;last=tick;
   if(initialized){const auto gap=tick-prev;
     if(!gap||gap>=0x80000000u)return false;
     if(gap<=std::max(8u,period+2u))pending+=gap;
     if(pending<4)return false;
   }
   const auto dt=std::min(24u,pending);pending=0;
   const bool first=!initialized;initialized=true;lo=g.lo;inv=g.inv;
   if(needsSeeds){for(unsigned i=0;i<count&&i<6;++i){int n=g.closest(seeds[i]);if(n>=0&&!arrival[n])arrival[n]=clock+1;}needsSeeds=false;}
   std::vector<std::pair<H,std::uint32_t>> newArrivals;
   for(auto n:g.active){
     if(arrival[n])continue;const auto& cell=g.node[n];float fastest=0;
     for(unsigned code=0;code<27;++code){if(!(cell.links&(1u<<code)))continue;
       const int other=n+int(code%3)-1+X*(int((code/3)%3)-1)+X*Y*(int(code/9)-1);
       if(!arrival[other])continue;
       const auto& s=g.node[other];auto delta=sub(cell.p,s.p);
       const float len=std::max(.009f,std::sqrt(dot(delta,delta)));auto dir=mul(delta,1.f/len);
       auto up=s.bone<pose.boneUp.size()?pose.boneUp[s.bone]:pose.up;
       float horizontal=std::abs(dot(up,s.normal));horizontal=std::clamp((horizontal-.42f)/.48f,0.f,1.f);
       auto flow=unit(add(mul(unit(tangent(up,s.normal)),1.f-horizontal),mul(unit(tangent(sub(pose.torso,s.p),s.normal)),horizontal)));
       float climb=std::clamp(.5f+.5f*dot(dir,flow),0.f,1.f);
       const float irregular=.78f+.44f*float(noise(std::uint32_t(salt)^std::uint32_t(n)*503u^std::uint32_t(other)*97u)&1023u)/1023.f;
       float velocity=(.014f+.111f*climb*climb)*irregular;
       fastest=std::max(fastest,velocity/len);
     }
     if(fastest<=0||!dt)continue;
     float before=progress[n]/65535.f,inc=fastest*float(dt)/16.f;
     if(before+inc>=1.f){auto offset=std::clamp<unsigned>(unsigned(std::ceil((1.f-before)*dt/inc)),1,dt);
       newArrivals.emplace_back(n,clock+offset+1);}
     else progress[n]=H(std::min(65534u,unsigned(std::lround((before+inc)*65535.f))));
   }
   for(auto x:newArrivals)arrival[x.first]=x.second;
   clock+=dt;
   bool changed=first;for(int i=0;i<Cells;++i){H value=0;if(g.nearest[i]){
       auto when=arrival[g.nearest[i]-1];if(when&&when<=clock+1){
         unsigned age=std::min(2046u,(clock+1-when)/2u);
         value=H((unsigned(g.depth[i])<<11)|(1u+age));}}
      if(display[i]!=value)changed=true;display[i]=value;}
   return changed;
 }
};
}
