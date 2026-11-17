#pragma once
// Sufficient certificate for the existing ring walk's winning part. Ambiguous
// queries keep the original walk, including its floating point pruning order.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include "ShortMinMax.h"

namespace gfy_part_owner_decision {
struct Interval { double lower{}, upper{}; bool valid{}; };

template<class Index, class Point, class FloorDiv>
inline Interval interval(const Index&ix,const Point&p,FloorDiv floorDiv){
 const double inf=std::numeric_limits<double>::infinity();
 Interval out{0.,inf,false};
 if(!ix.vol||!std::isfinite(ix.pitch)||!(ix.pitch>0.)||ix.F<=0||ix.F>4096||ix.C!=4*ix.F||
    ix.cnx<=0||ix.cny<=0||ix.cnx>1000000||ix.cny>1000000)return out;
 const std::size_t cells=std::size_t(ix.cnx)*std::size_t(ix.cny);
 if(cells>1048576||ix.start.size()!=cells+1||ix.fineStart.size()!=cells*16+1||
    ix.zlo.size()!=cells||ix.zhi.size()!=cells||ix.fzlo.size()!=cells*16||ix.fzhi.size()!=cells*16||
    ix.occupiedLeaves.empty())return out;
 for(int a=0;a<3;++a)if(ix.lo[a]<-1000000||ix.hi[a]>1000000||ix.lo[a]>=ix.hi[a])return out;
 if(ix.cx0<-1000000||ix.cx0>1000000||ix.cy0<-1000000||ix.cy0>1000000)return out;
 const double fx=(p.x-ix.ox)/ix.pitch,fy=(p.y-ix.oy)/ix.pitch,fz=(p.z-ix.oz)/ix.pitch,pitch2=ix.pitch*ix.pitch;
 if(!std::isfinite(fx)||!std::isfinite(fy)||!std::isfinite(fz)||std::abs(fx)>1000000.||
    std::abs(fy)>1000000.||std::abs(fz)>1000000.||!std::isfinite(pitch2)||!(pitch2>0.))return out;
 const auto qxl=std::int64_t(floorDiv(int(std::floor(fx)),ix.C))-ix.cx0;
 const auto qyl=std::int64_t(floorDiv(int(std::floor(fy)),ix.C))-ix.cy0;
 const auto rmax=painter_short_minmax::maxOf(qxl,std::int64_t(ix.cnx)-1-qxl,qyl,std::int64_t(ix.cny)-1-qyl,std::int64_t(0))+1;
 // All old int arithmetic through the witness ring is now representable.
 if(rmax>1000000||std::abs(qxl)+rmax>2000000||std::abs(qyl)+rmax>2000000||
    rmax*ix.C>1000000000||std::abs(std::int64_t(ix.cx0))*ix.C+std::int64_t(ix.cnx)*ix.C>1000000000||
    std::abs(std::int64_t(ix.cy0))*ix.C+std::int64_t(ix.cny)*ix.C>1000000000)return out;
 const int qx=int(qxl),qy=int(qyl);
 const double ax=painter_short_minmax::maxOf(double(ix.lo[0])-fx,fx-double(ix.hi[0]),0.);
 const double ay=painter_short_minmax::maxOf(double(ix.lo[1])-fy,fy-double(ix.hi[1]),0.);
 const double az=painter_short_minmax::maxOf(double(ix.lo[2])-fz,fz-double(ix.hi[2]),0.);
 const double outer=(ax*ax+ay*ay+az*az)*pitch2;
 const double overallLower=(ax*ax+ay*ay)*pitch2+az*az*pitch2;
 if(!std::isfinite(outer)||!std::isfinite(overallLower))return out;
 struct Choice {double lower;std::uint32_t f;};
 std::array<Choice,2> chosen{{{inf,0},{inf,0}}};
 // Cheap lower bound: never scan every occupied leaf merely to decline a
 // certificate. A bounded local witness search may be poor, but fallback is exact.
 double lower=overallLower;
 auto offer=[&](std::uint32_t f){
  if(f>=cells*16||ix.fineStart[f]>=ix.fineStart[f+1]||ix.fineStart[f+1]>ix.order.size())return;
  const std::size_t i=f/16;const int cx=int(i/std::size_t(ix.cny)),cy=int(i%std::size_t(ix.cny));
  const double x0=double((ix.cx0+cx)*ix.C),y0=double((ix.cy0+cy)*ix.C);
  const double fx0=x0+double(int((f-i*16)/4)*ix.F),fy0=y0+double(int((f-i*16)%4)*ix.F);
  const double dx=painter_short_minmax::maxOf(fx0-fx,fx-(fx0+ix.F),0.),dy=painter_short_minmax::maxOf(fy0-fy,fy-(fy0+ix.F),0.);
  const double dz=painter_short_minmax::maxOf(double(ix.fzlo[f])-fz,fz-double(ix.fzhi[f]),0.);
  const double value=(dx*dx+dy*dy)*pitch2+dz*dz*pitch2;
  if(!std::isfinite(value))return;
  if(value<chosen[0].lower){chosen[1]=chosen[0];chosen[0]={value,f};}
  else if(f!=chosen[0].f&&value<chosen[1].lower)chosen[1]={value,f};
 };
 const int bx=std::max(0,std::min(ix.cnx-1,qx)),by=std::max(0,std::min(ix.cny-1,qy));
 const std::size_t central=std::size_t(bx)*std::size_t(ix.cny)+std::size_t(by);
 if(ix.start[central]!=ix.start[central+1]){
  for(std::size_t f=central*16;f<central*16+16;++f)offer(std::uint32_t(f));
 }else{
  for(int x=std::max(0,bx-1);x<=std::min(ix.cnx-1,bx+1);++x)
   for(int y=std::max(0,by-1);y<=std::min(ix.cny-1,by+1);++y){
    const auto i=std::size_t(x)*std::size_t(ix.cny)+std::size_t(y);
    if(ix.start[i]==ix.start[i+1])continue;
    for(std::size_t f=i*16;f<i*16+16;++f)offer(std::uint32_t(f));}
 }
 if(!std::isfinite(chosen[0].lower)){
  offer(ix.occupiedLeaves.front());offer(ix.occupiedLeaves[ix.occupiedLeaves.size()/2]);offer(ix.occupiedLeaves.back());
 }
 double upper=inf;
 for(const auto&choice:chosen){if(!std::isfinite(choice.lower))continue;
  const std::size_t f=choice.f,i=f/16;const int cx=int(i/std::size_t(ix.cny)),cy=int(i%std::size_t(ix.cny));
  const int r=std::max(std::abs(cx-qx),std::abs(cy-qy));
  const double lb=r>0?double((r-1)*ix.C)*ix.pitch:0.;
  const double x0=double((ix.cx0+cx)*ix.C),y0=double((ix.cy0+cy)*ix.C);
  const double dx=painter_short_minmax::maxOf(x0-fx,fx-(x0+ix.C),0.),dy=painter_short_minmax::maxOf(y0-fy,fy-(y0+ix.C),0.);
  const double dz=painter_short_minmax::maxOf(double(ix.zlo[i])-fz,fz-double(ix.zhi[i]),0.);
  const double coarse=(dx*dx+dy*dy+dz*dz)*pitch2;
  const double fx0=x0+double(int((f-i*16)/4)*ix.F),fy0=y0+double(int((f-i*16)%4)*ix.F);
  const double dxF=painter_short_minmax::maxOf(fx0-fx,fx-(fx0+ix.F),0.),dyF=painter_short_minmax::maxOf(fy0-fy,fy-(fy0+ix.F),0.);
  const double dzF=painter_short_minmax::maxOf(double(ix.fzlo[f])-fz,fz-double(ix.fzhi[f]),0.);
  const double fine=(dxF*dxF+dyF*dyF+dzF*dzF)*pitch2;
  const double path=painter_short_minmax::maxOf(outer,lb*lb,coarse,fine);
  const auto first=ix.fineStart[f],last=ix.fineStart[f+1];
  const std::array<std::uint32_t,3> samples{{first,first+(last-first)/2,last-1}};
  for(const auto o:samples){const auto ci=ix.order[o];if(ci>=ix.vol->columns.size())return out;
   const auto&c=ix.vol->columns[ci];
   if(c.first>ix.vol->spans.size()||c.count>ix.vol->spans.size()-c.first)return out;
   const double dxC=painter_short_minmax::maxOf(double(c.x)-fx,fx-double(c.x+1),0.),dyC=painter_short_minmax::maxOf(double(c.y)-fy,fy-double(c.y+1),0.);
   const double dxy2=(dxC*dxC+dyC*dyC)*pitch2;
   for(auto s=c.first;s<c.first+c.count;++s){const auto&sp=ix.vol->spans[s];if(sp.lo>=sp.hi)return out;
    const double dzS=painter_short_minmax::maxOf(double(sp.lo)-fz,fz-double(sp.hi),0.);
    const double d2=dxy2+dzS*dzS*pitch2;
    const double witness=painter_short_minmax::maxOf(path,dxy2,d2);
    if(!std::isfinite(witness))return out;
    upper=std::min(upper,witness);
   }
  }
 }
 lower=std::max(lower,overallLower);
 // Padding only makes an already proved enclosure wider. Its magnitude is
 // not an assumption about the error of the old traversal's pruning bounds.
 out.lower=std::nextafter(lower*(1.-1e-9),0.);
 out.upper=std::nextafter(upper*(1.+1e-9),inf);
 out.valid=std::isfinite(out.lower)&&out.lower>=0.&&std::isfinite(out.upper)&&out.upper>=out.lower;
 return out;
}

template<class Parts,class Point,class FloorDiv>
inline int decide(const Parts&parts,const Point&p,FloorDiv floorDiv){
 if(parts.empty()||parts.size()>12)return -1;
 std::array<Interval,12> bounds;
 std::size_t winner=0;
 for(std::size_t k=0;k<parts.size();++k){
  bounds[k]=interval(*parts[k].second,p,floorDiv);
  if(!bounds[k].valid)return -1;
  if(bounds[k].upper<bounds[winner].upper)winner=k;
 }
 for(std::size_t k=0;k<parts.size();++k)if(k!=winner&&!(bounds[winner].upper<bounds[k].lower))return -1;
 return parts[winner].first;
}
}
