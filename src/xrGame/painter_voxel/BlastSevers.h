#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>
namespace blast_severs {
struct V3 {double x=0,y=0,z=0;};
inline V3 sub(V3 a,V3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
inline V3 add(V3 a,V3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
inline V3 mul(V3 a,double s){return {a.x*s,a.y*s,a.z*s};}
inline double dot(V3 a,V3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline double len(V3 a){return std::sqrt(dot(a,a));}
// A tear site: the segment from its bone's joint to its child's (posed), the part's radius (the joint profile's), its limb
// (one sever per chain), whether it is trunk, the part of the segment a tear may take (fractions of its length), and the armour gate.
struct Site {std::uint16_t bone=65535;V3 from,to;double radius=.09;int chain=0;bool trunk=false;double lo=.25,hi=.75;bool open=true;};
struct Request {std::uint16_t bone=65535;V3 point,push;double proximity=0,score=0;int site=-1;};
struct Choice {std::vector<Request> tears;double pmax=0,trunkChance=0;unsigned eligible=0,wanted=0;bool trunkRoll=false;};
// grenadetrunk (2026-11-14): trunkChance 1.0 - a grenade on the body cuts the torso every time, falling off with its force.
struct Settings {double reach=2.5,start=.35,limb=.25,trunk=.70,trunkWeight=.5,threeFrom=.55,threeSpan=.35,trunkChance=1.,trunkFloor=.25;unsigned most=3;
 double blast=3.,radius=6.;}; // rpgblast: the explosive ("blast", "blast_r"; the RGD-5's: every distance as it is)
// the force from which a site is in play (the game asks the armour gate from here)
inline double inPlay(const Site&z,const Settings&k){return z.trunk?std::min(k.trunk,k.trunkFloor):k.limb;}
inline double rgdDistance(double d,double blast,double radius){constexpr double refBlast=3.,refRadius=6.,extinction=3.;
 if(!(d>0)||!(blast>0)||!(radius>0)||!std::isfinite(blast)||!std::isfinite(radius))return d;
 const double q=(refBlast/blast)*(1+(extinction-1)*d*d/(radius*radius))-1;
 return std::min(d,q>0?refRadius*std::sqrt(q/(extinction-1)):0.);}
// where an explosive's gore ends: rgdDistance = the reach (the blast log's "reach=")
inline double reachOf(double blast,double radius,double reach=2.5){constexpr double refBlast=3.,refRadius=6.,extinction=3.;
 if(!(blast>0)||!(radius>0)||!std::isfinite(blast)||!std::isfinite(radius))return reach;
 const double q=((1+(extinction-1)*reach*reach/(refRadius*refRadius))*blast/refBlast-1)/(extinction-1);
 return std::max(reach,q>0?radius*std::sqrt(q):0.);}
inline double rnd(std::uint64_t&s){s^=s<<13;s^=s>>7;s^=s<<17;return double(s>>11)*(1./9007199254740992.);}
inline Choice choose(const std::vector<Site>&sites,V3 E,double cover,std::uint64_t seed,const Settings&k=Settings{}){
 Choice out;std::uint64_t s=seed?seed:0x5EED5EEDull;for(int i=0;i<4;++i)rnd(s);
 std::uint64_t r=s^0x7B1A57C0FFEEull;if(!r)r=1;for(int i=0;i<4;++i)rnd(r); // the trunk roll's own stream
 struct Cand {std::size_t site;double p,score,t;V3 at;};std::vector<Cand> cands,trunks;
 for(std::size_t i=0;i<sites.size();++i){const auto&z=sites[i];const V3 ab=sub(z.to,z.from);const double L2=dot(ab,ab);if(!(L2>1e-8))continue;
  double t=std::clamp(dot(sub(E,z.from),ab)/L2,z.lo,z.hi);
  const V3 at=add(z.from,mul(ab,t));const double d=rgdDistance(len(sub(at,E)),k.blast,k.radius); // rpgblast: by the explosive
  const double p=std::clamp((1-d/k.reach)*cover,0.,1.);out.pmax=std::max(out.pmax,z.open?p:0.);
  const double score=p*p*std::pow(.09/std::max(.03,z.radius),.75)*(z.trunk?k.trunkWeight:1.);
  if(z.open&&z.trunk&&p>=k.trunkFloor)trunks.push_back({i,p,score,t,at});
  if(!z.open||p<(z.trunk?k.trunk:k.limb))continue;
  cands.push_back({i,p,score,t,at});}
 out.eligible=unsigned(cands.size());
 if(out.pmax<k.start)return out;
 out.trunkChance=trunks.empty()?0.:k.trunkChance*std::clamp((out.pmax-k.start)/(1-k.start),0.,1.);
 const Cand*trunkHit=nullptr;
 if(out.trunkChance>0&&rnd(r)<out.trunkChance){double total=0;for(const auto&c:trunks)total+=c.p;double pick=rnd(r)*total;
  for(const auto&c:trunks){trunkHit=&c;if(pick<c.p)break;pick-=c.p;}}
 if(cands.empty()&&!trunkHit)return out;
 out.wanted=std::min<unsigned>(k.most,2u+(rnd(s)<std::clamp((out.pmax-k.threeFrom)/k.threeSpan,0.,1.)?1u:0u));
 std::vector<int> taken;
 auto tear=[&](const Cand&c,double jitter){const auto&z=sites[c.site];taken.push_back(z.chain);
  // the tear lands near the point the blast hit hardest, jittered along the part (each blast tears its own place)
  const V3 ab=sub(z.to,z.from);const double t=std::clamp(c.t+(jitter-.5)*.16,z.lo,z.hi);const V3 at=add(z.from,mul(ab,t));
  V3 push=sub(at,E);double l=len(push);
  if(l<1e-6){const V3 a=mul(ab,1./std::sqrt(dot(ab,ab)));push=std::abs(a.y)<.9?V3{-a.z,0,a.x}:V3{1,0,0};l=len(push);}
  Request q;q.bone=z.bone;q.point=at;q.push=mul(push,1./l);q.proximity=c.p;q.score=c.score;q.site=int(c.site);out.tears.push_back(q);};
 if(trunkHit){out.trunkRoll=true;tear(*trunkHit,rnd(r));}
 while(out.tears.size()<out.wanted){double total=0;for(const auto&c:cands)if(std::find(taken.begin(),taken.end(),sites[c.site].chain)==taken.end())total+=c.score*c.score*c.score;
  if(!(total>0))break;double pick=rnd(s)*total;const Cand*chosen=nullptr;
  for(const auto&c:cands){if(std::find(taken.begin(),taken.end(),sites[c.site].chain)!=taken.end())continue;const double w=c.score*c.score*c.score;chosen=&c;if(pick<w)break;pick-=w;}
  if(!chosen)break;tear(*chosen,rnd(s));}
 return out;}
}
