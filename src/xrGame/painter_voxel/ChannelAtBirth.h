#pragma once
// PRIVATE native diagnostic: plan a finite analytic channel before triangles.
// Not renderer integration. No voxel boundary faces and no mesh reducer.
#include <fstream>
#include <iomanip>
#include <set>
#include <stdexcept>

namespace channelbirth {
constexpr double tau=6.2831853071795864769;
struct Stats {std::size_t angles{},rings{},triangles{},checks{},outsidePatches{};double ms{},error{};};
struct Plan {std::vector<double> a,t;std::vector<Vec> vertices;std::vector<std::array<std::uint32_t,3>> triangles;Stats stats;bool admitted{};};
template<class Cut> Vec surface(const Cut& cut,double a,double t){
 const Vec ref=std::abs(cut.direction.x)<.8?Vec{1,0,0}:Vec{0,1,0};
 const Vec u=cut.edge.enabled()||cut.cracks?cut.edge_u:unit(cross(cut.direction,ref));
 const Vec v=cut.edge.enabled()||cut.cracks?cut.edge_v:cross(cut.direction,u);
 const double x=std::cos(a),y=std::sin(a);
 const double z=cut.exit_distance<0?std::max(0.,t-(cut.penetration-cut.cap))/cut.cap:0.;
 const double radius=cut.radiusAt(t)*std::sqrt(std::max(0.,1-z*z))
   *cut.crackScale(x,y,t)*cut.edge.radiusMultiplier(x,y,t/cut.penetration);
 return cut.entry+cut.direction*t+(u*x+v*y)*radius;
}
inline double wrapped(double a){a=std::fmod(a,tau);return a<0?a+tau:a;}
struct Box {Vec lo,hi;};
template<class C> auto planarSheet(const C&c,Plan&out,int)->decltype(c.planarSheet(out),bool()){return c.planarSheet(out);}
template<class C> bool planarSheet(const C&,Plan&,long){return false;}
template<class C> auto sheetBounds(const C&c,double a,double b,Box&out,int)->decltype(c.sheetBounds(a,b,out),bool()){return c.sheetBounds(a,b,out);}
template<class C> bool sheetBounds(const C&,double,double,Box&,long){return false;}
template<class C> auto extraDepths(const C&c,std::vector<double>&out,int)->decltype(c.seedDepths(out),void()){c.seedDepths(out);}
template<class C> void extraDepths(const C&,std::vector<double>&,long){}
// Conservative continuous bounds, not a centroid/sample-only rejection.
// Contract: the engine Tunnel family has monotone flare, sine-squared swell,
// nondecreasing nonnegative crack tents, and a piecewise-linear angular/depth
// edge table. This is not an interval bound for arbitrary Cut functors.
// Products use independent interval bounds; loss of correlation over-admits.
template<class Cut> Box patchBounds(const Cut& cut,double a,double b,double c,double d,bool disk=false){
 if(!(0.<=a&&a<=b&&b<=tau&&-cut.inlet<=c&&c<=d&&d<=cut.penetration))throw std::invalid_argument("channel_patch_parameter_range");
 Box sheet;if(sheetBounds(cut,c,d,sheet,0))return sheet;
 const Vec ref=std::abs(cut.direction.x)<.8?Vec{1,0,0}:Vec{0,1,0};
 const Vec u=cut.edge.enabled()||cut.cracks?cut.edge_u:unit(cross(cut.direction,ref));
 const Vec v=cut.edge.enabled()||cut.cracks?cut.edge_v:cross(cut.direction,u);
 std::vector<double> depths{c,d};
 for(double t:{cut.cav_t0,cut.cav_t1,.5*(cut.cav_t0+cut.cav_t1),cut.penetration-cut.cap,
   cut.exit_distance,cut.exit_distance-std::min(cut.exit_distance,cut.radius*1.5),cut.exit_distance-cut.burst_len})if(t>c&&t<d)depths.push_back(t);
 double swellMin=std::numeric_limits<double>::infinity(),swellMax=0,flareMin=swellMin,flareMax=-swellMin;
 for(double t:depths){swellMin=std::min(swellMin,cut.swell(t));swellMax=std::max(swellMax,cut.swell(t));flareMin=std::min(flareMin,cut.flareAt(t));flareMax=std::max(flareMax,cut.flareAt(t));}
 double low=cut.radius*(1+flareMin+swellMin),high=cut.radius*(1+flareMax+swellMax);
 auto cap=[&](double t){if(cut.exit_distance>=0)return 1.;const double z=std::max(0.,t-(cut.penetration-cut.cap))/cut.cap;return std::sqrt(std::max(0.,1-z*z));};
 low*=cap(d);high*=cap(c);
 std::vector<double> angles{a,b};
 for(unsigned i=0;i<=128;++i){const double t=tau*i/128;if(t>a&&t<b)angles.push_back(t);}
 for(int i=0;i<cut.cracks;++i)for(double t:{wrapped(cut.crack_angle[i]),wrapped(cut.crack_angle[i]-cut.crack_width),wrapped(cut.crack_angle[i]+cut.crack_width)})if(t>a&&t<b)angles.push_back(t);
 double edgeLow=1.,edgeHigh=0.,crackLow=1.,crackHigh=0.; // max of overlapping crack tents may have an unsampled interior minimum
 for(double angle:angles)for(double t:depths){const double x=std::cos(angle),y=std::sin(angle),e=cut.edge.radiusMultiplier(x,y,t/cut.penetration),k=cut.crackScale(x,y,t);
  edgeLow=std::min(edgeLow,e);edgeHigh=std::max(edgeHigh,e);crackLow=std::min(crackLow,k);crackHigh=std::max(crackHigh,k);}
 low*=edgeLow*crackLow;high*=edgeHigh*crackHigh;if(disk)low=0.;
 Box box;
 for(int axis=0;axis<3;++axis){auto radial=[&](double t){return u[axis]*std::cos(t)+v[axis]*std::sin(t);};double qlo=std::min(radial(a),radial(b)),qhi=std::max(radial(a),radial(b));
  const double peak=std::atan2(v[axis],u[axis]);for(int k=-3;k<=3;++k){double t=peak+k*tau*.5;if(t>a&&t<b){qlo=std::min(qlo,radial(t));qhi=std::max(qhi,radial(t));}}
  const double p0=cut.entry[axis]+cut.direction[axis]*c,p1=cut.entry[axis]+cut.direction[axis]*d;
  const double products[]={qlo*low,qlo*high,qhi*low,qhi*high};
  box.lo[axis]=std::min(p0,p1)+*std::min_element(products,products+4)-1e-9;
  box.hi[axis]=std::max(p0,p1)+*std::max_element(products,products+4)+1e-9;
 }
 return box;
}
struct WholeDomain {bool operator()(const Box&)const noexcept{return true;}};
template<class Cut> Plan plan(const Cut& cut,double tolerance=.002){
 const auto begin=core::Clock::now();Plan out;std::set<double> angles,depths;
 for(unsigned i=0;i<8;++i)angles.insert(tau*i/8);angles.insert(tau);
 depths.insert(-cut.inlet);depths.insert(0.);depths.insert(cut.penetration);
 auto depth=[&](double t){if(t> -cut.inlet&&t<cut.penetration)depths.insert(t);};
 {std::vector<double> extra;extraDepths(cut,extra,0);for(double t:extra)depth(t);}
 if(cut.exit_distance<0)depth(cut.penetration-cut.cap);
 else {depth(cut.exit_distance);depth(cut.exit_distance-std::min(cut.exit_distance,cut.radius*1.5));}
 if(cut.cav_amp>0){depth(cut.cav_t0);depth(.5*(cut.cav_t0+cut.cav_t1));depth(cut.cav_t1);}
 if(cut.cracks){depth(cut.exit_distance-cut.burst_len);
  for(int i=0;i<cut.cracks;++i){angles.insert(wrapped(cut.crack_angle[i]-cut.crack_width));angles.insert(wrapped(cut.crack_angle[i]));angles.insert(wrapped(cut.crack_angle[i]+cut.crack_width));}}
 // An angular edge-table corner is only a scalar sample, never a mesh vertex.
 // Refine the parameter plan when those samples cannot fit the finite patch.
 std::vector<double> witnesses;for(unsigned i=0;i<=256;++i)witnesses.push_back(tau*i/256);
 for(double a:angles)witnesses.push_back(a);std::sort(witnesses.begin(),witnesses.end());
 for(unsigned round=0;round<32;++round){
  out.a.assign(angles.begin(),angles.end());out.t.assign(depths.begin(),depths.end());
  std::set<double> addA,addT;double error=0;
  for(std::size_t j=0;j+1<out.t.size();++j)for(std::size_t i=0;i+1<out.a.size();++i){
   const double a0=out.a[i],a1=out.a[i+1],t0=out.t[j],t1=out.t[j+1];
   const Vec p00=surface(cut,a0,t0),p10=surface(cut,a1,t0),p01=surface(cut,a0,t1),p11=surface(cut,a1,t1);
   std::vector<double> aw{a0+(a1-a0)*.25,a0+(a1-a0)*.5,a0+(a1-a0)*.75};
   auto first=std::upper_bound(witnesses.begin(),witnesses.end(),a0),last=std::lower_bound(witnesses.begin(),witnesses.end(),a1);aw.insert(aw.end(),first,last);
   double angleError=0,depthError=0,mixedError=0;
   for(double a:aw){const double u=(a-a0)/(a1-a0);for(double v:{0.,.25,.5,.75,1.}){
     const double t=t0+(t1-t0)*v;const Vec exact=surface(cut,a,t);
     const Vec approx=u>=v?p00*(1-u)+p10*(u-v)+p11*v:p00*(1-v)+p11*u+p01*(v-u);
     const double e=length(exact-approx);error=std::max(error,e);++out.stats.checks;
     if(v==0||v==1)angleError=std::max(angleError,e);else mixedError=std::max(mixedError,e);
   }}
   for(double v:{.25,.5,.75})for(double u:{0.,1.}){const Vec exact=surface(cut,a0+(a1-a0)*u,t0+(t1-t0)*v);const Vec approx=u?p10*(1-v)+p11*v:p00*(1-v)+p01*v;depthError=std::max(depthError,length(exact-approx));++out.stats.checks;}
   if(angleError>tolerance)addA.insert((a0+a1)*.5);
   if(depthError>tolerance)addT.insert((t0+t1)*.5);
   if(mixedError>tolerance&&angleError<=tolerance&&depthError<=tolerance){
    // The bilinear cross term can exceed both edge errors. Subdivide its
    // larger parameter span rather than creating and reducing a dense mesh.
    if((a1-a0)*cut.radiusAt((t0+t1)*.5)>t1-t0)addA.insert((a0+a1)*.5);else addT.insert((t0+t1)*.5);
   }
  }
  out.stats.error=error;
  if(addA.empty()&&addT.empty()){out.admitted=true;break;}
  angles.insert(addA.begin(),addA.end());depths.insert(addT.begin(),addT.end());
  if(angles.size()*depths.size()>65536)break; // explicit diagnostic refusal
 }
 out.stats.angles=out.a.size()-1;out.stats.rings=out.t.size();
 // An inlet fan and either a far cap or one collapsed distal point close it.
 out.stats.triangles=out.stats.angles*(2*(out.stats.rings-1)+2);
 if(cut.exit_distance<0)out.stats.triangles-=2*out.stats.angles;
 out.stats.ms=core::ms(begin);return out;
}
// Local scalar patches avoid multiplying a single narrow tear's angular
// refinement by every section along the whole channel. No mesh exists during
// this phase. Shared parameter-edge ledgers determine the final rings before
// final vertex/triangle emission. Scalar ledgers are not a temporary dense mesh.
template<class Cut,class Domain> Plan planLocalDomain(const Cut& cut,const Domain& domain,double tolerance=.009,double patchFraction=.9){
 const auto begin=core::Clock::now();Plan out;
 if(planarSheet(cut,out,0)){out.stats.ms=core::ms(begin);return out;}
 struct Rect{double a,b,c,d;};std::vector<Rect> patches;
 std::set<double> ts{-cut.inlet,0.,cut.penetration};
 auto addT=[&](double x){if(x> -cut.inlet&&x<cut.penetration)ts.insert(x);};
 {std::vector<double> extra;extraDepths(cut,extra,0);for(double t:extra)addT(t);}
 if(cut.exit_distance<0)addT(cut.penetration-cut.cap);
 else {addT(cut.exit_distance);addT(cut.exit_distance-std::min(cut.exit_distance,cut.radius*1.5));}
 if(cut.cav_amp>0){addT(cut.cav_t0);addT(.5*(cut.cav_t0+cut.cav_t1));addT(cut.cav_t1);}
 if(cut.cracks||cut.burst_amp>0)addT(cut.exit_distance-cut.burst_len);
 // Independently calculated critical depths may differ only by arithmetic
 // roundoff (7*r versus 10*r-3*r). Canonicalize those scalar boundaries BEFORE
 // making patches: otherwise an almost-zero strip emits coincident corners.
 // This is machine-scale, not a display-resolution or wound-size reduction.
 const double depthEpsilon=64*std::numeric_limits<double>::epsilon()*std::max({1.,std::abs(cut.entry.x),std::abs(cut.entry.y),std::abs(cut.entry.z),cut.penetration,cut.inlet});
 for(double t:ts){if(out.t.empty()||t-out.t.back()>depthEpsilon)out.t.push_back(t);
  else if(t==cut.penetration||(t==0.&&out.t.back()!=-cut.inlet))out.t.back()=t;}
 if(out.t.size()<2){out.stats.ms=core::ms(begin);return out;}
 for(unsigned k=0;k<=8;++k)out.a.push_back(tau*k/8);
 // R24: a slab's box corners are vertices from the start (a chord across a corner would admit a wedge within tolerance).
 {std::vector<double> seeds;cut.seedAngles(seeds);for(double a:seeds){a=wrapped(a);if(a>0&&a<tau)out.a.push_back(a);}
  std::sort(out.a.begin(),out.a.end());out.a.erase(std::unique(out.a.begin(),out.a.end()),out.a.end());}
 for(std::size_t j=0;j+1<out.t.size();++j)for(std::size_t k=0;k+1<out.a.size();++k)patches.push_back({out.a[k],out.a[k+1],out.t[j],out.t[j+1]});
 // Clamp the sampled same-parameter distance for small bores. This is not a
 // radial or Hausdorff error bound; finite witnesses cannot prove either.
 tolerance=std::min(tolerance,cut.radius*.35);
 std::vector<double> features;for(unsigned k=0;k<=128;++k)features.push_back(tau*k/128);
 {std::vector<double> seeds;cut.seedAngles(seeds);for(double a:seeds)features.push_back(wrapped(a));} // R24
 for(int k=0;k<cut.cracks;++k){features.push_back(wrapped(cut.crack_angle[k]));features.push_back(wrapped(cut.crack_angle[k]-cut.crack_width));features.push_back(wrapped(cut.crack_angle[k]+cut.crack_width));}
 std::sort(features.begin(),features.end());
 const bool nearCap=domain(patchBounds(cut,0.,tau,-cut.inlet,-cut.inlet,true));
 const bool farCap=cut.exit_distance>=0&&domain(patchBounds(cut,0.,tau,cut.penetration,cut.penetration,true));
 auto samples=[&](const Rect& r){std::vector<double> a{r.a+(r.b-r.a)*.25,(r.a+r.b)*.5,r.a+(r.b-r.a)*.75};
  auto f=std::upper_bound(features.begin(),features.end(),r.a),l=std::lower_bound(features.begin(),features.end(),r.b);a.insert(a.end(),f,l);return a;};
 double admittedError=0;
 for(std::size_t i=0;i<patches.size();){const Rect r=patches[i];
  if(!((nearCap&&r.c==-cut.inlet)||(farCap&&r.d==cut.penetration))&&!domain(patchBounds(cut,r.a,r.b,r.c,r.d))){
   patches[i]=patches.back();patches.pop_back();++out.stats.outsidePatches;continue;
  }
  const Vec q00=surface(cut,r.a,r.c),q10=surface(cut,r.b,r.c),q01=surface(cut,r.a,r.d),q11=surface(cut,r.b,r.d);
  double angular=0,axial=0,maximum=0;
  for(double a:samples(r)){const double u=(a-r.a)/(r.b-r.a);for(double v:{0.,.25,.5,.75,1.}){
   const Vec q=surface(cut,a,r.c+(r.d-r.c)*v);const Vec p=u>=v?q00*(1-u)+q10*(u-v)+q11*v:q00*(1-v)+q11*u+q01*(v-u);
   const double error=length(p-q);maximum=std::max(maximum,error);if(v==0||v==1)angular=std::max(angular,error);++out.stats.checks;}}
  for(double u:{0.,1.})for(double v:{.25,.5,.75}){const double error=length(surface(cut,r.a+(r.b-r.a)*u,r.c+(r.d-r.c)*v)-(u?q10*(1-v)+q11*v:q00*(1-v)+q01*v));axial=std::max(axial,error);++out.stats.checks;}
  if(std::max(maximum,axial)<=tolerance*patchFraction){admittedError=std::max(admittedError,maximum);++i;continue;}
  if(patches.size()>32768||r.b-r.a<1e-9||r.d-r.c<1e-10){out.stats.ms=core::ms(begin);return out;}
  const bool splitA=angular>axial||(angular==axial&&(r.b-r.a)*cut.radiusAt((r.c+r.d)*.5)>r.d-r.c);
  if(splitA){double m=(r.a+r.b)*.5;patches[i].b=m;patches.push_back({m,r.b,r.c,r.d});}
  else {double m=(r.c+r.d)*.5;patches[i].d=m;patches.push_back({r.a,r.b,m,r.d});}
 }
 // All wall patch edge samples are canonical BEFORE a vertex/triangle exists.
 // The seam at 0 / 2pi uses one physical ledger and one final vertex id.
 auto seam=[](double a){return a==tau?0.:a;};
 std::map<double,std::set<double>> horizontal,vertical;
 for(const auto& r:patches){horizontal[r.c].insert(r.a);horizontal[r.c].insert(r.b);horizontal[r.d].insert(r.a);horizontal[r.d].insert(r.b);
  vertical[seam(r.a)].insert(r.c);vertical[seam(r.a)].insert(r.d);vertical[seam(r.b)].insert(r.c);vertical[seam(r.b)].insert(r.d);}
 using UV=std::pair<double,double>;struct Ring{std::vector<UV> p;UV center;bool fan{};};std::vector<Ring> rings;rings.reserve(patches.size()+2);
 for(const auto& r:patches){Ring ring;ring.center={(r.a+r.b)*.5,(r.c+r.d)*.5};
  auto& lo=horizontal.at(r.c);for(auto it=lo.lower_bound(r.a);it!=lo.end()&&*it<r.b;++it)ring.p.push_back({*it,r.c});
  auto& right=vertical.at(seam(r.b));for(auto it=right.lower_bound(r.c);it!=right.end()&&*it<r.d;++it)ring.p.push_back({r.b,*it});
  auto& hi=horizontal.at(r.d);for(auto it=hi.upper_bound(r.b);it!=hi.begin();){--it;if(*it<=r.a)break;ring.p.push_back({*it,r.d});}
  auto& left=vertical.at(seam(r.a));for(auto it=left.upper_bound(r.d);it!=left.begin();){--it;if(*it<=r.c)break;ring.p.push_back({r.a,*it});}
  ring.fan=ring.p.size()!=4;rings.push_back(std::move(ring));}
 // Fine surface comparison after the abstract edge ledger, before emission.
 // Fan subdivision changes interpolation; admission validates that actual
// planned interpolation as well rather than assuming the coarse quad test.
 auto bary=[](UV q,UV a,UV b,UV c,double& x,double& y,double& z){const double den=(b.second-c.second)*(a.first-c.first)+(c.first-b.first)*(a.second-c.second);if(std::abs(den)<1e-25)return false;
  x=((b.second-c.second)*(q.first-c.first)+(c.first-b.first)*(q.second-c.second))/den;
  y=((c.second-a.second)*(q.first-c.first)+(a.first-c.first)*(q.second-c.second))/den;z=1-x-y;return x>=-1e-9&&y>=-1e-9&&z>=-1e-9;};
 double finalError=0;
 for(std::size_t n=0;n<rings.size();++n){const auto& ring=rings[n];const auto& r=patches[n];
  std::vector<std::array<UV,3>> facets;if(ring.fan){for(std::size_t k=0;k<ring.p.size();++k)facets.push_back({ring.center,ring.p[k],ring.p[(k+1)%ring.p.size()]});}
  else{facets.push_back({ring.p[0],ring.p[1],ring.p[2]});facets.push_back({ring.p[0],ring.p[2],ring.p[3]});}
  // A planned corner is immutable. Evaluate it once per compact facet rather
  // than repeating its trigonometry for every error witness.
  std::vector<std::array<Vec,3>> corners;corners.reserve(facets.size());
  for(const auto&f:facets)corners.push_back({surface(cut,f[0].first,f[0].second),surface(cut,f[1].first,f[1].second),surface(cut,f[2].first,f[2].second)});
  auto finalAngles=samples(r);finalAngles.push_back(r.a);finalAngles.push_back(r.b);
  for(double a:finalAngles)for(double v:{0.,.125,.25,.5,.75,.875,1.}){UV q{a,r.c+(r.d-r.c)*v};bool found=false;
   for(std::size_t fidx=0;fidx<facets.size();++fidx){const auto&f=facets[fidx];double x,y,z;if(!bary(q,f[0],f[1],f[2],x,y,z))continue;
    const auto&points=corners[fidx];const Vec p=points[0]*x+points[1]*y+points[2]*z;
    finalError=std::max(finalError,length(surface(cut,q.first,q.second)-p));found=true;++out.stats.checks;break;}
   if(!found){out.stats.ms=core::ms(begin);return out;}}
 }
 out.stats.error=finalError;if(finalError>tolerance){
  // No triangles have been allocated. A hanging-edge fan can exceed the
  // patch's original diagonal interpolation: retry the scalar plan only.
  if(patchFraction>.5){auto retry=planLocalDomain(cut,domain,tolerance,std::max(.5,patchFraction*.8));retry.stats.checks+=out.stats.checks;retry.stats.ms=core::ms(begin);return retry;}
  out.stats.ms=core::ms(begin);return out;
 }
 // Only here are final mesh vertices and triangles allocated. Neither fine
 // voxel faces nor an oversized temporary triangle mesh has been constructed.
 auto key=[&](UV p){p.first=seam(p.first);if(cut.exit_distance<0&&p.second==cut.penetration)p.first=0.;return p;};
 std::set<UV> plannedVertices;std::size_t plannedTriangles=0;
 auto countTriangle=[&](UV a,UV b,UV c){a=key(a);b=key(b);c=key(c);if(a!=b&&a!=c&&b!=c)++plannedTriangles;};
 for(const auto&ring:rings){for(UV p:ring.p)plannedVertices.insert(key(p));
  if(ring.fan){plannedVertices.insert(key(ring.center));for(std::size_t k=0;k<ring.p.size();++k)countTriangle(ring.center,ring.p[k],ring.p[(k+1)%ring.p.size()]);}
  else {countTriangle(ring.p[0],ring.p[1],ring.p[2]);countTriangle(ring.p[0],ring.p[2],ring.p[3]);}}
 for(double t:{-cut.inlet,cut.penetration})if(t==-cut.inlet?nearCap:farCap){for(double a:horizontal.at(t))if(a<tau){plannedVertices.insert(key({a,t}));++plannedTriangles;}}
 out.vertices.reserve(plannedVertices.size()+unsigned(nearCap)+unsigned(farCap));out.triangles.reserve(plannedTriangles);
 std::map<UV,std::uint32_t> ids;
 auto vertex=[&](UV p){p.first=seam(p.first);if(cut.exit_distance<0&&p.second==cut.penetration)p.first=0.;auto it=ids.find(p);if(it!=ids.end())return it->second;
  const auto id=std::uint32_t(out.vertices.size());out.vertices.push_back(cut.exit_distance<0&&p.second==cut.penetration?cut.entry+cut.direction*cut.penetration:surface(cut,p.first,p.second));ids.emplace(p,id);return id;};
 auto triangle=[&](std::uint32_t a,std::uint32_t b,std::uint32_t c){if(a==b||a==c||b==c)return;out.triangles.push_back({a,b,c});};
 for(const auto& ring:rings){std::vector<std::uint32_t> v;for(UV p:ring.p)v.push_back(vertex(p));if(ring.fan){const auto center=vertex(ring.center);for(std::size_t k=0;k<v.size();++k)triangle(center,v[k],v[(k+1)%v.size()]);}
  else {triangle(v[0],v[1],v[2]);triangle(v[0],v[2],v[3]);}}
 auto cap=[&](double t,bool reverse){auto center=std::uint32_t(out.vertices.size());out.vertices.push_back(cut.entry+cut.direction*t);std::vector<std::uint32_t> v;
  for(double a:horizontal.at(t))if(a<tau)v.push_back(vertex({a,t}));for(std::size_t k=0;k<v.size();++k)if(reverse)triangle(center,v[(k+1)%v.size()],v[k]);else triangle(center,v[k],v[(k+1)%v.size()]);};
 if(nearCap)cap(-cut.inlet,true);if(farCap)cap(cut.penetration,false);
 out.admitted=true;out.stats.angles=horizontal.size();out.stats.rings=patches.size();out.stats.triangles=out.triangles.size();out.stats.ms=core::ms(begin);return out;
}
template<class Cut> Plan planLocal(const Cut& cut,double tolerance=.009,double patchFraction=.9){return planLocalDomain(cut,WholeDomain{},tolerance,patchFraction);}
template<class Cut> void diagnostic(const Cut& cut){
 const char* prefix=std::getenv("GFY_CHANNEL_DUMP");if(!prefix)return;
 static std::atomic<unsigned> serial{0};const unsigned id=serial.fetch_add(1);
 const auto result=plan(cut);const std::string base=std::string(prefix)+"-"+std::to_string(id);
 std::ofstream out(base+".json");out<<std::setprecision(17);
 auto vec=[&](Vec p){out<<'['<<p.x<<','<<p.y<<','<<p.z<<']';};
 out<<"{\"id\":"<<id<<",\"entry\":";vec(cut.entry);out<<",\"direction\":";vec(cut.direction);out<<",\"u\":";vec(cut.edge_u);out<<",\"v\":";vec(cut.edge_v);
 out<<",\"radius\":"<<cut.radius<<",\"penetration\":"<<cut.penetration<<",\"cap\":"<<cut.cap<<",\"inlet\":"<<cut.inlet<<",\"exit_distance\":"<<cut.exit_distance<<",\"exit_mult\":"<<cut.exit_mult<<",\"cav_amp\":"<<cut.cav_amp<<",\"cav_t0\":"<<cut.cav_t0<<",\"cav_t1\":"<<cut.cav_t1<<",\"burst_amp\":"<<cut.burst_amp<<",\"burst_len\":"<<cut.burst_len<<",\"crack_width\":"<<cut.crack_width<<",\"crack_angle\":[";
 for(int i=0;i<cut.cracks;++i)out<<(i?",":"")<<cut.crack_angle[i];out<<"],\"edge\":[";
 for(unsigned d=0;d<2;++d){out<<(d?",[":"[");for(unsigned i=0;i<128;++i){const double a=tau*i/128;out<<(i?",":"")<<cut.edge.radiusMultiplier(std::cos(a),std::sin(a),double(d));}out<<']';}out<<"],\"angles\":[";
 for(std::size_t i=0;i<result.a.size();++i)out<<(i?",":"")<<result.a[i];out<<"],\"depths\":[";
 for(std::size_t i=0;i<result.t.size();++i)out<<(i?",":"")<<result.t[i];
 out<<"],\"admitted\":"<<(result.admitted?"true":"false")<<",\"raw_triangles\":"<<result.stats.triangles<<",\"checks\":"<<result.stats.checks<<",\"plan_ms\":"<<result.stats.ms<<",\"sampled_error\":"<<result.stats.error<<"}\n";
 // Planned vertices and triangles are diagnostic only, never returned to the
 // live worker output. Fine logical masks and production display are intact.
}
}
