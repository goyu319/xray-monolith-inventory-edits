#pragma once
namespace channelworker {
struct EdgeView {
 std::shared_ptr<const gfy_live::ResolvedEdgeProfile> value;
 bool forced{}; // R24: a slab keeps the record's frame (edge_u = plane normal, edge_v = side) with a unit profile
 bool enabled()const{return forced||bool(value);}
 double radiusMultiplier(double x,double y,double depth=0)const{
  if(!value)return 1.;double angle=std::atan2(y,x);if(angle<0)angle+=channelbirth::tau;
  const double coordinate=angle*(128/channelbirth::tau);const unsigned i=std::min(unsigned(coordinate),127u);const double f=coordinate-i;
  const auto&a=value->radius[0];const auto&b=value->radius[1];double first=a[i]+(a[i+1]-a[i])*f,last=b[i]+(b[i+1]-b[i])*f;return first+(last-first)*std::clamp(depth,0.,1.);
 }
};
struct ResolvedView {
 core::Vec entry,direction,edge_u,edge_v;double radius,penetration,cap,inlet,exit_distance,exit_mult,cav_amp,cav_t0,cav_t1,burst_amp,burst_len,crack_width;
 int cracks;std::array<double,8>crack_angle{},crack_u{},crack_v{};double crack_cos2{};EdgeView edge;
 bool slab{};double slab_w{};gfy_live::RaggedBlade ragged; // R24: oriented box, half thickness = radius, half width = slab_w
 explicit ResolvedView(const gfy_live::ResolvedTunnel&c):entry{c.entry.x,c.entry.y,c.entry.z},direction{c.direction.x,c.direction.y,c.direction.z},edge_u{c.edge_u.x,c.edge_u.y,c.edge_u.z},edge_v{c.edge_v.x,c.edge_v.y,c.edge_v.z},radius(c.radius),penetration(c.penetration),cap(c.cap),inlet(c.inlet),exit_distance(c.exit_distance),exit_mult(c.exit_mult),cav_amp(c.cav_amp),cav_t0(c.cav_t0),cav_t1(c.cav_t1),burst_amp(c.burst_amp),burst_len(c.burst_len),crack_width(c.crack_width),cracks(int(c.cracks)),crack_angle(c.crack_angle),edge{c.edge}{
  if(c.slab){ragged=gfy_live::RaggedBlade(c.slab_seed,c.radius,c.slab_half_width,c.penetration);slab=true;slab_w=c.slab_half_width;edge.forced=true;if(!(std::isfinite(slab_w)&&slab_w>0)||c.cracks||!(c.exit_distance>=0))throw std::runtime_error("channel_resolved_slab");}
  const double scalars[]={entry.x,entry.y,entry.z,direction.x,direction.y,direction.z,radius,penetration,cap,inlet,exit_distance,exit_mult,cav_amp,cav_t0,cav_t1,burst_amp,burst_len,crack_width};for(double x:scalars)if(!std::isfinite(x))throw std::runtime_error("channel_resolved_nonfinite");
  if(!(radius>0&&penetration>0&&cap>0&&cap<=penetration&&inlet>=0&&exit_mult>=1&&cav_amp>=0&&burst_amp>=0&&cracks>=0&&cracks<=8))throw std::runtime_error("channel_resolved_range");
  if(std::abs(core::dot(direction,direction)-1)>1e-9)throw std::runtime_error("channel_resolved_direction");
  if(cav_amp>0&&!(cav_t1>cav_t0))throw std::runtime_error("channel_resolved_swell");if(cracks&&!(exit_distance>=0&&burst_len>0&&crack_width>0))throw std::runtime_error("channel_resolved_cracks");
  if(!edge.enabled()&&!cracks){const core::Vec ref=std::abs(direction.x)<.8?core::Vec{1,0,0}:core::Vec{0,1,0};edge_u=unit(core::cross(direction,ref));edge_v=core::cross(direction,edge_u);}
  if(std::abs(core::dot(edge_u,edge_u)-1)>1e-9||std::abs(core::dot(edge_v,edge_v)-1)>1e-9||std::abs(core::dot(direction,edge_u))>1e-9||std::abs(core::dot(direction,edge_v))>1e-9||std::abs(core::dot(edge_u,edge_v))>1e-9||std::abs(core::dot(core::cross(edge_u,edge_v),direction)-1)>1e-9)throw std::runtime_error("channel_resolved_frame");
  if(edge.value)for(const auto&ring:edge.value->radius){if(ring[0]!=ring[128])throw std::runtime_error("channel_resolved_periodic");for(double x:ring)if(!(x>=.65&&x<=1))throw std::runtime_error("channel_resolved_profile");}
  for(int i=0;i<cracks;++i){if(!std::isfinite(crack_angle[i]))throw std::runtime_error("channel_resolved_angle");crack_u[i]=std::cos(crack_angle[i]);crack_v[i]=std::sin(crack_angle[i]);}crack_cos2=std::pow(std::cos(crack_width),2);
 }
 // R60: publish the torn sheet's exact faces: the lattice triangles of the upper and lower faces (the same alternating
 // diagonals as RaggedBlade::disp), the inlet strip before the entry (planar, the entry row repeated), flat side walls
 // at s = -+width and end caps at both depth ends; outward winding as the former ring sheet.
 bool planarSheet(channelbirth::Plan&out)const{
  if(!ragged.active)return false;
  const int ns=ragged.ns;const double up=ragged.upperFace,down=ragged.lowerFace;
  std::vector<double> ts;if(inlet>0)ts.push_back(-inlet);for(int i=0;i<=ragged.nt;++i)ts.push_back(i*ragged.dt);
  const int rows=int(ts.size()),cols=ns+1,first=inlet>0?1:0;
  auto nodeD=[&](int r,int j){return ragged.node(std::max(0,r-first),j);};
  for(int face=0;face<2;++face)for(int r=0;r<rows;++r)for(int j=0;j<cols;++j){const double w=-slab_w+j*ragged.ds,d=nodeD(r,j);
   out.vertices.push_back(entry+direction*ts[r]+edge_u*(face==0?d+up:d-down)+edge_v*w);}
  auto U=[&](int r,int j){return std::uint32_t(r*cols+j);};auto L=[&](int r,int j){return std::uint32_t(rows*cols+r*cols+j);};
  auto emit=[&](std::uint32_t a,std::uint32_t b,std::uint32_t c,const core::Vec&outward){
   const core::Vec n=core::cross(out.vertices[b]-out.vertices[a],out.vertices[c]-out.vertices[a]);if(core::dot(n,outward)<0)std::swap(b,c);
   out.triangles.push_back({a,b,c});};
  const core::Vec upN=edge_u,downN=edge_u*-1.,leftN=edge_v*-1.,rightN=edge_v,backN=direction*-1.,frontN=direction;
  for(int r=0;r+1<rows;++r)for(int j=0;j+1<cols;++j){
   const bool even=r<first||gfy_live::RaggedBlade::evenSplit(r-first,j); // the inlet strip is planar: any diagonal is exact
   for(int face=0;face<2;++face){auto V=[&](int a,int b){return face==0?U(a,b):L(a,b);};const core::Vec&o=face==0?upN:downN;
    if(even){emit(V(r,j),V(r+1,j),V(r+1,j+1),o);emit(V(r,j),V(r+1,j+1),V(r,j+1),o);}
    else{emit(V(r,j),V(r+1,j),V(r,j+1),o);emit(V(r+1,j+1),V(r,j+1),V(r+1,j),o);}}}
  for(int r=0;r+1<rows;++r){emit(U(r,0),U(r+1,0),L(r+1,0),leftN);emit(U(r,0),L(r+1,0),L(r,0),leftN);
   emit(U(r,ns),U(r+1,ns),L(r+1,ns),rightN);emit(U(r,ns),L(r+1,ns),L(r,ns),rightN);}
  const int e=rows-1;
  for(int j=0;j+1<cols;++j){emit(U(0,j),U(0,j+1),L(0,j+1),backN);emit(U(0,j),L(0,j+1),L(0,j),backN);
   emit(U(e,j),U(e,j+1),L(e,j+1),frontN);emit(U(e,j),L(e,j+1),L(e,j),frontN);}
  out.t=ts;out.stats.angles=2*cols;out.stats.rings=ts.size();out.stats.triangles=out.triangles.size();out.stats.error=0.;out.admitted=true;return true;
 }
 bool sheetBounds(double c,double d,channelbirth::Box&box)const{
  if(!slab)return false;
  const auto bounds=ragged.active?ragged.bounds(c,d,-slab_w,slab_w):gfy_live::RaggedBlade::Bounds{-radius,-radius,radius,radius};
  for(int k=0;k<3;++k){const double a=direction[k]*c,b=direction[k]*d;
   const double x=edge_u[k]*bounds.lowerMin,y=edge_u[k]*bounds.upperMax,w=std::abs(edge_v[k])*slab_w;
   box.lo[k]=entry[k]+std::min(a,b)+std::min(x,y)-w-1e-9;
   box.hi[k]=entry[k]+std::max(a,b)+std::max(x,y)+w+1e-9;}
  return true;
 }
 double swell(double t)const{if(cav_amp<=0||t<=cav_t0||t>=cav_t1)return 0;const double s=std::sin(3.141592653589793*(t-cav_t0)/(cav_t1-cav_t0));return cav_amp*s*s;}
 double flareAt(double t)const{if(exit_distance<0)return 0;const double flare=std::min(exit_distance,radius*1.5);const double blend=flare>1e-12?std::clamp((t-(exit_distance-flare))/flare,0.,1.):1.;return (exit_mult*(1+burst_amp)-1)*blend;}
 double radiusAt(double t)const{return radius*(1+flareAt(t)+swell(t));}
 double crackScale(double eu,double ev,double t)const{
  // R24: a slab's section is the box rectangle: its polar radius over the half thickness (radius) scales the ring.
  if(slab){if(ragged.active){const double len=std::sqrt(eu*eu+ev*ev);return ragged.radial(len>1e-18?eu/len:1.,len>1e-18?ev/len:0.,t)/radius;}const double len=std::sqrt(eu*eu+ev*ev);const double ax=len>1e-18?std::abs(eu)/len:1.,ay=len>1e-18?std::abs(ev)/len:0.; // R26: any (eu,ev) scale
   const double rx=ax>1e-12?radius/ax:1e300,ry=ay>1e-12?slab_w/ay:1e300;return std::min(rx,ry)/radius;}
  if(cracks<=0||t<exit_distance-burst_len)return 1;const double len2=eu*eu+ev*ev;bool possible=!(len2>1e-24);for(int c=0;c<cracks&&!possible;++c){const double d=eu*crack_u[c]+ev*crack_v[c];possible=d>0&&d*d>=len2*crack_cos2;}if(!possible)return 1;const double ramp=std::clamp((t-(exit_distance-burst_len))/burst_len,0.,1.);const double th=std::atan2(ev,eu);double best=0;for(int c=0;c<cracks;++c){double d=std::fmod(std::abs(th-crack_angle[c]),channelbirth::tau);if(d>channelbirth::tau*.5)d=channelbirth::tau-d;if(d<crack_width)best=std::max(best,1-d/crack_width);}return 1+burst_amp*ramp*ramp*best;}
 // R24: the box corners become wall vertices (a chord across a corner would admit a wedge within tolerance).
 void seedDepths(std::vector<double>&out)const{if(ragged.active)ragged.seedDepths(out);}
 void seedAngles(std::vector<double>&out)const{if(!slab)return;if(ragged.active){ragged.seedAngles(out);return;}const double a=std::atan2(slab_w,radius),pi=channelbirth::tau*.5;out.insert(out.end(),{a,pi-a,pi+a,channelbirth::tau-a});}
};
}
