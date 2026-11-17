#pragma once
// Mask-only wounds, value 3: a head burst's record (headfix, 2026-11-12) - the ONE function both the damage step (HeadBurst.h:
// which cells to erase) and the separation (HeadBurstPieces.h: which piece holds which part) evaluate, and the pixel programs
// (shaders/r3/painter_vmask_head.h) evaluate per sample: the group of a point = the group of the site nearest to point + warp(point)
// (two octaves of seeded vector value noise); beyond Rcut of the blast point: the far part (group 0).
// The record is a gfy_packed::TearBlock: params word 29 = MAGIC (a decide-once tear writes 0 there; row 1 word 3 = H = -1: a
// decide-once reader finds no region), header (26 x 4 words, the pixel programs' row): 0-2 blast point B, 3 Rcut | 4 A1, 5 1/L1,
// 6 A2, 7 1/L2 | 8 ripR, 9 torn depth scale, 10 seed, 11 S | 13 groups, 14 version 1, 15 the piece's groups (bit g; the separation
// fills it per piece) | 24, 25: the sites' groups (4 bits each, site i at word 24 + i / 8, bits 4 (i % 8)) | 26: a group this piece
// shares with other pieces + 1 (0 none) | n others << 8, 28-30 this piece's centroid of that group, 32-34 / 36-38 / 40-42 the
// others' (the separation: the group is cut on the planes between them); 22 a far chunk's sphere, 23 bit 0 = the body (its split
// with the body: the chunk keeps the far part only inside its sphere) | 44 + 4 i: site i (x, y,
// z), i < 15. The atlas writes the .w word of texels 4 .. 11 (19, 27, ... 47) of every entry: never used here. section: sample points (x, y, z,
// group) inside each group, 2 cells from any other group (the separation counts them per piece) - the first header[12] floats -
// then the crack cells the damage step erased, one float each (its index in the labelling box: first cell header words 16-18,
// size x, y words 20, 21; exact below 2^24): the separation gives them back to the pieces' removal fields (the torn faces are the
// record's, never the cells' steps).
// No standard header is included here (the including file has them): it may be included inside a namespace.
namespace head_burst_record {
constexpr std::uint32_t MAGIC=0x50415254u;
constexpr unsigned maxSites=15; // header words 44 .. 103
inline bool is(const gfy_packed::TearBlock&b){return b.params.size()>29&&b.params[29]==MAGIC;}
inline std::uint32_t h32(std::uint32_t x){x^=x>>16;x*=0x7feb352du;x^=x>>15;x*=0x846ca68bu;x^=x>>16;return x;}
inline double lat(std::uint32_t s,int x,int y,int z){
 return double(h32(s^h32(std::uint32_t(x)*0x9e3779b1u+h32(std::uint32_t(y)*0x85ebca6bu+h32(std::uint32_t(z)+0x27d4eb2fu)))))*(2./4294967296.)-1.;}
inline double smooth(double f){return f*f*(3-2*f);}
// One octave of one noise over a box: the lattice values cached (exactly lat() at the same integers), trilinear + smoothstep as
// painter_vh_noise; a point outside the cached box is evaluated directly (the same number).
struct Octave {
 std::uint32_t s=0;double k=1;int lo[3]{},n[3]{};std::vector<double> v;
 void cache(std::uint32_t seed,double scale,const double bmin[3],const double bmax[3]){s=seed;k=scale;std::size_t total=1;
  for(int a=0;a<3;++a){lo[a]=int(std::floor(bmin[a]*k))-1;n[a]=int(std::floor(bmax[a]*k))+2-lo[a]+1;total*=std::size_t(n[a]);}
  if(total>(std::size_t(1)<<22)){v.clear();return;}
  v.resize(total);for(int z=0;z<n[2];++z)for(int y=0;y<n[1];++y)for(int x=0;x<n[0];++x)v[(std::size_t(z)*n[1]+std::size_t(y))*n[0]+std::size_t(x)]=lat(s,lo[0]+x,lo[1]+y,lo[2]+z);}
 // the range of this noise over an axis box: value noise is a convex blend of its 8 lattice corners, so it lies between the
 // smallest and largest lattice value of the lattice cells the box touches
 void range(const double bmin[3],const double bmax[3],double&lo_,double&hi_)const{lo_=1e30;hi_=-1e30;
  int a0[3],a1[3];for(int a=0;a<3;++a){a0[a]=int(std::floor(bmin[a]*k));a1[a]=int(std::floor(bmax[a]*k))+1;}
  for(int z=a0[2];z<=a1[2];++z)for(int y=a0[1];y<=a1[1];++y)for(int x=a0[0];x<=a1[0];++x){const int cx=x-lo[0],cy=y-lo[1],cz=z-lo[2];
   const double val=(!v.empty()&&cx>=0&&cy>=0&&cz>=0&&cx<n[0]&&cy<n[1]&&cz<n[2])?v[(std::size_t(cz)*n[1]+std::size_t(cy))*n[0]+std::size_t(cx)]:lat(s,x,y,z);
   lo_=std::min(lo_,val);hi_=std::max(hi_,val);}
  // tighter for a box much smaller than the lattice: the value at its centre +- the largest slope (1.5 k |lattice step| per axis)
  // times the half extent (a path along the axes stays inside the touched lattice cells)
  double slope[3]={0,0,0};
  for(int z=a0[2];z<=a1[2];++z)for(int y=a0[1];y<=a1[1];++y)for(int x=a0[0];x<=a1[0];++x){
   auto val=[&](int X,int Y,int Z){const int cx=X-lo[0],cy=Y-lo[1],cz=Z-lo[2];
    return (!v.empty()&&cx>=0&&cy>=0&&cz>=0&&cx<n[0]&&cy<n[1]&&cz<n[2])?v[(std::size_t(cz)*n[1]+std::size_t(cy))*n[0]+std::size_t(cx)]:lat(s,X,Y,Z);};
   const double c0=val(x,y,z);if(x<a1[0])slope[0]=std::max(slope[0],std::abs(val(x+1,y,z)-c0));
   if(y<a1[1])slope[1]=std::max(slope[1],std::abs(val(x,y+1,z)-c0));if(z<a1[2])slope[2]=std::max(slope[2],std::abs(val(x,y,z+1)-c0));}
  const double c=at(.5*(bmin[0]+bmax[0]),.5*(bmin[1]+bmax[1]),.5*(bmin[2]+bmax[2]));double spread=0;
  for(int a=0;a<3;++a)spread+=1.5*k*slope[a]*.5*(bmax[a]-bmin[a]);
  lo_=std::max(lo_,c-spread);hi_=std::min(hi_,c+spread);}
 static int ffloor(double x){int i=int(x);return x<double(i)?i-1:i;}
 double at(double px,double py,double pz)const{const double x=px*k,y=py*k,z=pz*k;
  const int X=ffloor(x),Y=ffloor(y),Z=ffloor(z);const double fx0=X,fy0=Y,fz0=Z;
  const double u=smooth(x-fx0),w=smooth(y-fy0),t=smooth(z-fz0);
  const int cx=X-lo[0],cy=Y-lo[1],cz=Z-lo[2];
  double c[8];
  if(!v.empty()&&cx>=0&&cy>=0&&cz>=0&&cx+1<n[0]&&cy+1<n[1]&&cz+1<n[2]){const std::size_t i0=(std::size_t(cz)*n[1]+std::size_t(cy))*n[0]+std::size_t(cx),sy=std::size_t(n[0]),sz=std::size_t(n[0])*n[1];
   c[0]=v[i0];c[1]=v[i0+1];c[2]=v[i0+sy];c[3]=v[i0+sy+1];c[4]=v[i0+sz];c[5]=v[i0+sz+1];c[6]=v[i0+sz+sy];c[7]=v[i0+sz+sy+1];}
  else{c[0]=lat(s,X,Y,Z);c[1]=lat(s,X+1,Y,Z);c[2]=lat(s,X,Y+1,Z);c[3]=lat(s,X+1,Y+1,Z);c[4]=lat(s,X,Y,Z+1);c[5]=lat(s,X+1,Y,Z+1);c[6]=lat(s,X,Y+1,Z+1);c[7]=lat(s,X+1,Y+1,Z+1);}
  const double x00=c[0]+(c[1]-c[0])*u,x10=c[2]+(c[3]-c[2])*u,x01=c[4]+(c[5]-c[4])*u,x11=c[6]+(c[7]-c[6])*u;
  const double y0=x00+(x10-x00)*w,y1=x01+(x11-x01)*w;return y0+(y1-y0)*t;}
};
struct Function {
 double B[3]{},Rcut=0,A1=0,k1=1,A2=0,k2=1,ripR=0,ds=1;std::uint32_t seed=0;unsigned S=0,groups=1;double site[maxSites][3]{};unsigned group[maxSites]{};
 Octave o[6];
 // the lattice of both octaves over the cut's box (the label then needs no hash)
 void cache(){const double bmin[3]={B[0]-Rcut,B[1]-Rcut,B[2]-Rcut},bmax[3]={B[0]+Rcut,B[1]+Rcut,B[2]+Rcut};
  static const std::uint32_t salt[6]={0x11u,0x22u,0x33u,0x44u,0x55u,0x66u};for(int i=0;i<6;++i)o[i].cache(seed^salt[i],i<3?k1:k2,bmin,bmax);}
 // the three components of one octave at once (the same lattice cell and weights; the same numbers as Octave::at)
 static void at3(const Octave*oc,double px,double py,double pz,double r[3]){const double k=oc[0].k,x=px*k,y=py*k,z=pz*k;
  const int X=Octave::ffloor(x),Y=Octave::ffloor(y),Z=Octave::ffloor(z);const double u=smooth(x-X),w=smooth(y-Y),t=smooth(z-Z);
  const int cx=X-oc[0].lo[0],cy=Y-oc[0].lo[1],cz=Z-oc[0].lo[2];const int*n=oc[0].n;
  const bool in=!oc[0].v.empty()&&cx>=0&&cy>=0&&cz>=0&&cx+1<n[0]&&cy+1<n[1]&&cz+1<n[2];
  for(int c3=0;c3<3;++c3){const Octave&o_=oc[c3];double c[8];
   if(in){const std::size_t i0=(std::size_t(cz)*n[1]+std::size_t(cy))*n[0]+std::size_t(cx),sy=std::size_t(n[0]),sz=std::size_t(n[0])*n[1];const double*v=o_.v.data();
    c[0]=v[i0];c[1]=v[i0+1];c[2]=v[i0+sy];c[3]=v[i0+sy+1];c[4]=v[i0+sz];c[5]=v[i0+sz+1];c[6]=v[i0+sz+sy];c[7]=v[i0+sz+sy+1];}
   else{const std::uint32_t s=o_.s;c[0]=lat(s,X,Y,Z);c[1]=lat(s,X+1,Y,Z);c[2]=lat(s,X,Y+1,Z);c[3]=lat(s,X+1,Y+1,Z);c[4]=lat(s,X,Y,Z+1);c[5]=lat(s,X+1,Y,Z+1);c[6]=lat(s,X,Y+1,Z+1);c[7]=lat(s,X+1,Y+1,Z+1);}
   const double x00=c[0]+(c[1]-c[0])*u,x10=c[2]+(c[3]-c[2])*u,x01=c[4]+(c[5]-c[4])*u,x11=c[6]+(c[7]-c[6])*u;
   const double y0=x00+(x10-x00)*w,y1=x01+(x11-x01)*w;r[c3]=y0+(y1-y0)*t;}}
 void warped(double px,double py,double pz,double x[3])const{double a[3],b[3];at3(o,px,py,pz,a);at3(o+3,px,py,pz,b);
  x[0]=px+A1*a[0]+A2*b[0];x[1]=py+A1*a[1]+A2*b[1];x[2]=pz+A1*a[2]+A2*b[2];}
 // the group of every point of an axis box when they all share one (a proof from the warp's range over the box), else 255
 unsigned boxGroup(const double bmin[3],const double bmax[3])const{
  double c2=0,f2=0;for(int a=0;a<3;++a){const double lo_=std::max(bmin[a]-B[a],0.)+std::max(B[a]-bmax[a],0.);c2+=lo_*lo_;const double hi_=std::max(std::abs(bmin[a]-B[a]),std::abs(bmax[a]-B[a]));f2+=hi_*hi_;}
  if(c2>Rcut*Rcut)return 0; // all beyond the cut
  double xmin[3],xmax[3];
  for(int a=0;a<3;++a){double l1,h1,l2,h2;o[a].range(bmin,bmax,l1,h1);o[3+a].range(bmin,bmax,l2,h2);xmin[a]=bmin[a]+A1*l1+A2*l2;xmax[a]=bmax[a]+A1*h1+A2*h2;}
  double dmin[maxSites],dmax[maxSites];
  for(unsigned i=0;i<S;++i){double a2=0,b2=0;for(int a=0;a<3;++a){const double s=site[i][a];const double lo_=std::max(xmin[a]-s,0.)+std::max(s-xmax[a],0.);a2+=lo_*lo_;
    const double hi_=std::max(std::abs(xmin[a]-s),std::abs(xmax[a]-s));b2+=hi_*hi_;}dmin[i]=a2;dmax[i]=b2;}
  for(unsigned i=0;i<S;++i){bool sure=true;for(unsigned j=0;j<S&&sure;++j)if(group[j]!=group[i]&&dmin[j]<=dmax[i])sure=false;
   if(sure){const unsigned g=group[i];if(f2>Rcut*Rcut&&g!=0)return 255;return g;}}
  return 255;}
 unsigned label(double px,double py,double pz)const{const double dx=px-B[0],dy=py-B[1],dz=pz-B[2];if(dx*dx+dy*dy+dz*dz>Rcut*Rcut)return 0;
  double x[3];warped(px,py,pz,x);double best=1e30;unsigned g=0;
  for(unsigned i=0;i<S;++i){const double a=x[0]-site[i][0],b=x[1]-site[i][1],c=x[2]-site[i][2],dd=a*a+b*b+c*c;if(dd<best){best=dd;g=group[i];}}return g;}
 // the group and the margin: the distance (warped space) from the warped point to the nearest boundary with another group
 unsigned labelMargin(double px,double py,double pz,double&margin)const{const double dx=px-B[0],dy=py-B[1],dz=pz-B[2];
  if(dx*dx+dy*dy+dz*dz>Rcut*Rcut){margin=0;return 0;}
  double x[3];warped(px,py,pz,x);double d2[maxSites];double best=1e30;unsigned bi=0;
  for(unsigned i=0;i<S;++i){const double a=x[0]-site[i][0],b=x[1]-site[i][1],c=x[2]-site[i][2];d2[i]=a*a+b*b+c*c;if(d2[i]<best){best=d2[i];bi=i;}}
  margin=1e30;for(unsigned j=0;j<S;++j){if(group[j]==group[bi])continue;const double a=site[j][0]-site[bi][0],b=site[j][1]-site[bi][1],c=site[j][2]-site[bi][2];
   const double l=std::sqrt(a*a+b*b+c*c);if(l>1e-9)margin=std::min(margin,(d2[j]-best)/(2*l));}
  return group[bi];}
};
inline float fl(const std::vector<std::uint32_t>&h,std::size_t i){float f;std::memcpy(&f,&h[i],4);return f;}
// the function a record holds (false: not a record)
inline bool decode(const gfy_packed::TearBlock&b,Function&f){if(!is(b)||b.header.size()<104)return false;const auto&h=b.header;
 for(int k=0;k<3;++k)f.B[k]=fl(h,std::size_t(k));f.Rcut=fl(h,3);f.A1=fl(h,4);f.k1=fl(h,5);f.A2=fl(h,6);f.k2=fl(h,7);f.ripR=fl(h,8);f.ds=fl(h,9);f.seed=h[10];
 f.S=std::min<unsigned>(h[11],maxSites);f.groups=h[13];
 for(unsigned i=0;i<f.S;++i){for(int k=0;k<3;++k)f.site[i][k]=fl(h,44+4*i+std::size_t(k));f.group[i]=(h[24+i/8]>>(4*(i%8)))&15u;}
 f.cache();return true;}
// the sample points' float count (the crack list follows them)
inline std::size_t sampleFloats(const gfy_packed::TearBlock&b){return b.header.size()>21?std::min<std::size_t>(b.section.size(),b.header[12])&~std::size_t(3):b.section.size()&~std::size_t(3);}
// the crack cells (grid cell coordinates) of a record, each passed to f(x, y, z)
template<class F> inline void eachCrackCell(const gfy_packed::TearBlock&b,F&&f){if(b.header.size()<22||!(b.header[20]&0x7fffffffu)||!b.header[21])return;
 const auto&h=b.header;const long long ox=std::int32_t(h[16]),oy=std::int32_t(h[17]),oz=std::int32_t(h[18]),nx=h[20]&0x7fffffffu,ny=h[21];const bool wide=(h[20]>>31)!=0;
 // (a wide box - a body burst's: two floats per cell, index >> 12 then index & 4095)
 for(std::size_t i=sampleFloats(b);i<b.section.size();i+=wide?2:1){if(wide&&i+1>=b.section.size())break;
  const long long v=wide?((long long)b.section[i]<<12)+(long long)b.section[i+1]:(long long)b.section[i];f(int(ox+v%nx),int(oy+(v/nx)%ny),int(oz+v/(nx*ny)));}}
} // namespace head_burst_record
