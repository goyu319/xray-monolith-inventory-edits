#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <vector>

// Classify an actual indexed opaque trunk section. Exact-position welding preserves UV seams.
namespace TreeCardTopology
{
using Point = std::array<float,3>;
using Card = std::array<Point,4>;
using Cards = std::set<Card>;
struct Triangle { unsigned v[3]; };
inline Point sub(const Point& a,const Point& b) { return {a[0]-b[0],a[1]-b[1],a[2]-b[2]}; }
inline float dot(const Point& a,const Point& b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
inline Point cross(const Point& a,const Point& b) { return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]}; }
inline unsigned root(std::vector<unsigned>& parent,unsigned n) {
    while (parent[n]!=n) { parent[n]=parent[parent[n]]; n=parent[n]; } return n;
}
inline bool rectangle(const Triangle& a,const Triangle& b,const std::vector<Point>& points, Card& signature) {
    unsigned shared[2],na=0,rareA=0,rareB=0;
    for (unsigned v:a.v) {
        if(std::find(b.v,b.v+3,v)==b.v+3) rareA=v;
        else { if(na==2)return false; shared[na++]=v; }
    }
    if(na!=2)return false;
    for(unsigned v:b.v) if(v!=shared[0] && v!=shared[1])rareB=v;
    const Point& q=points[shared[0]]; const Point& end=points[shared[1]];
    const Point u=sub(points[rareA],q),v=sub(points[rareB],q),diag=sub(end,q);
    const float uu=dot(u,u),vv=dot(v,v),dd=dot(diag,diag);
    if(uu<=0.f || vv<=0.f || dd<=0.f || (std::min)(uu,vv)<.25f*(std::max)(uu,vv)) return false;
    // The shared edge is the inner diagonal of a nearly rectangular, broad planar sheet.
    const Point residual=sub(diag,{u[0]+v[0],u[1]+v[1],u[2]+v[2]});
    if(dot(residual,residual)>.0016f*dd || std::abs(dot(u,v))>.12f*std::sqrt(uu*vv))return false;
    const Point n1=cross(sub(points[a.v[1]],points[a.v[0]]),sub(points[a.v[2]],points[a.v[0]]));
    const Point n2=cross(sub(points[b.v[1]],points[b.v[0]]),sub(points[b.v[2]],points[b.v[0]]));
    if(dot(n1,n2)<=0.f)return false;
    signature={q,end,points[rareA],points[rareB]}; std::sort(signature.begin(),signature.end());
    return true;
}
// A connected 3D support must survive in the very same material and draw window.
// Plane distance prevents geometrically flat fan/crown/billboard meshes passing via their AABB.
inline bool volumetric(const std::vector<unsigned>& faces,const std::vector<Triangle>& tri,const std::vector<Point>& p) {
    if(faces.size()<12)return false;
    float areaSq=0.f;Point anchor{},normal{},lo=p[tri[faces[0]].v[0]],hi=lo;
    for(unsigned i:faces) {
        const auto& t=tri[i];const Point a=p[t.v[0]];
        const Point n=cross(sub(p[t.v[1]],a),sub(p[t.v[2]],a));const float ar=dot(n,n);
        if(ar>areaSq){areaSq=ar;anchor=a;normal=n;}
        for(auto v:t.v)for(int k=0;k<3;k++){lo[k]=(std::min)(lo[k],p[v][k]);hi[k]=(std::max)(hi[k],p[v][k]);}
    }
    const Point d=sub(hi,lo); const float limit=.025f*.025f*dot(d,d)*areaSq;
    if(areaSq<=0.f || limit<=0.f)return false;
    for(unsigned i:faces)for(auto v:tri[i].v){
        float z=dot(normal,sub(p[v],anchor));if(z*z>limit)return true;
    }
    return false;
}
inline bool classify(const std::vector<Point>& positions,unsigned used,const unsigned short* indices,unsigned count,
                     const Cards* permitted,Cards& found,std::vector<unsigned short>& compact) {
    found.clear();compact.clear();
    if(!indices || used>positions.size() || used<12 || count<42 || count%3)return false;
    std::map<Point,unsigned> ids;
    std::vector<unsigned> parent,vertexMap(used);
    std::vector<Point> p;
    for(unsigned i=0;i<used;i++){
        const auto& pos=positions[i]; for(float f:pos)if(!std::isfinite(f))return false;
        auto result=ids.emplace(pos,(unsigned)ids.size());
        if(result.second){parent.push_back(result.first->second);p.push_back(pos);}
        vertexMap[i]=result.first->second;
    }
    std::vector<Triangle> tri(count/3);
    for(unsigned i=0;i<tri.size();i++){
        for(int k=0;k<3;k++){
            unsigned v=indices[3*i+k]; if(v>=used)return false;
            tri[i].v[k]=vertexMap[v];
        }
        auto& t=tri[i];for(int k=1;k<3;k++){auto a=root(parent,t.v[0]),b=root(parent,t.v[k]);parent[b]=a;}
    }
    std::vector<std::vector<unsigned>> groups(parent.size());
    for(unsigned i=0;i<tri.size();i++)groups[root(parent,tri[i].v[0])].push_back(i);
    bool haveSupport=false;
    std::vector<bool> remove(tri.size(),false);
    unsigned removed=0;
    for(const auto& g:groups) {
        if(!haveSupport && g.size()>=12)haveSupport=volumetric(g,tri,p);
        if(g.size()!=2)continue;
        Card card;
        if(!rectangle(tri[g[0]],tri[g[1]],p,card) || (permitted&&!permitted->count(card)))continue;
        found.insert(card); remove[g[0]]=remove[g[1]]=true; removed+=2;
    }
    // Keep sections assembled predominantly from cards, including impostor LODs.
    if(!haveSupport || !removed || removed*10u>tri.size()){found.clear();return false;}
    compact.reserve(count-3*removed);
    for(unsigned i=0;i<tri.size();i++)if(!remove[i])compact.insert(compact.end(),indices+3*i,indices+3*i+3);
    return true;
}
}
