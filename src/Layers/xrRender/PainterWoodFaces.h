#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
namespace PainterWoodFaces
{
struct Point { float x,y,z; };
struct Face { int id; unsigned vertex[3]; Point p[3]; };
struct Patch { int first=-1; std::vector<Point> vertices;std::vector<int> edge[6]; };
inline bool connected(const Face& a,const Face& b) {
    unsigned shared=0;
    for(auto av:a.vertex) for(auto bv:b.vertex) if(av==bv) ++shared;
    return shared>=2;
}
inline float coord(Point p,int axis) {return axis==0?p.x:axis==1?p.y:p.z;}
inline Point interpolate(Point a,Point b,float t) {
    return {a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t,a.z+(b.z-a.z)*t};
}
inline unsigned clipPlane(const Point* input,unsigned n,Point* out,int a,float plane,float sign) {
    unsigned k=0;
    for(unsigned i=0;i<n;i++) {
        Point p=input[i?i-1:n-1], q=input[i];
        float d=(coord(p,a)-plane)*sign, e=(coord(q,a)-plane)*sign;
        if((d<0)!=(e<0)) out[k++]=interpolate(p,q,d/(d-e));
        if(e>=0) out[k++]=q;
    }
    return k;
}
// The seed is the exact collision face returned by the ignition/spread query.
// Positional neighbours sharing a corner or merely overlapping a bounding box do not connect.
inline Patch build(const std::vector<Face>& faces,int seed,Point center,float halfSize) {
    Patch result;
    if(faces.size()>192) return result;
    std::vector<unsigned> path;std::vector<unsigned char> visited(faces.size(),0);
    for(unsigned i=0;i<faces.size();++i) if(faces[i].id==seed){path.push_back(i);visited[i]=1;break;}
    for(unsigned step=0;step<path.size();++step) {
        const Face& f=faces[path[step]];
        result.first=result.first<0?f.id:std::min(result.first,f.id);
        for(unsigned j=0;j<faces.size();++j) if(!visited[j] && connected(f,faces[j])) {
            visited[j]=1;path.push_back(j);
        }
        Point tmp[2][16];std::copy(f.p,f.p+3,tmp[0]);unsigned n=3;
        for(int axis=0;axis<3 && n;axis++) {
            float m=coord(center,axis);
            n=clipPlane(tmp[0],n,tmp[1],axis,m-halfSize,1.f);
            n=clipPlane(tmp[1],n,tmp[0],axis,m+halfSize,-1.f);
        }
        if(n>=3)for(int axis=0;axis<3;axis++){
            const float c=coord(center,axis);
            for(int side=0;side<2;side++){
                const float direction=side?1.f:-1.f;
                bool crosses=false;
                for(auto& v:f.p)crosses|=(coord(v,axis)-c)*direction>halfSize+.0001f;
                if(crosses)result.edge[2*axis+side].push_back(f.id);
            }
        }
        for(unsigned j=2;j<n;j++){
            result.vertices.push_back(tmp[0][0]);result.vertices.push_back(tmp[0][j-1]);result.vertices.push_back(tmp[0][j]);
        }
    }
    return result;
}
}
