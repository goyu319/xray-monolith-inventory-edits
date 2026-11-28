#pragma once
#include <cmath>
#include <algorithm>
namespace PainterFireColumn
{
struct Section
{
    float y0=0,x0=0,z0=0,rx0=0,rz0=0,y1=0,x1=0,z1=0,rx1=0,rz1=0;
    bool valid() const {return y1>y0 && rx0>0 && rz0>0 && rx1>0 && rz1>0;}
};
struct Point{float x,z;};
struct Ring{Point center{};float rx=0,rz=0;};
inline float distanceSq(Point a,Point b){float x=a.x-b.x,z=a.z-b.z;return x*x+z*z;}
class Contours
{
    struct Vertex {Point p;unsigned char links[8]={},n=0;};
    static constexpr unsigned limit=96;
    Vertex nodes[limit];unsigned used=0;bool overflow=false;
    unsigned node(Point p) {
        for(unsigned i=0;i<used;++i)
            if(distanceSq(nodes[i].p,p)<=0.000004f)return i; // original seam vertices can differ after interpolation
        if(used==limit){overflow=true;return 0;}
        nodes[used].p=p;return used++;
    }
    void join(Vertex& v,unsigned to){if(v.n==8)overflow=true;else v.links[v.n++]=(unsigned char)to;}
public:
    template<class V> void intersect(const V* tri,float y) {
        if(overflow)return;
        Point ends[2];unsigned n=0;
        for(unsigned i=0;i<3;++i) {
            const V&a=tri[i],&b=tri[(i+1)%3];
            if((a.y<=y && y<b.y)||(b.y<=y && y<a.y)){
                const float t=(y-a.y)/(b.y-a.y);
                ends[n++]={a.x+t*(b.x-a.x),a.z+t*(b.z-a.z)};
            }
        }
        if(n!=2 || distanceSq(ends[0],ends[1])<=0.000004f)return;
        unsigned a=node(ends[0]),b=node(ends[1]);if(overflow||a==b)return;
        const Vertex& v=nodes[a];
        for(unsigned i=0;i<v.n;++i)if(v.links[i]==b)return;
        join(nodes[a],b);join(nodes[b],a);
    }
    bool nearest(Point contact,bool timber,Ring& ring,float maxDistance=.46f,unsigned minTimberCorners=5) const {
        if(overflow)return false;
        bool seen[limit]={};float best=maxDistance*maxDistance;bool found=false;
        for(unsigned start=0;start<used;++start){
            if(seen[start])continue;
            unsigned group[limit],num=1,extreme=start;group[0]=start;seen[start]=true;
            bool simple=true;
            for(unsigned j=0;j<num;++j){
                unsigned id=group[j];const Vertex& v=nodes[id];simple&=v.n==2;
                if(v.p.x>nodes[extreme].p.x || (v.p.x==nodes[extreme].p.x && v.p.z<nodes[extreme].p.z))extreme=id;
                for(unsigned k=0;k<v.n;++k){unsigned to=v.links[k];if(!seen[to]){seen[to]=true;group[num++]=to;}}
            }
            if(num<4 || (timber && !simple))continue;
            if(!simple){
                // Several overlapping bark strips may share seam vertices. Only accept
                // the connected mesh if a real cycle winds around the measured centre.
                // Integrating integer ray crossings along a spanning tree proves it:
                // two paths to one vertex disagree only when their closed walk encloses it.
                float xmin=nodes[start].p.x,xmax=xmin,zmin=nodes[start].p.z,zmax=zmin;
                for(unsigned j=0;j<num;++j){const auto&p=nodes[group[j]].p;
                    xmin=std::min(xmin,p.x);xmax=std::max(xmax,p.x);
                    zmin=std::min(zmin,p.z);zmax=std::max(zmax,p.z);}
                const Point center={(xmin+xmax)*.5f,(zmin+zmax)*.5f};
                const float w=xmax-xmin,d=zmax-zmin;
                if(std::min(w,d)<.072f||std::max(w,d)>1.64f||std::max(w,d)>2.25f*std::min(w,d))continue;
                int winding[limit];std::fill(winding,winding+limit,1000000);
                unsigned queue[limit],count=1;queue[0]=start;winding[start]=0;
                bool enclosed=false;float dist=1.e8f;
                for(unsigned at=0;at<count;++at){unsigned id=queue[at];const auto&v=nodes[id];const auto&p=v.p;
                    for(unsigned k=0;k<v.n;++k){unsigned next=v.links[k];const Point q=nodes[next].p;
                        float dx=q.x-p.x,dz=q.z-p.z,l=dx*dx+dz*dz;
                        float u=std::max(0.f,std::min(1.f,((center.x-p.x)*dx+(center.z-p.z)*dz)/l));
                        if(distanceSq(center,{p.x+u*dx,p.z+u*dz})<.000004f)continue;
                        u=std::max(0.f,std::min(1.f,((contact.x-p.x)*dx+(contact.z-p.z)*dz)/l));
                        dist=std::min(dist,distanceSq(contact,{p.x+u*dx,p.z+u*dz}));
                        int cross=0;
                        if((p.z<=center.z && center.z<q.z)||(q.z<=center.z && center.z<p.z)){
                            if(p.x+(center.z-p.z)*dx/dz>center.x)cross=q.z>p.z?1:-1;
                        }
                        int target=winding[id]+cross;
                        if(winding[next]==1000000){winding[next]=target;queue[count++]=next;}
                        else if(winding[next]!=target)enclosed=true;
                    }
                }
                if(enclosed && dist<best){best=dist;ring={center,.5f*w,.5f*d};found=true;}
                continue;
            }
            unsigned path[limit],id=start,prev=limit;
            for(unsigned j=0;j<num;++j){path[j]=id;const auto&v=nodes[id];
                const unsigned next=v.links[v.links[0]==prev?1:0];prev=id;id=next;}
            if(id!=start)continue;
            float xmin=nodes[path[0]].p.x,xmax=xmin,zmin=nodes[path[0]].p.z,zmax=zmin;
            float area=0,dist=1e8f;unsigned corners=0;
            for(unsigned j=0;j<num;++j){
                Point p=nodes[path[j]].p,q=nodes[path[(j+1)%num]].p,r=nodes[path[(j+num-1)%num]].p;
                xmin=std::min(xmin,p.x);xmax=std::max(xmax,p.x);zmin=std::min(zmin,p.z);zmax=std::max(zmax,p.z);
                const float dx=q.x-p.x,dz=q.z-p.z,ex=p.x-r.x,ez=p.z-r.z,len2=dx*dx+dz*dz;
                const float t=std::max(0.f,std::min(1.f,((contact.x-p.x)*dx+(contact.z-p.z)*dz)/len2));
                dist=std::min(dist,distanceSq(contact,{p.x+t*dx,p.z+t*dz}));
                area+=(p.x-contact.x)*(q.z-contact.z)-(q.x-contact.x)*(p.z-contact.z);
                const float cross=dx*ez-dz*ex;
                if(cross*cross>.025f*len2*(ex*ex+ez*ez))++corners;
            }
            const float w=xmax-xmin,d=zmax-zmin;
            // Ordinary boards and crate corners keep their planar material route.
            if(dist>=best || std::min(w,d)<.072f || std::max(w,d)>1.64f ||
               std::max(w,d)>2.25f*std::min(w,d) || fabsf(area)<1.05f*w*d ||
               (timber && corners<minTimberCorners))continue;
            best=dist;ring={{(xmin+xmax)*.5f,(zmin+zmax)*.5f},w*.5f,d*.5f};found=true;
        }
        return found;
    }
};
inline bool related(const Ring&a,const Ring&b,float dy){
    const float w=std::max(std::max(a.rx,a.rz),std::max(b.rx,b.rz));
    const float allowance=.16f+.48f*fabsf(dy);
    return distanceSq(a.center,b.center)<allowance*allowance && std::min(std::min(a.rx,a.rz),std::min(b.rx,b.rz))>.16f*w;
}
template<class Slice>
bool qualify(float y,Point contact,bool timber,Slice&& slice,Section& out){
    out={};Ring center{},lo{},hi{},continuation{};
    if(!slice(y,contact,timber,center))return false;
    bool lower=slice(y-.42f,center.center,timber,lo)&&related(center,lo,.42f);
    bool upper=slice(y+.42f,center.center,timber,hi)&&related(center,hi,.42f);
    if(!lower&&!upper)return false;
    if(timber){
        // A pole continues well beyond this row; a compact pile or short round prop does not.
        if(!(slice(y+2.05f,center.center,true,continuation)&&related(center,continuation,2.05f)) &&
           !(slice(y-2.05f,center.center,true,continuation)&&related(center,continuation,2.05f)))return false;
    }
    if(!lower)lo=center;if(!upper)hi=center;
    out={y-(lower?.42f:0.f),lo.center.x,lo.center.z,lo.rx,lo.rz,
         y+(upper?.42f:0.f),hi.center.x,hi.center.z,hi.rx,hi.rz};return true;
}
}
