#pragma once
#include "PainterWrapBounds.h"

namespace PainterWoodFlame
{
struct Face
{
    float nx=0, nz=0, left=0, right=0, y=0, x=0, z=0, slope=0, base=0;
    bool valid() const { return nx*nx+nz*nz>.5f; }
};

// A section that cannot be closed still has a confirmed face and an oriented contact.
// Project its vertices onto the contact tangent. This keeps the fire on the finite board.
template<class Point>
Face face(const Point& contact,const Point& normal,const Point* vertices,float width,float below)
{
    Face out;
    const float horizontal=std::sqrt(normal.x*normal.x+normal.z*normal.z);
    if(horizontal<.001f) return out;
    out.nx=normal.x/horizontal;out.nz=normal.z/horizontal;
    out.x=contact.x;out.y=contact.y;out.z=contact.z;
    out.left=-width*.36f;out.right=width*.36f;
    out.base=contact.y-below;
    if(vertices)
    {
        out.left=0;out.right=0;
        float minY=vertices[0].y;
        for(int i=0;i<3;++i)
        {
            const float tangent=(vertices[i].z-contact.z)*out.nx-(vertices[i].x-contact.x)*out.nz;
            out.left=std::min(out.left,tangent);out.right=std::max(out.right,tangent);
            minY=std::min(minY,vertices[i].y);
        }
        out.left=std::max(-width*.55f,std::min(out.left,-.035f));
        out.right=std::min(width*.55f,std::max(out.right,.035f));
        out.base=std::max(out.base,minY);
        const float ax=vertices[1].x-vertices[0].x,ay=vertices[1].y-vertices[0].y,az=vertices[1].z-vertices[0].z;
        const float bx=vertices[2].x-vertices[0].x,by=vertices[2].y-vertices[0].y,bz=vertices[2].z-vertices[0].z;
        const float horizontalDot=(ay*bz-az*by)*out.nx+(ax*by-ay*bx)*out.nz;
        if(std::abs(horizontalDot)>.00001f)
            out.slope=std::max(-.75f,std::min(.75f,-(az*bx-ax*bz)/horizontalDot));
    }
    return out;
}

inline PainterWrapBounds::Bounds bounds(const Face& f,float lowY,float height,float scale)
{
    const float thick=std::max(.13f,std::min(.34f,.11f+.13f*scale));
    // The same wind and radial support limits used by the volume program.
    const float margin=3.90f*thick;
    float x0=f.x+f.nx*(lowY-f.y)*f.slope,x1=x0+f.nx*height*f.slope;
    float z0=f.z+f.nz*(lowY-f.y)*f.slope,z1=z0+f.nz*height*f.slope;
    const float xl=std::min(-f.nz*f.left,-f.nz*f.right),xh=std::max(-f.nz*f.left,-f.nz*f.right);
    const float zl=std::min(f.nx*f.left,f.nx*f.right),zh=std::max(f.nx*f.left,f.nx*f.right);
    return {std::min(x0,x1)+xl-margin,lowY,std::min(z0,z1)+zl-margin,
            std::abs(x1-x0)+xh-xl+2.f*margin,height,std::abs(z1-z0)+zh-zl+2.f*margin,thick};
}
}

