#pragma once
// Find a point of the rendered/collision surface in the requested horizontal band.
// At each clipped edge and the centre cross-section, test the closest point in XZ.
// Works on the engine Fvector and the offline replay's matching three-float type.
template<class V>
bool painterFireTriangleContact(const V* vertices, const V& at, float radius, float low, float high, V& hit, float& best)
{
    const float y0 = at.y + low, y1 = at.y + high, ym = (y0+y1)*.5f;
    bool found = false;
    auto interp = [](const V& a,const V& b,float t) -> V {
        return {a.x+(b.x-a.x)*t, a.y+(b.y-a.y)*t, a.z+(b.z-a.z)*t};
    };
    auto clamp01 = [](float v){return v<0.f ? 0.f : (v>1.f ? 1.f : v);};
    auto segment = [&](const V& a,const V& b) {
        const float dx=b.x-a.x,dz=b.z-a.z,den=dx*dx+dz*dz;
        const float t=den>1e-13f ? clamp01(((at.x-a.x)*dx+(at.z-a.z)*dz)/den)
            : (b.y!=a.y ? clamp01((ym-a.y)/(b.y-a.y)) : 0.f);
        const V p=interp(a,b,t);
        const float x=p.x-at.x,z=p.z-at.z,d=x*x+z*z;
        if (d<=radius*radius && (d<best || (found && d==best && fabsf(p.y-ym)<fabsf(hit.y-ym)))) {
            found=true; best=d; hit=p;
        }
    };
    // First pick the desired storey in a vertical face, not its lowest vertex.
    V cuts[3];unsigned n=0;
    for(unsigned i=0;i<3;++i) {
        const V& a=vertices[i];const V& b=vertices[(i+1)%3];
        if ((a.y<=ym && ym<b.y) || (b.y<=ym && ym<a.y))
            cuts[n++]=interp(a,b,(ym-a.y)/(b.y-a.y));
    }
    if(n==2) segment(cuts[0],cuts[1]);
    // Each remaining edge of the clipped triangle is either an original edge
    // within the band, or a segment on one of the two horizontal clip planes.
    for(unsigned plane=0;plane<2;++plane) {
        const float y=plane?y1:y0;
        n=0;
        for(unsigned i=0;i<3;++i) {
            const V& a=vertices[i];const V& b=vertices[(i+1)%3];
            if ((a.y<=y && y<b.y) || (b.y<=y && y<a.y))
                cuts[n++]=interp(a,b,(y-a.y)/(b.y-a.y));
        }
        if(n==2) segment(cuts[0],cuts[1]);
    }
    for(unsigned i=0;i<3;++i) {
        const V& a=vertices[i];const V& b=vertices[(i+1)%3];
        if ((a.y<y0 && b.y<y0)||(a.y>y1 && b.y>y1)) continue;
        if (a.y==b.y) { segment(a,b); continue; }
        const float t0=clamp01((y0-a.y)/(b.y-a.y)),t1=clamp01((y1-a.y)/(b.y-a.y));
        segment(interp(a,b,t0),interp(a,b,t1));
    }
    return found;
}
