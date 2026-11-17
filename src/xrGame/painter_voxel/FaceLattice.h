#pragma once
namespace fastwound {
struct LatticeSpec {Vec origin;double pitch{};}; // tile planes at origin[a]+8*i*pitch
enum class LineKind:std::uint8_t {edge,plane,inner,side};
struct LineId {LineKind kind;std::int8_t a{};std::int32_t i{};};
// A polygon vertex; `line` is the line of the edge from this vertex to the next.
// weightRule: 0 corner (canonical position weights), 1 original edge point
// (canonical edge interpolation), 2 face-internal (barycentric interpolation).
struct LatticeVertex {core::V v;LineId line;std::uint8_t weightRule{2};std::uint32_t lo{},hi{};double t{};bool crossing{};
    // Tile planes this vertex lies on (its canonical position is snapped to the
    // 1e-8 lattice; exactPlanes() restores the exact plane coordinates).
    std::int8_t planeAxis[2]{-1,-1};std::int32_t planeIndex[2]{};};
struct LatticePiece {Cell tile;std::vector<LatticeVertex> poly;bool strip{};int edge{-1}; /* strip: its protected edge */};
struct FaceFrame;
// parts: inner triangle, then per edge its trapezoid (strip when protected;
// a strip carries every lattice crossing of its inner/side lines).
struct FaceLattice {std::vector<LatticePiece> parts;std::shared_ptr<const FaceFrame> frame;};

// A vertex within planeTol of a tile plane counts as ON it (both sides keep it,
// no crossing is made beside it): a plane passing a mesh vertex by nanometres
// would otherwise leave sliver pieces whose near-coincident vertices fold.
inline constexpr double planeTol=2e-8;
inline double tilePlane(const LatticeSpec& l,int axis,int i){return l.origin[axis]+double(i)*8*l.pitch;}
inline Vec snapped(Vec p){return core::snap(p);}

// Canonical geometry of one face (A,B,C in the face's own corner order).
struct FaceFrame {
    const core::Source* s{};std::uint32_t face{};LatticeSpec lattice;
    Vec c[3];core::V corner[3];core::V inner[3];bool hasInner=false;
    std::uint32_t cornerVertex[3]{};
    // Canonical point where line `line` meets tile plane (axis,i).
    LatticeVertex onLine(LineId line,int axis,int i)const{
        const double X=tilePlane(lattice,axis,i);LatticeVertex out;out.v.material=0;
        auto segment=[&](const core::V& p0,const core::V& p1,int rule,std::uint32_t id0,std::uint32_t id1){
            // Endpoints in a canonical order so both neighbours of an edge agree.
            const bool swap=core::quantExact(p1.p)<core::quantExact(p0.p);const core::V& lo=swap?p1:p0;const core::V& hi=swap?p0:p1;
            const double t=(X-lo.p[axis])/(hi.p[axis]-lo.p[axis]);Vec p=lo.p+(hi.p-lo.p)*t;p[axis]=X;
            out.v.p=snapped(p);for(int k=0;k<3;++k)out.v.b[k]=lo.b[k]+(hi.b[k]-lo.b[k])*t;
            out.weightRule=std::uint8_t(rule);out.lo=swap?id1:id0;out.hi=swap?id0:id1;out.t=t;};
        switch(line.kind){
        case LineKind::edge:{const int e=line.a;segment(corner[e],corner[(e+1)%3],1,cornerVertex[e],cornerVertex[(e+1)%3]);break;}
        case LineKind::inner:{const int e=line.a;segment(inner[e],inner[(e+1)%3],2,0,0);break;}
        case LineKind::side:{const int k=line.a;segment(corner[k],inner[k],2,0,0);break;}
        case LineKind::plane:{ // face plane x plane(line.a,line.i) x plane(axis,i)
            int a0=line.a,a1=axis;double X0=tilePlane(lattice,line.a,line.i),X1=X;if(a1<a0){std::swap(a0,a1);std::swap(X0,X1);}
            const Vec u=c[1]-c[0],w=c[2]-c[0];const double m00=u[a0],m01=w[a0],m10=u[a1],m11=w[a1],r0=X0-c[0][a0],r1=X1-c[0][a1];
            const double det=m00*m11-m01*m10;double s=0,t=0;if(std::abs(det)>1e-30){s=(r0*m11-m01*r1)/det;t=(m00*r1-r0*m10)/det;}
            Vec p=c[0]+u*s+w*t;p[a0]=X0;p[a1]=X1;out.v.p=snapped(p);
            // Barycentrics of the final (forced, snapped) position by least
            // squares in the face plane: consistent with the position even when
            // the two-plane solve is ill-conditioned.
            {const Vec d=out.v.p-c[0];const double uu=dot(u,u),uw=dot(u,w),ww=dot(w,w),du=dot(d,u),dw=dot(d,w),den=uu*ww-uw*uw;
             if(std::abs(den)>1e-300){s=(ww*du-uw*dw)/den;t=(uu*dw-uw*du)/den;}}
            out.v.b={1-s-t,s,t};out.weightRule=2;break;}}
        out.crossing=true;out.planeAxis[0]=std::int8_t(axis);out.planeIndex[0]=i;
        if(line.kind==LineKind::plane){out.planeAxis[1]=line.a;out.planeIndex[1]=line.i;}
        return out;
    }
};

// Split a convex polygon by the tile plane (axis,i); both sides returned.
// Edge lines: an edge that is (part of) input edge k keeps line k; any new
// edge closing a side lies on the splitting plane.
inline void splitByPlane(const FaceFrame& frame,const std::vector<LatticeVertex>& poly,int axis,int i,std::vector<LatticeVertex>& below,std::vector<LatticeVertex>& above){
    const double X=tilePlane(frame.lattice,axis,i);const LineId planeLine{LineKind::plane,std::int8_t(axis),i};
    auto side=[&](const Vec& p){const double d=p[axis]-X;return d>planeTol?1:(d< -planeTol?-1:0);};
    const std::size_t n=poly.size();std::vector<int> sides(n);for(std::size_t k=0;k<n;++k)sides[k]=side(poly[k].v.p);
    std::vector<LatticeVertex> cuts(n);std::vector<char> hasCut(n,0);
    for(std::size_t k=0;k<n;++k){const int su=sides[k],sw=sides[(k+1)%n];if(su*sw<0){cuts[k]=frame.onLine(poly[k].line,axis,i);hasCut[k]=1;}}
    for(int sign:{-1,1}){auto& out=sign<0?below:above;out.clear();
        struct Tag {int orig=-1,cut=-1;};std::vector<Tag> tags;std::vector<LatticeVertex> verts;
        for(std::size_t k=0;k<n;++k){if(sides[k]*sign>=0){verts.push_back(poly[k]);tags.push_back({int(k),-1});}
            if(hasCut[k]){verts.push_back(cuts[k]);tags.push_back({-1,int(k)});}}
        // Drop consecutive duplicates (a cut indistinguishable from a neighbour).
        std::vector<LatticeVertex> v2;std::vector<Tag> t2;
        for(std::size_t k=0;k<verts.size();++k){if(!v2.empty()&&core::quantExact(v2.back().v.p)==core::quantExact(verts[k].v.p)){if(t2.back().orig<0)t2.back()=tags[k],v2.back()=verts[k];continue;}v2.push_back(verts[k]);t2.push_back(tags[k]);}
        while(v2.size()>1&&core::quantExact(v2.front().v.p)==core::quantExact(v2.back().v.p)){if(t2.front().orig<0){t2.front()=t2.back();v2.front()=v2.back();}v2.pop_back();t2.pop_back();}
        const std::size_t m=v2.size();if(m<3)continue;
        for(std::size_t k=0;k<m;++k){const Tag a=t2[k],b=t2[(k+1)%m];LineId line=planeLine;
            if(a.orig>=0&&b.orig>=0&&std::size_t(b.orig)==(std::size_t(a.orig)+1)%n)line=poly[a.orig].line;
            else if(a.orig>=0&&b.cut==a.orig)line=poly[a.orig].line;
            else if(a.cut>=0&&b.orig>=0&&std::size_t(b.orig)==(std::size_t(a.cut)+1)%n)line=poly[a.cut].line;
            v2[k].line=line;}
        core::Poly tmp;for(auto&v:v2)tmp.push_back(v.v);if(core::area(tmp)<1e-16)continue;
        out=std::move(v2);}
}

inline int tileIndex(const LatticeSpec& l,int axis,double x){return int(std::floor((x-l.origin[axis])/(8*l.pitch)));}
// Position with exact tile-plane coordinates (for clipping by cell boxes whose
// faces lie on those planes: no nanometre cuts beside the snapped vertex).
// Any coordinate within planeTol of a tile plane (a vertex the split treats as
// ON that plane) is moved onto it as well.
inline Vec exactPlanes(const LatticeSpec& l,const LatticeVertex& v){Vec p=v.v.p;for(int k=0;k<2;++k)if(v.planeAxis[k]>=0)p[v.planeAxis[k]]=tilePlane(l,v.planeAxis[k],v.planeIndex[k]);
    for(int a=0;a<3;++a){const int i=int(std::llround((p[a]-l.origin[a])/(8*l.pitch)));const double X=tilePlane(l,a,i);if(std::abs(p[a]-X)<=planeTol)p[a]=X;}return p;}
inline Cell tileOfPoint(const LatticeSpec& l,Vec p){return Cell{tileIndex(l,0,p.x),tileIndex(l,1,p.y),tileIndex(l,2,p.z)};}

// Pieces of a convex part inside the tile range [lo,hi] (inclusive): the part is
// cut along the tile planes bounding that range and every plane inside it, in
// the fixed order (axis 0 ascending, then 1, then 2). Canonical cut vertices
// make each piece independent of the range it was produced for.
inline void latticeSplitRange(const FaceFrame& frame,std::vector<LatticeVertex> poly,Cell lo,Cell hi,std::vector<LatticePiece>& out){
    std::vector<std::vector<LatticeVertex>> work{std::move(poly)},next;std::vector<LatticeVertex> below,above;
    for(int axis=0;axis<3;++axis){next.clear();const int tlo=(&lo.x)[axis],thi=(&hi.x)[axis];
        for(auto& p:work){if(p.empty())continue;double mn=1e30,mx=-1e30;for(auto&v:p){mn=std::min(mn,v.v.p[axis]);mx=std::max(mx,v.v.p[axis]);}
            const int first=tileIndex(frame.lattice,axis,mn),last=tileIndex(frame.lattice,axis,mx);
            if(last<tlo||first>thi)continue;
            std::vector<LatticeVertex> rest=std::move(p);bool restInside=true;
            for(int i=first+1;i<=last&&!rest.empty();++i){if(i<tlo||i>thi+1)continue;const double X=tilePlane(frame.lattice,axis,i);if(X<=mn+planeTol||X>=mx-planeTol)continue;
                splitByPlane(frame,rest,axis,i,below,above);
                if(!below.empty()&&i>tlo)next.push_back(below); // below plane tlo: outside the range
                rest=above;if(i==thi+1)restInside=false;}
            if(!rest.empty()&&restInside)next.push_back(std::move(rest));}
        work.swap(next);}
    for(auto& p:work){if(p.empty())continue;Vec c;for(auto&v:p)c=c+v.v.p;c=c*(1./double(p.size()));
        const Cell t=tileOfPoint(frame.lattice,c);
        if(t.x<lo.x||t.x>hi.x||t.y<lo.y||t.y>hi.y||t.z<lo.z||t.z>hi.z)continue;
        LatticePiece piece;piece.tile=t;piece.poly=std::move(p);out.push_back(std::move(piece));}
}

// Every canonical crossing of line `line` (from p0 to p1) with tile planes,
// ordered from p0 to p1, excluding points equal to the endpoints.
inline std::vector<LatticeVertex> lineCrossings(const FaceFrame& frame,LineId line,const core::V& p0,const core::V& p1){
    std::vector<std::pair<double,LatticeVertex>> hits;const Vec d=p1.p-p0.p;const double dd=dot(d,d);
    const auto q0=core::quantExact(p0.p),q1=core::quantExact(p1.p);
    for(int axis=0;axis<3;++axis){const double lo=std::min(p0.p[axis],p1.p[axis]),hi=std::max(p0.p[axis],p1.p[axis]);
        for(int i=tileIndex(frame.lattice,axis,lo)+1;i<=tileIndex(frame.lattice,axis,hi);++i){const double X=tilePlane(frame.lattice,axis,i);if(X<=lo+planeTol||X>=hi-planeTol)continue;
            auto v=frame.onLine(line,axis,i);const auto q=core::quantExact(v.v.p);if(q==q0||q==q1)continue;hits.push_back({dot(v.v.p-p0.p,d)/dd,v});}}
    std::sort(hits.begin(),hits.end(),[](const auto&a,const auto&b){return a.first<b.first;});
    std::vector<LatticeVertex> out;for(auto&h:hits)if(out.empty()||!(core::quantExact(out.back().v.p)==core::quantExact(h.second.v.p)))out.push_back(h.second);
    return out;
}

// protectMask bit e: edge (corner e -> e+1) keeps its original geometry.
inline FaceLattice buildFaceLattice(const core::Source& s,std::uint32_t face,const LatticeSpec& lattice,unsigned protectMask){
    FaceFrame frame;frame.s=&s;frame.face=face;frame.lattice=lattice;
    for(int k=0;k<3;++k){frame.cornerVertex[k]=s.f[face][k];frame.c[k]=s.v[s.f[face][k]].p;frame.corner[k].p=frame.c[k];frame.corner[k].b={};frame.corner[k].b[k]=1;frame.corner[k].material=0;}
    FaceLattice out;
    auto cornerVertex=[&](int k){LatticeVertex v;v.v=frame.corner[k];v.weightRule=0;v.lo=frame.cornerVertex[k];return v;};
    if(!protectMask){// No protected edge: the whole triangle is the only part (no
        // face-internal lines at all).
        LatticePiece tri;for(int k=0;k<3;++k){auto v=cornerVertex(k);v.line={LineKind::edge,std::int8_t(k),0};tri.poly.push_back(v);}
        out.parts.push_back(std::move(tri));out.frame=std::make_shared<FaceFrame>(frame);return out;}
    // The inner triangle and the three edge trapezoids exist for every mask, so
    // protecting or releasing one edge changes only that edge's trapezoid.
    // Homothety toward the centroid: strip width lambda*h_e <= pitch/4.
    const Vec g=(frame.c[0]+frame.c[1]+frame.c[2])*(1./3);double hmax=0;
    for(int e=0;e<3;++e){const Vec a=frame.c[e],b=frame.c[(e+1)%3];const Vec d=b-a;const double dd=dot(d,d);if(dd<=0)continue;const double t=dot(g-a,d)/dd;hmax=std::max(hmax,length(g-(a+d*t)));}
    const double lambda=hmax>0?std::min(.25,.25*lattice.pitch/hmax):.25;frame.hasInner=true;
    for(int k=0;k<3;++k){frame.inner[k].p=snapped(frame.c[k]+(g-frame.c[k])*lambda);for(int j=0;j<3;++j)frame.inner[k].b[j]=(j==k?1-lambda:0)+lambda/3;frame.inner[k].material=0;}
    auto innerVertex=[&](int k,LineId line){LatticeVertex v;v.v=frame.inner[k];v.weightRule=2;v.line=line;return v;};
    {LatticePiece tri;for(int k=0;k<3;++k)tri.poly.push_back(innerVertex(k,{LineKind::inner,std::int8_t(k),0}));out.parts.push_back(std::move(tri));}
    for(int e=0;e<3;++e){const int f=(e+1)%3;
        std::vector<LatticeVertex> trap;auto ce=cornerVertex(e);ce.line={LineKind::edge,std::int8_t(e),0};auto cf=cornerVertex(f);cf.line={LineKind::side,std::int8_t(f),0};
        trap.push_back(ce);trap.push_back(cf);trap.push_back(innerVertex(f,{LineKind::inner,std::int8_t(e),0}));trap.push_back(innerVertex(e,{LineKind::side,std::int8_t(e),0}));
        if(!(protectMask>>e&1u)){LatticePiece piece;piece.poly=std::move(trap);piece.edge=e;out.parts.push_back(std::move(piece));continue;}
        // Protected strip: kept whole, with every lattice crossing of its three
        // inner boundary lines (shared with the split neighbours), never of the edge.
        std::vector<LatticeVertex> poly;poly.push_back(trap[0]);poly.push_back(trap[1]);
        for(auto v:lineCrossings(frame,{LineKind::side,std::int8_t(f),0},frame.corner[f],frame.inner[f])){v.line={LineKind::side,std::int8_t(f),0};poly.push_back(v);}
        poly.push_back(trap[2]);
        for(auto v:lineCrossings(frame,{LineKind::inner,std::int8_t(e),0},frame.inner[f],frame.inner[e])){v.line={LineKind::inner,std::int8_t(e),0};poly.push_back(v);}
        poly.push_back(trap[3]);
        for(auto v:lineCrossings(frame,{LineKind::side,std::int8_t(e),0},frame.inner[e],frame.corner[e])){v.line={LineKind::side,std::int8_t(e),0};poly.push_back(v);}
        LatticePiece piece;piece.poly=std::move(poly);piece.strip=true;piece.edge=e;out.parts.push_back(std::move(piece));
    }
    out.frame=std::make_shared<FaceFrame>(frame);
    return out;
}
}
