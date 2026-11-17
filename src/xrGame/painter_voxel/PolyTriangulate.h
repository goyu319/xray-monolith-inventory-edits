#pragma once
// Triangulation of a planar region given by its directed boundary edges
// (region on the LEFT of every edge), for the tile-local static exterior.
//
//  * every boundary vertex is used and no vertex is added, so a vertex shared
//    with a neighbouring piece (a lattice crossing) is never a T-junction;
//  * holes (bridged), pinch vertices and collinear boundary vertices allowed;
//  * each triangle keeps the boundary orientation (CCW in the 2D frame);
//  * validated: triangle area sum == region area (relative 1e-9), every
//    triangle strictly positive (height > tol). Failure -> error string, no
//    output; the caller must not emit a partial region.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace polytri {
struct P2 {double x{},y{};};
inline double orient(P2 a,P2 b,P2 c){return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);}
inline double dist(P2 a,P2 b){return std::hypot(b.x-a.x,b.y-a.y);}
using Tri=std::array<std::uint32_t,3>;
using DEdge=std::pair<std::uint32_t,std::uint32_t>;

namespace detail {
// Signed distance of c from the line a->b (positive: left).
inline double height(P2 a,P2 b,P2 c){const double l=dist(a,b);return l>0?orient(a,b,c)/l:dist(a,c);}
// Segments p0p1 and q0q1 intersect (closed, with tolerance) -- conservative.
inline bool segmentsTouch(P2 p0,P2 p1,P2 q0,P2 q1,double tol){
    const double d1=height(p0,p1,q0),d2=height(p0,p1,q1),d3=height(q0,q1,p0),d4=height(q0,q1,p1);
    if(((d1>tol&&d2<-tol)||(d1<-tol&&d2>tol))&&((d3>tol&&d4<-tol)||(d3<-tol&&d4>tol)))return true; // proper crossing
    auto onSeg=[&](P2 a,P2 b,P2 c){ // c within tol of segment ab
        const double dx=b.x-a.x,dy=b.y-a.y,dd=dx*dx+dy*dy;double t=dd>0?((c.x-a.x)*dx+(c.y-a.y)*dy)/dd:0;t=std::clamp(t,0.,1.);
        return std::hypot(a.x+dx*t-c.x,a.y+dy*t-c.y)<=tol;};
    return onSeg(p0,p1,q0)||onSeg(p0,p1,q1)||onSeg(q0,q1,p0)||onSeg(q0,q1,p1);
}
// Is direction q (a point) strictly inside the region wedge at vertex o with
// boundary prev p -> o -> next n (region on the left)?
inline bool inWedge(P2 p,P2 o,P2 n,P2 q){
    const bool convex=orient(p,o,n)>0;const bool l1=orient(p,o,q)>0,l2=orient(o,n,q)>0;
    return convex?(l1&&l2):(l1||l2);
}
}

// pts: 2D positions by vertex id; keysEqual(a,b): same position identity.
// edges: the region's directed boundary edges. tol: distance tolerance (m).
inline bool triangulate(const std::vector<P2>& pts,const std::vector<DEdge>& edges,double tol,std::vector<Tri>& out,std::string& error){
    using namespace detail;
    out.clear();error.clear();if(edges.empty())return true;
    // 1. Loops: at a vertex with several outgoing edges take the smallest
    // clockwise turn from the reversed incoming edge (stays in one lobe).
    std::unordered_map<std::uint32_t,std::vector<std::uint32_t>> outgoing;outgoing.reserve(edges.size()*2);
    for(std::uint32_t e=0;e<edges.size();++e)outgoing[edges[e].first].push_back(e);
    std::vector<char> used(edges.size(),0);std::vector<std::vector<std::uint32_t>> loops;
    for(std::uint32_t start=0;start<edges.size();++start){if(used[start])continue;
        std::vector<std::uint32_t> loop;std::uint32_t e=start;used[e]=1;
        for(std::size_t guard=0;;++guard){if(guard>edges.size()+1){error="loop_walk";return false;}
            loop.push_back(edges[e].first);const std::uint32_t u=edges[e].first,v=edges[e].second;
            const P2 back{pts[u].x-pts[v].x,pts[u].y-pts[v].y};
            std::int64_t best=-1;double bestAngle=1e30;
            auto consider=[&](std::uint32_t c){const auto w=edges[c].second;const P2 d{pts[w].x-pts[v].x,pts[w].y-pts[v].y};
                double a=-std::atan2(back.x*d.y-back.y*d.x,back.x*d.x+back.y*d.y); // clockwise angle
                if(a<=0)a+=2*3.141592653589793;if(a<bestAngle){bestAngle=a;best=c;}};
            auto found=outgoing.find(v);if(found!=outgoing.end())for(auto c:found->second)if(!used[c]||c==start)consider(c);
            if(best<0){error="open_boundary";return false;}
            if(std::uint32_t(best)==start)break;
            e=std::uint32_t(best);used[e]=1;}
        if(loop.size()<3){error="short_loop";return false;}
        loops.push_back(std::move(loop));}
    auto loopArea=[&](const std::vector<std::uint32_t>& l){double a=0;for(std::size_t k=0;k<l.size();++k){const P2 p=pts[l[k]],q=pts[l[(k+1)%l.size()]];a+=p.x*q.y-q.x*p.y;}return a*.5;};
    std::vector<double> area(loops.size());double total=0;
    for(std::size_t k=0;k<loops.size();++k){area[k]=loopArea(loops[k]);total+=area[k];}
    if(!(total>0)){error="nonpositive_area";return false;}
    // 2. Holes -> smallest containing outer loop (even-odd test on one hole
    // vertex not shared with the outer loop).
    auto inside=[&](const std::vector<std::uint32_t>& l,P2 q){bool in=false;for(std::size_t k=0,j=l.size()-1;k<l.size();j=k++){const P2 a=pts[l[k]],b=pts[l[j]];
            if((a.y>q.y)!=(b.y>q.y)&&q.x<(b.x-a.x)*(q.y-a.y)/(b.y-a.y)+a.x)in=!in;}return in;};
    std::vector<std::vector<std::size_t>> holesOf(loops.size());
    for(std::size_t h=0;h<loops.size();++h){if(area[h]>0)continue;if(area[h]==0){error="zero_loop";return false;}
        std::int64_t owner=-1;double ownerArea=1e300;
        for(std::size_t o=0;o<loops.size();++o){if(area[o]<=0)continue;
            bool test=false,in=false;
            for(auto v:loops[h]){bool shared=false;for(auto w:loops[o])if(w==v){shared=true;break;}if(shared)continue;
                bool onEdge=false;for(std::size_t k=0;k<loops[o].size()&&!onEdge;++k)onEdge=segmentsTouch(pts[loops[o][k]],pts[loops[o][(k+1)%loops[o].size()]],pts[v],pts[v],tol);
                if(onEdge)continue;test=true;in=inside(loops[o],pts[v]);break;}
            if(test&&in&&area[o]<ownerArea){ownerArea=area[o];owner=std::int64_t(o);}}
        if(owner<0){error="orphan_hole";return false;}
        holesOf[std::size_t(owner)].push_back(h);}
    // 3. Per outer loop: bridge holes (rightmost hole first), then clip ears.
    for(std::size_t o=0;o<loops.size();++o){if(area[o]<=0)continue;
        std::vector<std::uint32_t> poly=loops[o];
        auto& holes=holesOf[o];
        auto maxX=[&](std::size_t h){double m=-1e300;for(auto v:loops[h])m=std::max(m,pts[v].x);return m;};
        std::sort(holes.begin(),holes.end(),[&](std::size_t a,std::size_t b){return maxX(a)>maxX(b);});
        for(std::size_t hi=0;hi<holes.size();++hi){const auto& hole=loops[holes[hi]];
            // Candidate (hole vertex, polygon vertex occurrence) by distance.
            struct Cand {double d;std::size_t hk,pk;};std::vector<Cand> cands;
            for(std::size_t hk=0;hk<hole.size();++hk)for(std::size_t pk=0;pk<poly.size();++pk)cands.push_back({dist(pts[hole[hk]],pts[poly[pk]]),hk,pk});
            std::sort(cands.begin(),cands.end(),[](const Cand& a,const Cand& b){return a.d<b.d;});
            bool bridged=false;
            for(const auto& c:cands){const auto hv=hole[c.hk],pv=poly[c.pk];const P2 a=pts[hv],b=pts[pv];if(c.d<=tol)continue;
                // Locally inside at both ends.
                if(!inWedge(pts[poly[(c.pk+poly.size()-1)%poly.size()]],b,pts[poly[(c.pk+1)%poly.size()]],a))continue;
                if(!inWedge(pts[hole[(c.hk+hole.size()-1)%hole.size()]],a,pts[hole[(c.hk+1)%hole.size()]],b))continue;
                // No boundary (polygon, this and remaining holes) touched except at the ends.
                auto clear=[&](const std::vector<std::uint32_t>& l){for(std::size_t k=0;k<l.size();++k){const auto x=l[k],y=l[(k+1)%l.size()];
                        if(x==hv||y==hv||x==pv||y==pv)continue;if(segmentsTouch(a,b,pts[x],pts[y],tol))return false;}return true;};
                bool ok=clear(poly);for(std::size_t r=hi;r<holes.size()&&ok;++r)ok=clear(loops[holes[r]]);
                // Nor any vertex other than the ends on the bridge.
                auto noVertex=[&](const std::vector<std::uint32_t>& l){for(auto x:l){if(x==hv||x==pv)continue;if(segmentsTouch(a,b,pts[x],pts[x],tol))return false;}return true;};
                ok=ok&&noVertex(poly);for(std::size_t r=hi;r<holes.size()&&ok;++r)ok=noVertex(loops[holes[r]]);
                if(!ok)continue;
                std::vector<std::uint32_t> merged;merged.reserve(poly.size()+hole.size()+2);
                for(std::size_t k=0;k<=c.pk;++k)merged.push_back(poly[k]);
                for(std::size_t k=0;k<=hole.size();++k)merged.push_back(hole[(c.hk+k)%hole.size()]);
                for(std::size_t k=c.pk;k<poly.size();++k)merged.push_back(poly[k]);
                poly=std::move(merged);bridged=true;break;}
            if(!bridged){error="no_bridge";return false;}}
        // Ear clipping on the (weakly simple) polygon.
        const std::size_t n=poly.size();std::vector<std::size_t> prev(n),next(n);std::vector<char> alive(n,1);
        for(std::size_t k=0;k<n;++k){prev[k]=(k+n-1)%n;next[k]=(k+1)%n;}
        std::size_t count=n,cur=0,stall=0;
        auto P=[&](std::size_t k){return pts[poly[k]];};
        // eps: the distance tolerance; relaxed to 0 (strict orientation) only
        // when no ear exists at tol (near-degenerate slivers).
        double eps=tol;
        auto convex=[&](std::size_t b){const P2 pa=P(prev[b]),pb=P(b),pc=P(next[b]);return height(pa,pb,pc)>eps&&height(pb,pc,pa)>eps&&height(pc,pa,pb)>eps;};
        auto earOk=[&](std::size_t b)->bool{const std::size_t a=prev[b],c=next[b];
            if(!convex(b))return false;const P2 pa=P(a),pb=P(b),pc=P(c);
            const auto ia=poly[a],ib=poly[b],ic=poly[c];
            // Diagonal a->c locally inside at a and c.
            if(!inWedge(P(prev[a]),pa,pb,pc))return false;
            if(!inWedge(pb,pc,P(next[c]),pa))return false;
            for(std::size_t r=next[c];r!=a;r=next[r]){const auto ir=poly[r];if(ir==ia||ir==ib||ir==ic)continue;const P2 pr=P(r);
                // r inside or within tol of the closed triangle -> not an ear.
                if(height(pa,pb,pr)>-eps&&height(pb,pc,pr)>-eps&&height(pc,pa,pr)>-eps)return false;
                // The diagonal crosses no remaining edge.
                const P2 pn=P(next[r]);const auto in=poly[next[r]];
                if(in!=ia&&in!=ic&&ir!=ia&&ir!=ic&&segmentsTouch(pa,pc,pr,pn,eps))return false;}
            return true;};
        while(count>3){
            const std::size_t a=prev[cur],c=next[cur];
            if(poly[a]==poly[c]){ // spike a->b->a: drop b and the repeated a
                alive[cur]=0;alive[c]=0;next[a]=next[c];prev[next[c]]=a;count-=2;cur=a;stall=0;continue;}
            if(earOk(cur)){out.push_back({poly[a],poly[cur],poly[c]});alive[cur]=0;next[a]=c;prev[c]=a;--count;cur=c;stall=0;continue;}
            cur=next[cur];if(++stall>count){if(eps>0){eps=0;stall=0;continue;}error="no_ear";out.clear();return false;}}
        if(count==3){const std::size_t a=prev[cur],c=next[cur];
            // The remainder: a (possibly sliver) triangle; a zero-area one is
            // dropped (the area validation below still has to hold).
            if(orient(P(a),P(cur),P(c))>0)out.push_back({poly[a],poly[cur],poly[c]});}
    }
    // 4. Validation.
    double sum=0;for(const auto& t:out){const double a=orient(pts[t[0]],pts[t[1]],pts[t[2]])*.5;if(!(a>0)){error="nonpositive_triangle";out.clear();return false;}sum+=a;}
    if(std::abs(sum-total)>1e-9*total+1e-18){error="area_mismatch";out.clear();return false;}
    return true;
}
}
