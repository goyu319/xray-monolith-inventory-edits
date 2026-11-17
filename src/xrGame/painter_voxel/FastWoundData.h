#pragma once
// Included by LiveDamageWorker.cpp inside namespace gfy_live, after the core
// kernel and RuntimeMeshPacking.h. Pure CPU data; no engine types.
//
// Per-asset data for the incremental full-body wound path:
//  * the packed ABI row of every ORIGINAL source face corner, computed with
//    exactly the same interpolateVertex()+packing::vertex() calls the old
//    untouched-triangle path used (signed zeros and all), and
//  * the packed output of each whole exterior child when none of its faces is
//    part of any wound patch (a static group of packMulti).
// Nothing here depends on damage, so it is built once during asynchronous
// asset preparation and shared read-only by every owner of the asset.
namespace fastwound {
using core::packing::VertexIndex;
using core::packing::vertexHash;

// One triangle reference for the exact packMulti group loop.
struct TriRef {const gfy_packed::Vertex* row[3];std::uint64_t hash[3];double discarded[3];std::uint32_t original_face;};

// Exact replica of the per-group body of core::packing::packMulti. `next` is the
// shared appended-slot counter; `vertices` is scratch.
// Provisional packing: a child that takes a child slot gets provisionalSlot; the caller assigns the real slots in order.
inline constexpr std::uint16_t provisionalSlot=0xFFFF;
inline void packGroup(gfy_packed::Mesh& out,unsigned& next,VertexIndex& vertices,unsigned material,std::uint16_t source,
                      const std::vector<TriRef>& triangles,bool provisional=false){
    using core::packing::packingRequire;
    auto appendSlot=[&]{if(provisional)return provisionalSlot;packingRequire(next<gfy_packed::max_child_slots,"Runtime child slots exhausted");return std::uint16_t(next++);};
    gfy_packed::Child child;child.material=material;child.material_source_child=source;
    child.child_index=child.material?appendSlot():source;
    vertices.reset(std::min<std::size_t>(3*triangles.size(),65535));
    auto reserve=[&](std::size_t remaining){remaining=std::min<std::size_t>(remaining,65535);child.indices.reserve(3*remaining);child.original_faces.reserve(remaining);};
    reserve(triangles.size());
    auto flush=[&]{packingRequire(provisional||child.child_index<gfy_packed::max_child_slots,"Runtime child slots exhausted");out.children.push_back(std::move(child));};
    for(std::size_t t=0;t<triangles.size();++t){const auto& tri=triangles[t];
        std::array<long,3> found;unsigned fresh=0;
        for(unsigned k=0;k<3;++k){found[k]=vertices.find(child.vertices,*tri.row[k],tri.hash[k]);if(found[k]<0)++fresh;}
        if(child.vertices.size()+fresh>65535||child.indices.size()/3>=65535){flush();child={};child.material=material;child.material_source_child=source;child.child_index=appendSlot();vertices.clear();found.fill(-1);reserve(triangles.size()-t);}
        for(unsigned k=0;k<3;++k){
            if(found[k]>=0){child.indices.push_back(std::uint16_t(found[k]));continue;}
            const auto [index,inserted]=vertices.insert(child.vertices,*tri.row[k],tri.hash[k]);
            if(inserted){out.reduced_skin_vertices+=tri.discarded[k]>1e-12;out.max_discarded_skin_weight=std::max(out.max_discarded_skin_weight,tri.discarded[k]);}
            child.indices.push_back(index);
        }
        child.original_faces.push_back(tri.original_face);
    }
    flush();
}

struct AssetData {
    // Deduplicated rows (exact bytes + exact discarded value) and per corner ids.
    std::vector<gfy_packed::Vertex> rows;std::vector<std::uint64_t> hashes;std::vector<double> discarded;
    std::vector<std::uint32_t> corner; // 3*face+k -> row
    std::vector<std::vector<std::uint32_t>> child_faces; // ascending original faces per child slot
    // Static packMulti output of a whole untouched exterior child. Chunk 0 keeps
    // child_index=child; later split chunks get appended slots at assembly.
    struct StaticChild {std::vector<gfy_packed::Child> chunks;std::size_t reduced{};double max_discarded{};};
    std::vector<StaticChild> static_child; // indexed by child slot
    double build_ms{};std::size_t bytes{};bool valid{true}; // false: fall back to the former union path
    // clean[f]: face f, when not clipped and far from every removed cell, leaves
    // core::join exactly as its untouched packed rows. Requires: non-degenerate
    // after float/quantization, a unique position key, every corner position
    // carrying bit-identical skin weights in all faces that use it, and no
    // T-junction with any other face (no corner strictly inside another edge).
    std::vector<char> clean;std::size_t cleanFaces{};
    // ringFirst/ringFaces (CSR): faces sharing at least one vertex POSITION with
    // face f. A clipped face's slab splits can end anywhere on its edges, so
    // such neighbours may receive conform points and are never passive.
    std::vector<std::uint32_t> ringFirst,ringFaces;
    // Tile-local exterior (FaceLattice.h): canonical skin per source POSITION
    // (first face corner using it; sparse bone/weight entries), faces per
    // position-keyed edge (3*f+k -> neighbours), and edges that carry an
    // authored T-junction or a near edge (never split).
    core::FlatMap<core::Quant,std::uint32_t,core::QuantHash> positionIndex;
    std::vector<std::uint32_t> weightFirst;std::vector<std::pair<std::uint32_t,double>> weightEntries;
    std::vector<std::uint32_t> edgeNeighbourFirst,edgeNeighbours; // CSR over face edges (other faces with the same edge key)
    std::vector<char> edgeBad;
    std::vector<char> positionConsistent; // every face corner at the position carries bit-identical skin
    bool skinConsistent(const core::Quant& key)const{auto it=positionIndex.find(key);return it!=positionIndex.end()&&positionConsistent[it->second];}
    std::array<double,64> positionWeights(const core::Quant& key)const{std::array<double,64> w{};auto it=positionIndex.find(key);if(it==positionIndex.end())return w;
        for(auto e=weightFirst[it->second];e<weightFirst[it->second+1];++e)w[weightEntries[e].first]=weightEntries[e].second;return w;}
    TriRef untouched(std::uint32_t face,const std::vector<std::uint32_t>& faceIndices)const{
        TriRef r;for(int k=0;k<3;++k){auto id=corner[3*face+k];r.row[k]=&rows[id];r.hash[k]=hashes[id];r.discarded[k]=discarded[id];}
        r.original_face=faceIndices[face];return r;
    }
};

inline std::shared_ptr<const AssetData> buildAssetData(const core::Source& source,const std::vector<std::uint16_t>& faceChildren,
                                                        const std::vector<std::uint32_t>& faceIndices,std::uint16_t originalChildren){
    const auto start=core::Clock::now();
    auto data=std::make_shared<AssetData>();auto& d=*data;
    d.corner.resize(3*source.f.size());
    // Rows are compared by bytes AND by the exact discarded value, so a row that
    // is identical in bytes but was reduced differently stays distinct.
    std::unordered_map<std::uint64_t,std::vector<std::uint32_t>> byHash;byHash.reserve(source.v.size()*2);
    try{
    for(std::uint32_t f=0;f<source.f.size();++f)for(int k=0;k<3;++k){
        core::V v;v.p=source.v[source.f[f][k]].p;v.b[k]=1;
        const auto joined=core::interpolateVertex(source,f,v);double discarded=0;
        const auto row=core::packing::vertex(joined,&discarded);const auto hash=vertexHash(row);
        auto& bucket=byHash[hash];std::uint32_t id=~0u;
        for(auto candidate:bucket)if(!std::memcmp(&d.rows[candidate],&row,sizeof(row))&&d.discarded[candidate]==discarded){id=candidate;break;}
        if(id==~0u){id=std::uint32_t(d.rows.size());d.rows.push_back(row);d.hashes.push_back(hash);d.discarded.push_back(discarded);bucket.push_back(id);}
        d.corner[3*f+k]=id;
    }
    }catch(const core::packing::PackingInfluenceError&){d.valid=false;d.build_ms=core::ms(start);return data;}
    core::StageProfile prepStages("asset");prepStages.mark("rows_static_start");
    {// Clean faces (see AssetData::clean).
        const std::size_t nf=source.f.size();d.clean.assign(nf,1);
        auto cornerWeights=[&](std::uint32_t f,int k){core::V v;v.p=source.v[source.f[f][k]].p;v.b[k]=1;return core::weights(source,f,v);};
        core::FlatMap<core::Quant,std::uint32_t,core::QuantHash> firstAt;firstAt.reserve(nf*2);std::vector<char> consistent;std::vector<std::array<double,64>> weightsAt;
        core::FlatMap<std::array<core::Quant,3>,std::uint32_t,core::QuantArrayHash<3>> faceKeys;faceKeys.reserve(nf);std::vector<std::uint32_t> duplicateOf(nf,~0u);
        for(std::uint32_t f=0;f<nf;++f){std::array<core::Quant,3> key;
            for(int k=0;k<3;++k){const Vec p=source.v[source.f[f][k]].p;key[k]=core::quantExact(p);
                const auto w=cornerWeights(f,k);auto [it,fresh]=firstAt.try_emplace(key[k],std::uint32_t(weightsAt.size()));
                if(fresh){weightsAt.push_back(w);consistent.push_back(1);}else if(std::memcmp(weightsAt[it->second].data(),w.data(),sizeof(w)))consistent[it->second]=0;}
            const Vec a=source.v[source.f[f][0]].p,b=source.v[source.f[f][1]].p,c=source.v[source.f[f][2]].p;
            if(key[0]==key[1]||key[1]==key[2]||key[2]==key[0]||length(cross(b-a,c-a))<1e-16)d.clean[f]=0;
            auto sorted=key;std::sort(sorted.begin(),sorted.end());auto [it,fresh]=faceKeys.try_emplace(sorted,f);if(!fresh){d.clean[f]=0;d.clean[it->second]=0;}}
        for(std::uint32_t f=0;f<nf;++f)for(int k=0;k<3;++k)if(!consistent[firstAt.find(core::quantExact(source.v[source.f[f][k]].p))->second])d.clean[f]=0;
        prepStages.mark("clean_keys_weights");
        // T-junctions: any vertex position strictly inside another face's edge
        // (the conform tolerance, 3e-8 m) marks both faces unclean.
        constexpr double cell=.001;const Vec origin{};
        core::FlatMap<Cell,std::vector<std::uint32_t>,core::CellHash> buckets; // vertex position index -> faces via positionFaces
        std::vector<Vec> positions;std::vector<std::vector<std::uint32_t>> positionFaces;core::FlatMap<core::Quant,std::uint32_t,core::QuantHash> positionId;
        for(std::uint32_t f=0;f<nf;++f)for(int k=0;k<3;++k){const Vec p=source.v[source.f[f][k]].p;auto [it,fresh]=positionId.try_emplace(core::quantExact(p),std::uint32_t(positions.size()));
            if(fresh){positions.push_back(p);positionFaces.emplace_back();buckets[core::joinBucket(origin,cell,p)].push_back(it->second);}positionFaces[it->second].push_back(f);}
        // Edge table (id 3*f+k) with endpoints and position keys.
        const std::size_t ne=3*nf;std::vector<Vec> edgeA(ne),edgeB(ne);std::vector<core::Quant> keyA(ne),keyB(ne);
        for(std::uint32_t e=0;e<ne;++e){const auto f=e/3;const int k=int(e%3);edgeA[e]=source.v[source.f[f][k]].p;edgeB[e]=source.v[source.f[f][(k+1)%3]].p;keyA[e]=core::quantExact(edgeA[e]);keyB[e]=core::quantExact(edgeB[e]);}
        auto visitEdgeCellsAt=[&](double size,std::uint32_t e,auto&&visit){core::Bounds bb;bb.add(edgeA[e]);bb.add(edgeB[e]);
            const Cell lo=core::joinBucket(origin,size,bb.lo-Vec{3e-8,3e-8,3e-8}),hi=core::joinBucket(origin,size,bb.hi+Vec{3e-8,3e-8,3e-8});core::edgeCorridorCells(origin,size,edgeA[e],edgeB[e],lo,hi,visit);};
        auto visitEdgeCells=[&](std::uint32_t e,auto&&visit){visitEdgeCellsAt(cell,e,visit);};
        // Each chunk of edges marks unclean faces in its own table (merged below).
        std::vector<std::vector<char>> dirtyMarks(core::parallelWidth(),std::vector<char>(nf,0)),edgeMarks(core::parallelWidth(),std::vector<char>(ne,0));
        core::parallelChunks(ne,4096,[&](std::size_t first,std::size_t last,std::size_t chunk){auto&mark=dirtyMarks[chunk];auto&edgeMark=edgeMarks[chunk];
            for(std::size_t e=first;e<last;++e){const Vec a=edgeA[e],dvec=edgeB[e]-a;const double dd=dot(dvec,dvec);if(dd<1e-20)continue;
                visitEdgeCells(std::uint32_t(e),[&](Cell c){auto it=buckets.find(c);if(it==buckets.end())return;
                    for(auto id:it->second){const Vec p=positions[id];const core::Quant qp=core::quantExact(p);if(qp==keyA[e]||qp==keyB[e])continue;
                        const double t=dot(p-a,dvec)/dd;if(t<=1e-8||t>=1-1e-8||length(p-(a+dvec*t))>3e-8)continue;
                        mark[e/3]=1;edgeMark[e]=1;for(auto g:positionFaces[id])mark[g]=1;}});}});
        prepStages.mark("clean_tjunctions");
        // Non-adjacent edges passing within the conform tolerance of each other
        // (crossing or overlapping sheets) could carry a split point from one
        // face onto the other: both faces are unclean.
        {core::FlatMap<Cell,std::vector<std::uint32_t>,core::CellHash> edgeCells;
            constexpr double edgeCell=.004;
            for(std::uint32_t e=0;e<ne;++e)visitEdgeCellsAt(edgeCell,e,[&](Cell c){edgeCells[c].push_back(e);});
            auto segmentDistance=[](Vec p1,Vec q1,Vec p2,Vec q2){const Vec d1=q1-p1,d2=q2-p2,r=p1-p2;const double a=dot(d1,d1),e=dot(d2,d2),f=dot(d2,r);double s,t;
                if(a<=1e-30&&e<=1e-30)return length(p1-p2);
                if(a<=1e-30){s=0;t=std::clamp(f/e,0.,1.);}else{const double c=dot(d1,r);if(e<=1e-30){t=0;s=std::clamp(-c/a,0.,1.);}
                    else{const double b=dot(d1,d2),den=a*e-b*b;s=den>1e-30?std::clamp((b*f-c*e)/den,0.,1.):0.;t=(b*s+f)/e;
                        if(t<0){t=0;s=std::clamp(-c/a,0.,1.);}else if(t>1){t=1;s=std::clamp((b-c)/a,0.,1.);}}}
                return length((p1+d1*s)-(p2+d2*t));};
            std::vector<core::Bounds> edgeBox(ne);for(std::uint32_t e=0;e<ne;++e){edgeBox[e].add(edgeA[e]);edgeBox[e].add(edgeB[e]);}
            std::vector<std::vector<std::uint32_t>> seen(core::parallelWidth(),std::vector<std::uint32_t>(ne,~0u));
            core::parallelChunks(ne,4096,[&](std::size_t first,std::size_t last,std::size_t chunk){auto&mark=dirtyMarks[chunk];auto&stamp=seen[chunk];auto&edgeMark=edgeMarks[chunk];
                for(std::size_t e=first;e<last;++e){
                    visitEdgeCellsAt(edgeCell,std::uint32_t(e),[&](Cell c){auto it=edgeCells.find(c);if(it==edgeCells.end())return;
                        for(auto o:it->second){if(o<=e||o/3==e/3||stamp[o]==e)continue;stamp[o]=std::uint32_t(e);
                            if(keyA[o]==keyA[e]||keyA[o]==keyB[e]||keyB[o]==keyA[e]||keyB[o]==keyB[e])continue; // identical or vertex-adjacent: ring rule
                            bool apart=false;for(int k=0;k<3;++k)apart=apart||edgeBox[o].lo[k]>edgeBox[e].hi[k]+3e-8||edgeBox[e].lo[k]>edgeBox[o].hi[k]+3e-8;if(apart)continue;
                            if(segmentDistance(edgeA[e],edgeB[e],edgeA[o],edgeB[o])<=3e-8){mark[e/3]=1;mark[o/3]=1;edgeMark[e]=1;edgeMark[o]=1;}}});}});}
        d.edgeBad.assign(ne,0);for(const auto&m:edgeMarks)for(std::size_t e=0;e<ne;++e)if(m[e])d.edgeBad[e]=1;
        {// Edge neighbours by position key.
            core::FlatMap<std::array<core::Quant,2>,std::vector<std::uint32_t>,core::QuantArrayHash<2>> byKey;byKey.reserve(ne);
            for(std::uint32_t e=0;e<ne;++e){auto k=std::array<core::Quant,2>{keyA[e],keyB[e]};if(k[1]<k[0])std::swap(k[0],k[1]);byKey[k].push_back(e/3);}
            d.edgeNeighbourFirst.assign(ne+1,0);std::vector<std::vector<std::uint32_t>> lists(ne);
            for(std::uint32_t e=0;e<ne;++e){auto k=std::array<core::Quant,2>{keyA[e],keyB[e]};if(k[1]<k[0])std::swap(k[0],k[1]);for(auto g:byKey.at(k))if(g!=e/3)lists[e].push_back(g);}
            for(std::uint32_t e=0;e<ne;++e)d.edgeNeighbourFirst[e+1]=d.edgeNeighbourFirst[e]+std::uint32_t(lists[e].size());
            for(auto&l:lists)d.edgeNeighbours.insert(d.edgeNeighbours.end(),l.begin(),l.end());
            // A bad edge is bad for every face using it (same position key).
            for(auto&entry:byKey){bool bad=false;for(auto g:entry.second)for(int k=0;k<3;++k){auto kk=std::array<core::Quant,2>{keyA[3*g+k],keyB[3*g+k]};if(kk[1]<kk[0])std::swap(kk[0],kk[1]);if(kk==entry.first&&d.edgeBad[3*g+k])bad=true;}
                if(bad)for(auto g:entry.second)for(int k=0;k<3;++k){auto kk=std::array<core::Quant,2>{keyA[3*g+k],keyB[3*g+k]};if(kk[1]<kk[0])std::swap(kk[0],kk[1]);if(kk==entry.first)d.edgeBad[3*g+k]=1;}}}
        {// Canonical skin per position (the weights the first face corner there carries).
            d.positionIndex.reserve(firstAt.size());std::vector<std::pair<core::Quant,std::uint32_t>> order(firstAt.begin(),firstAt.end());
            d.weightFirst.assign(order.size()+1,0);d.positionConsistent.assign(order.size(),0);
            for(std::uint32_t n=0;n<order.size();++n){d.positionIndex.try_emplace(order[n].first,n);const auto&w=weightsAt[order[n].second];d.positionConsistent[n]=consistent[order[n].second];
                for(unsigned b=0;b<64;++b){std::uint64_t bits;std::memcpy(&bits,&w[b],8);if(bits)d.weightEntries.emplace_back(b,w[b]);}
                d.weightFirst[n+1]=std::uint32_t(d.weightEntries.size());}}
        for(const auto&mark:dirtyMarks)for(std::size_t f=0;f<nf;++f)if(mark[f])d.clean[f]=0;
        prepStages.mark("clean_edge_proximity");
        // Vertex-position rings.
        {std::vector<std::vector<std::uint32_t>> ring(nf);
            for(const auto&faces:positionFaces)for(auto f:faces)for(auto g:faces)if(f!=g)ring[f].push_back(g);
            d.ringFirst.assign(nf+1,0);for(std::uint32_t f=0;f<nf;++f){auto&r=ring[f];std::sort(r.begin(),r.end());r.erase(std::unique(r.begin(),r.end()),r.end());d.ringFirst[f+1]=d.ringFirst[f]+std::uint32_t(r.size());}
            d.ringFaces.reserve(d.ringFirst[nf]);for(auto&r:ring)d.ringFaces.insert(d.ringFaces.end(),r.begin(),r.end());}
        for(auto c:d.clean)d.cleanFaces+=c!=0;prepStages.mark("rings");
    }
    std::uint16_t maxChild=0;for(auto c:faceChildren)maxChild=std::max(maxChild,c);
    d.child_faces.resize(std::size_t(maxChild)+1);
    for(std::uint32_t f=0;f<faceChildren.size();++f)d.child_faces[faceChildren[f]].push_back(f);
    d.static_child.resize(d.child_faces.size());
    VertexIndex scratch;
    for(std::size_t c=0;c<d.child_faces.size();++c){if(d.child_faces[c].empty())continue;
        std::vector<TriRef> refs;refs.reserve(d.child_faces[c].size());
        for(auto f:d.child_faces[c])refs.push_back(d.untouched(f,faceIndices));
        gfy_packed::Mesh mesh;unsigned next=originalChildren;
        packGroup(mesh,next,scratch,0,std::uint16_t(c),refs);
        d.static_child[c].chunks=std::move(mesh.children);d.static_child[c].reduced=mesh.reduced_skin_vertices;d.static_child[c].max_discarded=mesh.max_discarded_skin_weight;
    }
    d.bytes=d.rows.size()*(sizeof(gfy_packed::Vertex)+16)+d.corner.size()*4+faceChildren.size()*4;
    for(auto&s:d.static_child)for(auto&ch:s.chunks)d.bytes+=ch.vertices.size()*80+ch.indices.size()*2+ch.original_faces.size()*4;
    d.build_ms=core::ms(start);
    return data;
}
}
