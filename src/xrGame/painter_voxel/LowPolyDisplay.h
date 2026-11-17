#pragma once
namespace lowpoly {
struct Settings {
    bool enabled{true};
    double trisPerCm2{.14};          // wall budget per cm2 of voxel wall (head blowout ~1800 cm2 -> ~250)
    double minTris{80},maxTris{600}; // wall budget clamp per region
    double errorFraction{.2};        // error limit = this x the rim loops' (length-weighted) radius ...
    double minError{.0005},maxError{.009}; // ... clamped, absolute metres
    float uvWeight{.05f};            // skin UV preservation against position error
    bool rimPriority{true};          // rim vertices get meshopt's priority flag
    double seamSpacing{.006};        // chunked regions: spacing of kept wall vertices along a seam line (m)
    double minFlipArea{1e-8};        // m2; smaller reduced skin slivers are not treated as flips
    // Depth tiers: wall triangles whose corners all lie deeper than max(deepStart, 1.5 x radius) below the skin
    // reduce first with a coarser limit (deepErrorFraction x radius, clamped) and deepDensity x the wall density.
    // Entry and exit walls lie within the band (an exit is skin too) and keep the normal detail.
    // The deep error limit never exceeds 3/4 of the band depth (a deep vertex cannot reach the skin). Applied where
    // the triangle count is final: merged display blocks and unchunked regions (a chunk's output is re-reduced in its block).
    // Measured 2026-09-25 (pkm16/slug/common, 1 mm): no triangle saving (the deep wall already reduces to the error
    // limit) for ~+20% reduction CPU, so off by default (GFY_LOWPOLY_DEEP=1 enables).
    bool deepEnabled{false};double deepStart{.008},deepBandFraction{.5},deepErrorFraction{1.},deepMaxError{.02},deepDensity{.3};
};
inline Settings loadSettings(){Settings s;
    if(const char* v=std::getenv("GFY_LOWPOLY"))s.enabled=std::atoi(v)!=0;
    if(const char* v=std::getenv("GFY_LOWPOLY_DENSITY"))s.trisPerCm2=std::atof(v);
    if(const char* v=std::getenv("GFY_LOWPOLY_MIN"))s.minTris=std::atof(v);
    if(const char* v=std::getenv("GFY_LOWPOLY_MAX"))s.maxTris=std::atof(v);
    if(const char* v=std::getenv("GFY_LOWPOLY_ERROR"))s.maxError=std::atof(v);
    if(const char* v=std::getenv("GFY_LOWPOLY_ERROR_FRACTION"))s.errorFraction=std::atof(v);
    if(const char* v=std::getenv("GFY_LOWPOLY_UVW"))s.uvWeight=float(std::atof(v));
    if(const char* v=std::getenv("GFY_LOWPOLY_RIMPRIORITY"))s.rimPriority=std::atoi(v)!=0;
    if(const char* v=std::getenv("GFY_LOWPOLY_SEAM_SPACING"))s.seamSpacing=std::atof(v);
    if(const char* v=std::getenv("GFY_LOWPOLY_DEEP"))s.deepEnabled=std::atoi(v)!=0;
    if(const char* v=std::getenv("GFY_LOWPOLY_DEEP_START"))s.deepStart=std::atof(v);
    if(const char* v=std::getenv("GFY_LOWPOLY_DEEP_BAND"))s.deepBandFraction=std::atof(v);
    if(const char* v=std::getenv("GFY_LOWPOLY_DEEP_ERROR_FRACTION"))s.deepErrorFraction=std::atof(v);
    if(const char* v=std::getenv("GFY_LOWPOLY_DEEP_DENSITY"))s.deepDensity=std::atof(v);
    return s;}
inline const Settings& settings(){static const Settings s=loadSettings();return s;}

// Depth below the skin: distance to the closed envelope, trilinear over a 1 cm node lattice cached per asset
// (Asset::depth_nodes); a node is one exact BVH nearest query on first use.
struct DepthField {
    const Asset& asset;static constexpr double h=.01;std::unordered_map<std::uint64_t,float> local;
    explicit DepthField(const Asset& a):asset(a){}
    static std::uint64_t key(int x,int y,int z){auto u=[](int v){return std::uint64_t(std::uint32_t(v+(1<<20)))&0x1FFFFFull;};return (u(x)<<42)|(u(y)<<21)|u(z);}
    float node(int x,int y,int z){const auto k=key(x,y,z);auto it=local.find(k);if(it!=local.end())return it->second;
        {std::lock_guard<std::mutex> lock(asset.depth_nodes.mutex);auto g=asset.depth_nodes.map.find(k);if(g!=asset.depth_nodes.map.end())return local.emplace(k,g->second).first->second;}
        double best=1e300;std::uint32_t face=0;Vec q;asset.envelope_tree.nearest(0,Vec{x*h,y*h,z*h},best,face,q);float d=float(std::sqrt(best));
        {std::lock_guard<std::mutex> lock(asset.depth_nodes.mutex);if(asset.depth_nodes.map.size()>=400000)asset.depth_nodes.map.clear();d=asset.depth_nodes.map.try_emplace(k,d).first->second;}
        return local.emplace(k,d).first->second;}
    double sample(const Vec& p){const double fx=p.x/h,fy=p.y/h,fz=p.z/h;const int x=int(std::floor(fx)),y=int(std::floor(fy)),z=int(std::floor(fz));const double tx=fx-x,ty=fy-y,tz=fz-z;
        double d=0;for(int c=0;c<8;++c){const int dx=c&1,dy=(c>>1)&1,dz=(c>>2)&1;const double k=(dx?tx:1-tx)*(dy?ty:1-ty)*(dz?tz:1-tz);if(k>0)d+=k*node(x+dx,y+dy,z+dz);}
        return d;}
};
struct PKey {std::uint32_t x,y,z,tag;bool operator==(const PKey& o)const{return x==o.x&&y==o.y&&z==o.z&&tag==o.tag;}};
struct PKeyHash {std::size_t operator()(const PKey& k)const{
    std::uint64_t h=std::uint64_t(k.x)*0x9E3779B97F4A7C15ull;h^=(h>>29)^(std::uint64_t(k.y)*0xBF58476D1CE4E5B9ull);
    h^=(h>>31)^(std::uint64_t(k.z)*0x94D049BB133111EBull);h^=std::uint64_t(k.tag)*0x2545F4914F6CDD1Dull;return std::size_t(h^(h>>32));}};
constexpr std::uint32_t cavityTag=0x80000000u;
struct U64Hash {std::size_t operator()(std::uint64_t k)const{return std::size_t(k);}};
struct U32Hash {std::size_t operator()(std::uint32_t k)const{return std::size_t(k);}};

// fixedPositions: exact positions (tag 0) of geometry outside this reduction that shares vertices with it.
template<class Region,class AssetT>
void reduceRegion(const AssetT& asset,Region& r,const std::unordered_set<PKey,PKeyHash>* fixedPositions=nullptr){
    const auto& S=settings();if(!S.enabled||!r.packingError.empty()||r.groups.empty())return;
    const auto start=std::chrono::steady_clock::now();
    auto rowPos=[&](std::uint32_t row){const auto& s=r.rows[row].surface;return Vec{s[0],s[1],s[2]};};
    auto pkey=[&](std::uint32_t row,std::uint32_t tag){PKey k;const auto& s=r.rows[row].surface;
        std::memcpy(&k.x,&s[0],4);std::memcpy(&k.y,&s[1],4);std::memcpy(&k.z,&s[2],4);k.tag=tag;return k;};
    // Wedges: exterior = (packed row, source child); wall = exact position. Wall rows at one position differ only
    // in the flat per-face normal/UV (rebuilt here) and the tissue material, which becomes a per-vertex majority
    // (the reduced triangle takes the majority of its corners), so tissue borders never pin the reduction.
    std::vector<std::uint32_t> wRow,wTag;std::vector<unsigned char> wLock;
    core::FlatMap<std::uint64_t,std::uint32_t,U64Hash> extIndex;core::FlatMap<PKey,std::uint32_t,PKeyHash> cavIndex,extAt;
    std::vector<std::pair<std::uint32_t,std::uint32_t>> wedgeFaces; // (exterior wedge, source face)
    std::vector<std::pair<std::uint32_t,unsigned>> wallMaterials; // (wall wedge, material of an incident triangle)
    std::vector<unsigned int> indices;std::size_t inCav=0,fixedExt=0,freeExt=0;double cavArea=0;
    {std::size_t tris=0;for(const auto& g:r.groups)tris+=g.second.size();indices.reserve(tris*3);cavIndex.reserve(tris);extIndex.reserve(tris);extAt.reserve(tris);}
    auto addWedge=[&](std::uint32_t row,std::uint32_t tag){wRow.push_back(row);wTag.push_back(tag);wLock.push_back(0);return std::uint32_t(wRow.size()-1);};
    for(const auto& g:r.groups){const bool cav=g.first.first!=0;
        for(const auto& t:g.second){
            unsigned locked=0;
            if(cav){++inCav;cavArea+=.5*length(cross(rowPos(t.row[1])-rowPos(t.row[0]),rowPos(t.row[2])-rowPos(t.row[0])));}
            for(int k=0;k<3;++k){std::uint32_t w;
                if(cav){auto it=cavIndex.try_emplace(pkey(t.row[k],cavityTag),std::uint32_t(wRow.size()));
                    if(it.second)addWedge(t.row[k],cavityTag);w=it.first->second;wallMaterials.emplace_back(w,g.first.first);}
                else{auto it=extIndex.try_emplace((std::uint64_t(t.row[k])<<16)|g.first.second,std::uint32_t(wRow.size()));
                    if(it.second){addWedge(t.row[k],g.first.second);extAt.try_emplace(pkey(t.row[k],g.first.second),it.first->second);}
                    w=it.first->second;
                    // An original corner of the source face is uncut skin: it never moves.
                    if(t.face<asset.source.f.size()){const auto& s=r.rows[t.row[k]].surface;
                        for(auto id:asset.source.f[t.face]){const Vec& p=asset.source.v[id].p;
                            if(float(p.x)==s[0]&&float(p.y)==s[1]&&float(p.z)==s[2]){wLock[w]=meshopt_SimplifyVertex_Lock;break;}}
                        wedgeFaces.emplace_back(w,t.face);}
                    locked+=wLock[w]!=0;}
                indices.push_back(w);}
            if(!cav)(locked==3?fixedExt:freeExt)++;}}
    if(!inCav)return;
    const std::size_t nw=wRow.size();
    std::vector<unsigned> wMat(nw,0);{std::sort(wallMaterials.begin(),wallMaterials.end());
        for(std::size_t i=0;i<wallMaterials.size();){std::size_t j=i;unsigned best=0;std::size_t bestCount=0;
            while(j<wallMaterials.size()&&wallMaterials[j].first==wallMaterials[i].first){std::size_t k=j;while(k<wallMaterials.size()&&wallMaterials[k]==wallMaterials[j])++k;
                if(k-j>bestCount){bestCount=k-j;best=wallMaterials[j].second;}j=k;}
            wMat[wallMaterials[i].first]=best;i=j;}}
    // Position-welded edges. An exterior edge with no partner (wall or skin) is the border to untouched skin or to
    // a tile-local static piece: lock it. Open wall edges (e.g. where a tunnel crosses the air gap between suit and
    // body) stay free. Rim = edges carried by both skin and wall: measure each rim loop to size the error limit.
    std::vector<std::uint32_t> posOf(nw);{core::FlatMap<PKey,std::uint32_t,PKeyHash> ids;ids.reserve(nw);
        for(std::size_t w=0;w<nw;++w)posOf[w]=ids.try_emplace(pkey(wRow[w],0),std::uint32_t(ids.size())).first->second;}
    core::FlatMap<std::uint64_t,std::array<std::uint32_t,2>,U64Hash> edges;edges.reserve(indices.size());
    auto edgeKey=[&](std::uint32_t a,std::uint32_t b){a=posOf[a];b=posOf[b];if(a>b)std::swap(a,b);return (std::uint64_t(a)<<32)|b;};
    for(std::size_t t=0;t<indices.size();t+=3){const bool cav=(wTag[indices[t]]&cavityTag)!=0;
        for(int k=0;k<3;++k)++edges[edgeKey(indices[t+k],indices[t+(k+1)%3])][cav];}
    std::vector<char> rimPos(nw,0);std::vector<std::uint32_t> parent(nw);for(std::uint32_t i=0;i<nw;++i)parent[i]=i;
    auto root=[&](std::uint32_t i){while(parent[i]!=i){parent[i]=parent[parent[i]];i=parent[i];}return i;};
    std::vector<Vec> posAt(nw);for(std::size_t w=0;w<nw;++w)posAt[posOf[w]]=rowPos(wRow[w]);
    core::FlatMap<std::uint32_t,double,U32Hash> loopLength;
    std::vector<char> fixedPos(nw,0);
    if(fixedPositions)for(std::size_t w=0;w<nw;++w)if(fixedPositions->count(pkey(wRow[w],0))){fixedPos[posOf[w]]=1;wLock[w]=meshopt_SimplifyVertex_Lock;}
    // Rim: an edge carried by skin and wall, or an open wall edge along fixed outside geometry.
    auto isRim=[&](const auto& e){
        return (e.second[0]&&e.second[1])||(!e.second[0]&&e.second[1]==1&&fixedPos[std::uint32_t(e.first>>32)]&&fixedPos[std::uint32_t(e.first&0xffffffffu)]);};
    for(const auto& e:edges){const std::uint32_t a=std::uint32_t(e.first>>32),b=std::uint32_t(e.first&0xffffffffu);
        if(isRim(e)){rimPos[a]=rimPos[b]=1;const auto ra=root(a),rb=root(b);if(ra!=rb)parent[std::max(ra,rb)]=std::min(ra,rb);}}
    for(const auto& e:edges){const std::uint32_t a=std::uint32_t(e.first>>32),b=std::uint32_t(e.first&0xffffffffu);
        if(isRim(e))loopLength[root(a)]+=length(posAt[b]-posAt[a]);}
    // Direct display: skin and wall are not welded; only the skin's canonical lattice points (its seams with
    // static pieces and other regions) stay fixed, the skin opening's own outline may smooth.
    auto lockedSkinPoint=[&](std::uint32_t w){if(!r.directDisplay)return true;const auto& sf=r.rows[wRow[w]].surface;
        const auto q=core::quantExact(Vec{sf[0],sf[1],sf[2]});return std::binary_search(r.lockedSkinKeys.begin(),r.lockedSkinKeys.end(),q);};
    for(std::size_t t=0;t<indices.size();t+=3){if(wTag[indices[t]]&cavityTag)continue;
        for(int k=0;k<3;++k){const auto a=indices[t+k],b=indices[t+(k+1)%3];const auto& c=edges[edgeKey(a,b)];
            if(c[0]+c[1]==1){if(lockedSkinPoint(a))wLock[a]=meshopt_SimplifyVertex_Lock;if(lockedSkinPoint(b))wLock[b]=meshopt_SimplifyVertex_Lock;}}}
    // Rim loops: voxel stair-step perimeter ~1.27x the smooth one. The error limit follows the loops that carry most
    // of the rim (length-weighted radius), so a bullet hole never collapses shut and a blowout still reduces far.
    // Radius of each rim component from its own shape: a closed loop from its length; an open fragment (a
    // chunk sees part of a hole's rim) from arc length vs chord (arc = 2R asin(c/2R)). Stair-step length is
    // ~1.27x the smooth one.
    core::FlatMap<std::uint32_t,int,U32Hash> rimDegree;core::FlatMap<std::uint32_t,std::vector<std::uint32_t>,U32Hash> rimEnds;
    for(const auto& e:edges)if(isRim(e)){++rimDegree[std::uint32_t(e.first>>32)];++rimDegree[std::uint32_t(e.first&0xffffffffu)];}
    for(const auto& d:rimDegree)if(d.second==1)rimEnds[root(d.first)].push_back(d.first);
    double rimSum=0,rimWeighted=0;for(const auto& l:loopLength){double radius=l.second/(2*3.14159265358979*1.27);
        auto ends=rimEnds.find(l.first);
        if(ends!=rimEnds.end()&&ends->second.size()==2){const double c=length(posAt[ends->second[0]]-posAt[ends->second[1]]),arc=l.second/1.27;
            if(arc<=c*1.002)radius=.1;else{double lo=c*.5,hi=1.;for(int it=0;it<60;++it){const double m=.5*(lo+hi);const double a=2*m*std::asin(std::min(1.,c/(2*m)));if(a>arc)lo=m;else hi=m;}radius=std::min(.1,.5*(lo+hi));}}
        rimSum+=l.second;rimWeighted+=l.second*radius;}
    const double radius=r.directDisplay?std::max(1e-3,r.featureRadius):(rimSum>0?rimWeighted/rimSum:.05);
    if(std::getenv("GFY_LOWPOLY_DEBUG")){std::size_t fixedCount=0,openWall=0;for(auto c:fixedPos)fixedCount+=c!=0;for(const auto& e:edges)openWall+=!e.second[0]&&e.second[1]==1;
        double longest=0;for(const auto& l:loopLength)longest=std::max(longest,l.second);
        std::cerr<<"lowpoly_rim loops="<<loopLength.size()<<" rim_m="<<rimSum<<" longest_m="<<longest<<" radius_mm="<<radius*1e3<<" fixed_positions="<<fixedCount<<" open_wall_edges="<<openWall<<std::endl;}
    const double errorLimit=std::clamp(S.errorFraction*radius,S.minError,S.maxError);
    // Depth tiers: a position below the band is "deep" (Settings::deep*; DepthField is a pure function of position).
    const auto deepClock=std::chrono::steady_clock::now();
    const bool deepHere=S.deepEnabled&&(!r.chunked||r.chunkSize>0);
    const double band=deepHere?std::max(S.deepStart,S.deepBandFraction*radius):1e30;
    DepthField depthField(asset);std::vector<float> posDepth(deepHere?nw:0,-1.f);
    auto posDeep=[&](std::uint32_t p){if(!deepHere)return false;auto& d=posDepth[p];if(d<0)d=float(depthField.sample(posAt[p]));return d>band;};
    // Chunked region: the wall part of every seam line is thinned identically in both chunks that share it
    // (same edges, same walk from the lower key, same spacing), dropped seam vertices collapse onto the previous
    // kept one along the line; the kept seam vertices never move. Skin seam vertices stay as they are.
    std::size_t seamDropped=0;
    if(r.chunked){
        auto planeKey=[&](const Vec& p,int& axis,int& side){for(int a=0;a<3;++a){if(std::abs(p[a]-r.seamLoM[a])<=1e-6){axis=a;side=0;return true;}if(std::abs(p[a]-r.seamHiM[a])<=1e-6){axis=a;side=1;return true;}}return false;};
        auto planeCount=[&](const Vec& p){int n=0;for(int a=0;a<3;++a)n+=(std::abs(p[a]-r.seamLoM[a])<=1e-6||std::abs(p[a]-r.seamHiM[a])<=1e-6);return n;};
        // Seam edges: border edges (one triangle) with both ends on the same seam plane, cavity-only.
        core::FlatMap<std::uint32_t,std::vector<std::uint32_t>,U32Hash> adj;std::vector<char> keep(nw,0),skinTouch(nw,0);
        for(std::size_t w=0;w<nw;++w)if(!(wTag[w]&cavityTag))skinTouch[posOf[w]]=1;
        for(const auto& e:edges){if(e.second[0]+e.second[1]!=1||e.second[0])continue;const std::uint32_t a=std::uint32_t(e.first>>32),b=std::uint32_t(e.first&0xffffffffu);
            int ax,sa,bx,sb;if(!planeKey(posAt[a],ax,sa)||!planeKey(posAt[b],bx,sb))continue;
            if(!((std::abs(posAt[a][ax]-posAt[b][ax])<=1e-6)))continue;
            adj[a].push_back(b);adj[b].push_back(a);}
        for(auto& item:adj){auto& l=item.second;std::sort(l.begin(),l.end());l.erase(std::unique(l.begin(),l.end()),l.end());
            if(l.size()!=2||skinTouch[item.first]||planeCount(posAt[item.first])>1)keep[item.first]=1;}
        // Deterministic order key of a position: its exact float bits.
        auto key=[&](std::uint32_t p){PKey k;const Vec& v=posAt[p];const float f[3]={float(v.x),float(v.y),float(v.z)};std::memcpy(&k.x,&f[0],4);std::memcpy(&k.y,&f[1],4);std::memcpy(&k.z,&f[2],4);return std::make_tuple(k.x,k.y,k.z);};
        std::vector<std::uint32_t> collapseTo(nw,~0u);std::vector<char> visited(nw,0);
        const double spacing=S.seamSpacing;
        auto walk=[&](std::uint32_t start,std::uint32_t next){ // start is kept
            std::uint32_t prev=start,at=next,lastKept=start;double run=length(posAt[at]-posAt[start]);
            while(true){visited[at]=1;
                if(keep[at]){lastKept=at;if(at==start)break;}
                else if(run>=spacing){keep[at]=1;lastKept=at;run=0;}
                else collapseTo[at]=lastKept;
                const auto& l=adj[at];if(l.size()!=2||at==start)break;
                const std::uint32_t n=l[0]==prev?l[1]:l[0];if(n==start&&!keep[at]){break;}
                run+=length(posAt[n]-posAt[at]);prev=at;at=n;if(visited[at]&&!keep[at])break;}};
        std::vector<std::uint32_t> order;for(const auto& item:adj)order.push_back(item.first);
        std::sort(order.begin(),order.end(),[&](std::uint32_t a,std::uint32_t b){return key(a)<key(b);});
        for(auto p:order)if(keep[p]){visited[p]=1;auto l=adj[p];std::sort(l.begin(),l.end(),[&](std::uint32_t a,std::uint32_t b){return key(a)<key(b);});for(auto n:l)if(!visited[n]&&!keep[n])walk(p,n);}
        for(auto p:order)if(!visited[p]){keep[p]=1;visited[p]=1;auto l=adj[p];std::sort(l.begin(),l.end(),[&](std::uint32_t a,std::uint32_t b){return key(a)<key(b);});if(!l.empty()&&!visited[l[0]])walk(p,l[0]);}
        // Remap every wall wedge at a dropped position onto the wall wedge of the same material at its target.
        std::vector<std::uint32_t> remap(nw);for(std::uint32_t w=0;w<nw;++w)remap[w]=w;
        core::FlatMap<std::uint32_t,std::uint32_t,U32Hash> wallAt; // position -> first wall wedge
        for(std::uint32_t w=0;w<nw;++w)if(wTag[w]&cavityTag)wallAt.try_emplace(posOf[w],w);
        for(std::uint32_t w=0;w<nw;++w){const auto p=posOf[w];if(collapseTo[p]==~0u)continue;auto t=wallAt.find(collapseTo[p]);if(t==wallAt.end()||!(wTag[w]&cavityTag))continue;remap[w]=t->second;++seamDropped;}
        if(seamDropped){std::vector<unsigned int> kept;kept.reserve(indices.size());
            for(std::size_t t=0;t<indices.size();t+=3){const unsigned a=remap[indices[t]],b=remap[indices[t+1]],c=remap[indices[t+2]];if(a==b||b==c||a==c)continue;kept.insert(kept.end(),{a,b,c});}
            indices=std::move(kept);}}
    // Chunked region: seam vertices are shared with the neighbouring chunk's own reduction: never move them.
    if(r.chunked)for(std::size_t w=0;w<nw;++w){const auto& sf=r.rows[wRow[w]].surface;
        for(int a=0;a<3;++a)if(std::abs(sf[a]-r.seamLoM[a])<=1e-6||std::abs(sf[a]-r.seamHiM[a])<=1e-6){wLock[w]=meshopt_SimplifyVertex_Lock;break;}}
    if(!r.lockMembers.empty()&&r.chunkSize>0)for(std::size_t w=0;w<nw;++w){const auto& sf=r.rows[wRow[w]].surface;int lo[3],hi[3];bool onPlane=false;
        for(int a=0;a<3;++a){const double x=(double(sf[a])-r.chunkOrigin[a])/r.chunkSize;const double fx=std::floor(x);lo[a]=hi[a]=int(fx);
            if((x-fx)*r.chunkSize<=1e-6){lo[a]=int(fx)-1;onPlane=true;}else if((fx+1-x)*r.chunkSize<=1e-6){hi[a]=int(fx)+1;onPlane=true;}}
        if(!onPlane)continue;bool inside=true;
        for(int X=lo[0];X<=hi[0]&&inside;++X)for(int Y=lo[1];Y<=hi[1]&&inside;++Y)for(int Z=lo[2];Z<=hi[2]&&inside;++Z)inside=std::binary_search(r.lockMembers.begin(),r.lockMembers.end(),Cell{X,Y,Z});
        if(!inside)wLock[w]=meshopt_SimplifyVertex_Lock;}
    if(S.rimPriority)for(std::size_t w=0;w<nw;++w)if(rimPos[posOf[w]]&&!wLock[w])wLock[w]=meshopt_SimplifyVertex_Priority;
    fixedExt=freeExt=0;
    for(std::size_t t=0;t<indices.size();t+=3)if(!(wTag[indices[t]]&cavityTag))(wLock[indices[t]]==meshopt_SimplifyVertex_Lock&&wLock[indices[t+1]]==meshopt_SimplifyVertex_Lock&&wLock[indices[t+2]]==meshopt_SimplifyVertex_Lock?fixedExt:freeExt)++;
    std::vector<float> P(3*nw),A(2*nw,0.f);
    for(std::size_t w=0;w<nw;++w){const auto& s=r.rows[wRow[w]].surface;for(int k=0;k<3;++k)P[3*w+k]=s[k];
        if(!(wTag[w]&cavityTag)){A[2*w]=s[12];A[2*w+1]=s[13];}}
    auto wpos=[&](std::uint32_t w){return Vec{P[3*w],P[3*w+1],P[3*w+2]};};
    const float attributeWeights[2]={S.uvWeight,S.uvWeight};
    // Depth tiers (Settings::deep*): the wall below the band reduces first, coarser; its band-boundary positions
    // and every existing lock stay fixed, so the near-skin wall (entry and exit) is reduced exactly as before.
    std::size_t deepIn=0,deepOut=0;double deepArea=0;
    if(deepHere){const std::size_t npos=nw;
        std::vector<unsigned int> deepIdx,keepIdx;deepIdx.reserve(indices.size());keepIdx.reserve(indices.size());
        for(std::size_t t=0;t<indices.size();t+=3){bool deep=true;
            for(int k=0;k<3&&deep;++k){const auto w=indices[t+k];deep=(wTag[w]&cavityTag)!=0&&posDeep(posOf[w]);}
            auto& dst=deep?deepIdx:keepIdx;for(int k=0;k<3;++k)dst.push_back(indices[t+k]);
            if(deep)deepArea+=.5*length(cross(wpos(indices[t+1])-wpos(indices[t]),wpos(indices[t+2])-wpos(indices[t])));}
        if(deepIdx.size()>=48){std::vector<char> boundary(npos,0);for(auto w:keepIdx)boundary[posOf[w]]=1;
            std::vector<unsigned char> dLock(wLock);for(std::uint32_t w=0;w<nw;++w)if(boundary[posOf[w]])dLock[w]|=meshopt_SimplifyVertex_Lock;
            const double deepError=std::clamp(S.deepErrorFraction*radius,errorLimit,std::min(S.deepMaxError,.75*band));
            const std::size_t deepTarget=std::size_t(std::max(8.,S.trisPerCm2*S.deepDensity*deepArea*1e4));
            std::vector<unsigned int> dout(deepIdx.size());float derr=0;
            dout.resize(::meshopt_simplifyWithAttributes(dout.data(),deepIdx.data(),deepIdx.size(),P.data(),nw,12,A.data(),8,attributeWeights,2,
                dLock.data(),3*deepTarget,float(deepError),meshopt_SimplifyErrorAbsolute,&derr));
            if(!dout.empty()&&dout.size()<deepIdx.size()){deepIn=deepIdx.size()/3;deepOut=dout.size()/3;indices=std::move(keepIdx);indices.insert(indices.end(),dout.begin(),dout.end());}
            else deepArea=0;}
        else deepArea=0;}
    r.lowpolyDeepIn=deepIn;r.lowpolyDeepOut=deepOut;r.lowpolyDeepMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-deepClock).count();
    // The wall budget covers the band at the normal density; the deep part keeps what its own pass left.
    const double wallBudget=std::clamp(S.trisPerCm2*(cavArea-deepArea)*1e4,r.chunked?16.:S.minTris,S.maxTris)+double(deepOut);
    // Skin: triangles between the locked skin vertices and the reduced rim, about two per locked skin position
    // that touches a reducible skin triangle (whole untouched faces are already counted in fixedExt).
    std::size_t lockedSkin=0;{std::vector<char> seen(nw,0);
        for(std::size_t t=0;t<indices.size();t+=3){if(wTag[indices[t]]&cavityTag)continue;
            bool fixed=true;for(int k=0;k<3;++k)fixed=fixed&&(wLock[indices[t+k]]&meshopt_SimplifyVertex_Lock);if(fixed)continue;
            for(int k=0;k<3;++k){const auto w=indices[t+k];if((wLock[w]&meshopt_SimplifyVertex_Lock)&&!seen[posOf[w]]){seen[posOf[w]]=1;++lockedSkin;}}}}
    const std::size_t target=std::size_t(wallBudget+double(fixedExt)+2.*double(lockedSkin));
    if(3*target>=indices.size()&&!deepIn)return;
    std::sort(wedgeFaces.begin(),wedgeFaces.end());wedgeFaces.erase(std::unique(wedgeFaces.begin(),wedgeFaces.end()),wedgeFaces.end());
    std::vector<std::uint32_t> faceFirst(nw+1,0);for(auto& e:wedgeFaces)++faceFirst[e.first+1];for(std::size_t w=0;w<nw;++w)faceFirst[w+1]+=faceFirst[w];
    auto faceNormal=[&](std::uint32_t f){const auto& id=asset.source.f[f];const auto& v=asset.source.v;return cross(v[id[1]].p-v[id[0]].p,v[id[2]].p-v[id[0]].p);};
    struct Tri {std::array<std::uint32_t,3> w;bool cav;std::uint32_t tag;std::uint32_t face{~0u};unsigned material{};};
    // Exterior provenance: the source face shared by most corners, never one the reduced triangle faces against.
    // Returns false when every candidate face points away (a flip against the original skin).
    auto chooseFace=[&](Tri& tri,const Vec& n){std::tuple<int,int,double,std::int64_t> bestKey{-1,-1,0.,0};
        for(auto w:tri.w)for(auto e=faceFirst[w];e<faceFirst[w+1];++e){const auto f=wedgeFaces[e].second;int score=0;
            for(auto v:tri.w)for(auto q=faceFirst[v];q<faceFirst[v+1];++q)score+=wedgeFaces[q].second==f;
            const Vec fn=faceNormal(f);const double fl=length(fn),nl=length(n);const double d=fl>0&&nl>0?dot(fn,n)/(fl*nl):0.;
            const std::tuple<int,int,double,std::int64_t> key{d>0,score,d,-std::int64_t(f)};if(key>bestKey){bestKey=key;tri.face=f;}}
        return std::get<0>(bestKey)==1||.5*length(n)<S.minFlipArea;};
    std::vector<unsigned int> out;float error=0;
    bool anySkinWedge=false;for(std::size_t w=0;w<nw&&!anySkinWedge;++w)anySkinWedge=!(wTag[w]&cavityTag);
    std::vector<Tri> tris;std::size_t mixed=0,flipped=0;
    for(unsigned attempt=0;;++attempt){
        out.assign(indices.size(),0);
        if(3*target>=indices.size())out=indices; // already within budget after the deep pass
        else if(anySkinWedge)out.resize(::meshopt_simplifyWithAttributes(out.data(),indices.data(),indices.size(),P.data(),nw,12,A.data(),8,attributeWeights,2,
            wLock.data(),3*target,float(errorLimit),meshopt_SimplifyErrorAbsolute,&error));
        else out.resize(::meshopt_simplifyWithAttributes(out.data(),indices.data(),indices.size(),P.data(),nw,12,nullptr,0,nullptr,0, // walls only: no UV attributes to carry
            wLock.data(),3*target,float(errorLimit),meshopt_SimplifyErrorAbsolute,&error));
        if(out.empty())return; // never publish an empty region; keep the exact mesh
        // Classify reduced triangles; a corner of the other class moves to its sibling wedge at the same position.
        tris.clear();mixed=flipped=0;r.lowpolyFlipArea=0;std::vector<std::pair<Vec,Vec>> flips;
        for(std::size_t t=0;t+2<out.size();t+=3){Tri tri;tri.w={out[t],out[t+1],out[t+2]};
            unsigned cavCorners=0;for(auto w:tri.w)cavCorners+=(wTag[w]&cavityTag)!=0;tri.cav=cavCorners>=2;
            tri.tag=~0u;for(auto w:tri.w)if(bool(wTag[w]&cavityTag)==tri.cav){tri.tag=wTag[w];break;}
            for(auto& w:tri.w)if(wTag[w]!=tri.tag){auto& index=tri.cav?cavIndex:extAt;auto f=index.find(pkey(wRow[w],tri.tag));
                if(f!=index.end())w=f->second;else ++mixed;}
            if(tri.cav){const unsigned m0=wMat[tri.w[0]],m1=wMat[tri.w[1]],m2=wMat[tri.w[2]];tri.material=(m1==m2&&m1!=m0)?m1:m0;}
            const Vec tn=cross(wpos(tri.w[1])-wpos(tri.w[0]),wpos(tri.w[2])-wpos(tri.w[0]));
            if(!tri.cav&&!chooseFace(tri,tn)){++flipped;r.lowpolyFlipArea=std::max(r.lowpolyFlipArea,float(.5*length(tn)));
                Vec lo=wpos(tri.w[0]),hi=lo;for(auto w:tri.w){const Vec p=wpos(w);for(int a=0;a<3;++a){lo[a]=std::min(lo[a],p[a]);hi[a]=std::max(hi[a],p[a]);}}
                flips.emplace_back(lo,hi);}
            tris.push_back(tri);}
        if(flips.empty()||attempt==3)break;
        // A skin triangle flipped against the original skin: freeze the skin around it and reduce again.
        for(auto& f:flips){const Vec c=(f.first+f.second)*.5,h=(f.second-f.first)*.6+Vec{.001,.001,.001};
            for(std::size_t w=0;w<nw;++w){if(wTag[w]&cavityTag&&!rimPos[posOf[w]])continue;const Vec p=wpos(std::uint32_t(w));
                if(std::abs(p.x-c.x)<=h.x&&std::abs(p.y-c.y)<=h.y&&std::abs(p.z-c.z)<=h.z)wLock[w]|=meshopt_SimplifyVertex_Lock;}}
        r.lowpolyRetries=attempt+1;}
    // Smooth wall normals: area-weighted over every reduced cavity triangle at the exact position (all materials).
    core::FlatMap<PKey,Vec,PKeyHash> smooth;smooth.reserve(nw);
    for(const auto& tri:tris)if(tri.cav){const Vec n=cross(wpos(tri.w[1])-wpos(tri.w[0]),wpos(tri.w[2])-wpos(tri.w[0]));
        for(auto w:tri.w){auto& s=smooth[pkey(wRow[w],0)];s=s+n;}}
    decltype(r.rows) rows;decltype(r.hashes) hashes;decltype(r.discarded) discarded;
    core::FlatMap<std::uint64_t,std::uint32_t,U64Hash> rowOf;rowOf.reserve(nw*2);
    std::map<GroupKey,std::vector<PackedTri>> groups;std::size_t cavOut=0;const std::uint16_t cavitySource=asset.face_children.front();
    for(const auto& tri:tris){
        const Vec n=cross(wpos(tri.w[1])-wpos(tri.w[0]),wpos(tri.w[2])-wpos(tri.w[0]));
        PackedTri packed;packed.rect=~0u;
        if(tri.cav){++cavOut;packed.face=~std::uint32_t(0);
            int axis=0;for(int a=1;a<3;++a)if(std::abs(n[a])>std::abs(n[axis]))axis=a;
            Vec an{};an[axis]=n[axis]<0?-1.:1.;
            // Former cavity convention (join_cavity.hpp cavityPolygons): axis tangent, binormal = N x T, UV = p.(T,B)/3 cm.
            const Vec T=std::abs(an.x)>.9?Vec{0,0,1}:Vec{1,0,0};const Vec B=cross(an,T);
            const std::uint32_t code=std::uint32_t(axis*2+(n[axis]<0));
            for(int k=0;k<3;++k){const auto w=tri.w[k];const std::uint64_t key=(std::uint64_t(w)<<3)|code;
                auto it=rowOf.find(key);
                if(it==rowOf.end()){auto row=r.rows[wRow[w]];const Vec p=wpos(w);
                    Vec N=an;{auto s=smooth.find(pkey(wRow[w],0));if(s!=smooth.end()&&length(s->second)>1e-20)N=s->second*(1/length(s->second));}
                    Vec t=T-N*dot(N,T);if(length(t)<1e-6)t=B-N*dot(N,B);t=t*(1/length(t));const Vec b=cross(N,t);
                    const Vec vectors[3]={N,t,b};for(int v=0;v<3;++v)for(int c=0;c<3;++c)row.surface[3+3*v+c]=float(vectors[v][c]);
                    row.surface[12]=float(dot(p,T)/.08);row.surface[13]=float(dot(p,B)/.08); /* wall texture tile 8 cm */
                    it=rowOf.emplace(key,std::uint32_t(rows.size())).first;
                    rows.push_back(row);hashes.push_back(core::packing::vertexHash(row));discarded.push_back(r.discarded[wRow[w]]);}
                packed.row[k]=it->second;}
            groups[{tri.material,cavitySource}].push_back(packed);
            continue;}
        packed.face=tri.face;
        for(int k=0;k<3;++k){const auto w=tri.w[k];const std::uint64_t key=(std::uint64_t(w)<<3)|7u;auto it=rowOf.find(key);
            if(it==rowOf.end()){it=rowOf.emplace(key,std::uint32_t(rows.size())).first;rows.push_back(r.rows[wRow[w]]);hashes.push_back(r.hashes[wRow[w]]);discarded.push_back(r.discarded[wRow[w]]);}
            packed.row[k]=it->second;}
        groups[{0u,std::uint16_t(tri.tag)}].push_back(packed);}
    {std::size_t locked=0;for(auto l:wLock)locked+=(l&meshopt_SimplifyVertex_Lock)!=0;r.lowpolyLocked=locked;r.lowpolySeamDropped=seamDropped;r.lowpolyWedges=nw;}
    {std::size_t deepFinal=0;if(deepHere)for(const auto& tri:tris)if(tri.cav){bool deep=true;for(auto w:tri.w)deep=deep&&posDepth[posOf[w]]>band;deepFinal+=deep;}r.lowpolyDeepFinal=deepFinal;}
    r.lowpolyInput=r.triangles;r.lowpolyTarget=target;r.lowpolyFixedExt=fixedExt;r.lowpolyFreeExt=lockedSkin;r.lowpolyError=error;r.lowpolyLimit=float(errorLimit);r.lowpolyMixed=mixed;r.lowpolyFlipped=flipped;
    r.rows=std::move(rows);r.hashes=std::move(hashes);r.discarded=std::move(discarded);
    r.groups.clear();for(auto& g:groups)r.groups.emplace_back(g.first,std::move(g.second));
    r.triangles=tris.size();r.cavityTriangles=cavOut;r.rects.clear();r.planeRows.clear();
    r.lowpolyMs=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
}

// Same reduction over a finished packed mesh (dismemberment pieces and stumps, which are packed by
// packRefs/packMulti: every original exterior child first, then cavity children appended in material order).
// Exterior triangles whose source face is unknown to the asset (e.g. accessory surfaces) keep all their vertices.
// Debug only (GFY_LOWPOLY_DUMP=<prefix>): VXPACK01 dump of a packed mesh for the offline renderer.
inline void dumpPacked(const gfy_packed::Mesh& mesh,const std::string& path){
    std::ofstream o(path,std::ios::binary);o.write("VXPACK01",8);const std::uint32_t n=std::uint32_t(mesh.children.size());o.write((const char*)&n,4);
    for(const auto& c:mesh.children){const std::uint32_t h[5]={c.child_index,c.material_source_child,c.material,std::uint32_t(c.vertices.size()),std::uint32_t(c.indices.size())};
        o.write((const char*)h,20);o.write((const char*)c.vertices.data(),std::streamsize(c.vertices.size()*80));o.write((const char*)c.indices.data(),std::streamsize(c.indices.size()*2));}}
template<class AssetT>
void reducePackedMesh(const AssetT& asset,gfy_packed::Mesh& mesh){
    static std::atomic<unsigned> dumpCounter{0};const char* dumpPrefix=std::getenv("GFY_LOWPOLY_DUMP");const unsigned dumpId=dumpPrefix?dumpCounter++:0;
    if(dumpPrefix)dumpPacked(mesh,std::string(dumpPrefix)+std::to_string(dumpId)+"_in.packed");
    struct DumpOut {const char* prefix;unsigned id;gfy_packed::Mesh& mesh;~DumpOut(){if(prefix)dumpPacked(mesh,std::string(prefix)+std::to_string(id)+"_out.packed");}} dumpOut{dumpPrefix,dumpId,mesh};
    if(!settings().enabled||mesh.children.empty()||asset.face_children.empty())return;
    std::size_t cavity=0;for(const auto& c:mesh.children)if(c.material)cavity+=c.indices.size()/3;
    if(!cavity)return;
    std::unordered_map<std::uint64_t,std::uint32_t> sourceFace;sourceFace.reserve(asset.face_children.size());
    for(std::uint32_t f=0;f<asset.face_children.size();++f)sourceFace.emplace((std::uint64_t(asset.face_children[f])<<32)|asset.face_indices[f],f);
    RegionResult r;std::map<GroupKey,std::vector<PackedTri>> groups;std::vector<std::uint16_t> exteriorOrder;
    // Exterior triangles of other surfaces (accessory/garment faces appended by the separation) stay verbatim;
    // the vertices they share with reducible skin are border vertices of the reduction and therefore locked.
    struct Foreign {std::array<gfy_packed::Vertex,3> v;std::uint32_t original;};std::map<std::uint16_t,std::vector<Foreign>> foreign;std::size_t foreignCount=0;
    for(const auto& c:mesh.children){const std::uint32_t base=std::uint32_t(r.rows.size());
        for(const auto& v:c.vertices){r.rows.push_back(v);r.hashes.push_back(core::packing::vertexHash(v));r.discarded.push_back(0.);}
        if(!c.material)exteriorOrder.push_back(c.child_index);
        for(std::size_t t=0;t+2<c.indices.size();t+=3){PackedTri tri;tri.row[0]=base+c.indices[t];tri.row[1]=base+c.indices[t+1];tri.row[2]=base+c.indices[t+2];
            if(c.material){tri.face=~std::uint32_t(0);groups[{c.material,asset.face_children.front()}].push_back(tri);continue;}
            auto f=sourceFace.find((std::uint64_t(c.material_source_child)<<32)|c.original_faces[t/3]);
            if(c.material_source_child!=c.child_index)return; // overflowed exterior child: keep this mesh exact
            if(f==sourceFace.end()){foreign[c.child_index].push_back({{c.vertices[c.indices[t]],c.vertices[c.indices[t+1]],c.vertices[c.indices[t+2]]},c.original_faces[t/3]});++foreignCount;continue;}
            tri.face=f->second;groups[{0u,c.child_index}].push_back(tri);}}
    static const bool debug=std::getenv("GFY_LOWPOLY_DEBUG")!=nullptr;
    if(debug){std::size_t over=0;for(const auto& c:mesh.children)if(!c.material&&c.child_index>=asset.original_children)++over;
        std::cerr<<"lowpoly_mesh children="<<mesh.children.size()<<" cavity="<<cavity<<" foreign="<<foreignCount<<" overflow_exterior_children="<<over<<std::endl;}
    for(auto& g:groups){r.triangles+=g.second.size();if(g.first.first)r.cavityTriangles+=g.second.size();r.groups.emplace_back(g.first,std::move(g.second));}
    std::unordered_set<PKey,PKeyHash> fixed;
    for(const auto& f:foreign)for(const auto& t:f.second)for(const auto& v:t.v){PKey k;std::memcpy(&k.x,&v.surface[0],4);std::memcpy(&k.y,&v.surface[1],4);std::memcpy(&k.z,&v.surface[2],4);k.tag=0;fixed.insert(k);}
    reduceRegion(asset,r,fixed.empty()?nullptr:&fixed);
    if(debug)std::cerr<<"lowpoly_mesh in="<<r.lowpolyInput<<" out="<<r.triangles<<" cavity="<<r.cavityTriangles<<" target="<<r.lowpolyTarget<<" error_mm="<<r.lowpolyError*1e3<<" limit_mm="<<r.lowpolyLimit*1e3<<" ms="<<r.lowpolyMs<<std::endl;
    if(!r.lowpolyInput)return; // nothing reduced
    // Repack exactly like packRefs: exterior children in their former order (empty ones stay, they remove the
    // render child), then cavity groups in material order on consecutive appended slots.
    std::map<std::uint16_t,const std::vector<PackedTri>*> exterior;std::vector<const std::pair<GroupKey,std::vector<PackedTri>>*> cav;
    for(const auto& g:r.groups){if(g.first.first)cav.push_back(&g);else exterior[g.first.second]=&g.second;}
    unsigned next=asset.original_children;
    gfy_packed::Mesh out;out.reduced_skin_vertices=mesh.reduced_skin_vertices;out.max_discarded_skin_weight=mesh.max_discarded_skin_weight;
    core::packing::VertexIndex vertices;
    auto pack=[&](unsigned material,std::uint16_t source,std::uint16_t index,const std::vector<PackedTri>* tris){
        gfy_packed::Child child;child.material=material;child.material_source_child=source;child.child_index=index;
        if(tris){vertices.reset(std::min<std::size_t>(3*tris->size(),65535));
            for(const auto& t:*tris){std::array<long,3> found;unsigned fresh=0;
                for(int k=0;k<3;++k){found[k]=vertices.find(child.vertices,r.rows[t.row[k]],r.hashes[t.row[k]]);if(found[k]<0)++fresh;}
                if(child.vertices.size()+fresh>65535||child.indices.size()/3>=65535){
                    core::packing::packingRequire(material!=0,"lowpoly_exterior_child_overflow");
                    out.children.push_back(std::move(child));child={};child.material=material;child.material_source_child=source;
                    core::packing::packingRequire(next<gfy_packed::max_child_slots,"Runtime child slots exhausted");child.child_index=std::uint16_t(next++);vertices.clear();found.fill(-1);}
                for(int k=0;k<3;++k)child.indices.push_back(found[k]>=0?std::uint16_t(found[k]):vertices.insert(child.vertices,r.rows[t.row[k]],r.hashes[t.row[k]]).first);
                child.original_faces.push_back(material?~std::uint32_t(0):asset.face_indices[t.face]);}}
        if(!material){auto it=foreign.find(index);if(it!=foreign.end()){if(!tris)vertices.reset(std::min<std::size_t>(3*it->second.size(),65535));
            for(const auto& t:it->second){std::array<std::uint64_t,3> h;std::array<long,3> found;unsigned fresh=0;
                for(int k=0;k<3;++k){h[k]=core::packing::vertexHash(t.v[k]);found[k]=vertices.find(child.vertices,t.v[k],h[k]);if(found[k]<0)++fresh;}
                core::packing::packingRequire(child.vertices.size()+fresh<=65535&&child.indices.size()/3<65535,"lowpoly_exterior_child_overflow");
                for(int k=0;k<3;++k)child.indices.push_back(found[k]>=0?std::uint16_t(found[k]):vertices.insert(child.vertices,t.v[k],h[k]).first);
                child.original_faces.push_back(t.original);}}}
        out.children.push_back(std::move(child));};
    for(const auto& c:mesh.children)if(!c.material){auto it=exterior.find(c.child_index);pack(0,c.material_source_child,c.child_index,it==exterior.end()?nullptr:it->second);}
    for(const auto* g:cav){core::packing::packingRequire(next<gfy_packed::max_child_slots,"Runtime child slots exhausted");pack(g->first.first,g->first.second,std::uint16_t(next++),&g->second);}
    mesh=std::move(out);
}
} // namespace lowpoly
