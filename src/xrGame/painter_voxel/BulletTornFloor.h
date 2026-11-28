#pragma once
#include "LocalCraterCells.h"
#include "CraterNoiseRow.h"
#include "CraterGrain.h"
#include "CraterBackedBlocks.h"
#include "CraterPlanes.h"
#include "CraterMargin.h"
#include "CraterMaterialRow.h"
#include "CraterFour.h"
#include "CraterErase.h"
// Bullet craters keep the released warped cell ray and radius. This local
// height field shapes only tissue that the original crater elected to remove.
struct BulletSurfaceSource {
    const core::BodyBVH* tree{};
    const painter_detail::TissueField* tissue{};
    unsigned bodyFaces{};
    Vec inward{};
    Vec limbAxis{0,1,0};
};
struct BulletTornFloor {
    struct CertifiedBoxes {
        struct Bound {Vec n,absolute;double offset{},warp{};};
        std::vector<CraterPlanes::Group> nearSites;
        const head_burst_record::Function& fn;
        explicit CertifiedBoxes(const head_burst_record::Function& f):fn(f){
            for(unsigned i=0;i<f.S;++i)if(f.group[i]){
                const Vec a{f.site[i][0],f.site[i][1],f.site[i][2]};
                CraterPlanes::Group bounds;
                for(unsigned j=0;j<f.S;++j)if(!f.group[j]){
                    const Vec b{f.site[j][0],f.site[j][1],f.site[j][2]},k=a-b;
                    const Vec ab{std::abs(k.x),std::abs(k.y),std::abs(k.z)};
                    bounds.add(k,-dot(k,(a+b)*.5),(ab.x+ab.y+ab.z)*(std::abs(f.A1)+std::abs(f.A2))+1e-13);
                }
                nearSites.push_back(std::move(bounds));
            }
        }
        unsigned test(const double lo[3],const double hi[3])const{
            const Vec a{lo[0],lo[1],lo[2]},b{hi[0],hi[1],hi[2]},p=(a+b)*.5,h=(b-a)*.5;
            const Vec delta{std::abs(p.x-fn.B[0]),std::abs(p.y-fn.B[1]),std::abs(p.z-fn.B[2])};
            const Vec gap{std::max(0.,delta.x-h.x),std::max(0.,delta.y-h.y),std::max(0.,delta.z-h.z)};
            if(dot(gap,gap)>fn.Rcut*fn.Rcut+1e-13)return 0;
            bool ambiguous=false;
            for(const auto& site:nearSites){
                const int bound=site.box(p,h);
                if(bound>0&&dot(delta+h,delta+h)<fn.Rcut*fn.Rcut-1e-13)return 1;
                if(bound>=0)ambiguous=true;
            }
            return ambiguous?255u:0u;
        }
    };


    struct Column { double lo=1e8,hi=-1e8;unsigned deepest=0; };
    std::vector<Column> column;
    std::vector<float> bottom;
    Vec mouth{},dir{},u{},v{};
    double x0{},y0{},step{},inv{};
    int nx{},ny{};
    struct Face { Vec normal{}; double offset{}; };
    struct Region { std::vector<Face> sides; };
    std::vector<Region> regions;
    const head_burst_record::Function* cutter=nullptr;
    double scale{}, pitch{};
    const painter_detail::TissueField* display=nullptr;
    std::uint64_t pattern{};
    bool ready=false;
    static double grain(double x,double y,std::uint64_t seed){
        const int ix=int(std::floor(x)),iy=int(std::floor(y));
        const double fx=x-ix,fy=y-iy,s=fx*fx*(3.-2.*fx),t=fy*fy*(3.-2.*fy);
        auto corner=[&](int a,int b){return double(core::quantMix(seed^(std::uint64_t(a)*0xc2b2ae3d27d4eb4full)^(std::uint64_t(b)*0x9e3779b97f4a7c15ull))>>11)*(2./9007199254740992.)-1.;};
        return (corner(ix,iy)*(1.-s)+corner(ix+1,iy)*s)*(1.-t)+(corner(ix,iy+1)*(1.-s)+corner(ix+1,iy+1)*s)*t;
    }
    void prepare(Grid& g,const std::vector<Cell>& cells,const head_burst::Plan& plan,const head_burst_record::Function& fn,const BulletSurfaceSource& src){
        if(cells.size()<32||!src.tissue||dot(src.inward,src.inward)<.9)return;
        dir=unit(src.inward);mouth=plan.entry;
        cutter=&fn; display=src.tissue; scale=plan.rf; pitch=g.pitch; pattern=plan.seed;
        // A bullet's existing near-site union determines the original size.
        // In the warped record space its boundary has exact signed halfplanes.
        // Internal fragment partition planes are excluded from a plain crater.
        for(unsigned i=0;i<fn.S;++i)if(fn.group[i]){
            Region region;
            const Vec nearSite{fn.site[i][0],fn.site[i][1],fn.site[i][2]};
            for(unsigned j=0;j<fn.S;++j)if(!fn.group[j]){
                const Vec farSite{fn.site[j][0],fn.site[j][1],fn.site[j][2]};
                const Vec delta=nearSite-farSite;
                const double len=length(delta);
                if(len<=1e-9)continue;
                const Vec normal=delta*(1./len);
                region.sides.push_back({normal,-dot(normal,(nearSite+farSite)*.5)});
            }
            if(!region.sides.empty())regions.push_back(std::move(region));
        }
        if(regions.empty())return;
        const Vec localAxis=src.limbAxis-dir*dot(src.limbAxis,dir);
        u=length(localAxis)>.15?unit(localAxis):unit(cross(dir,std::abs(dir.x)<.8?Vec{1,0,0}:Vec{0,1,0}));
        v=cross(dir,u);
        double u0=1e8,v0=1e8,u1=-1e8,v1=-1e8;
        double z0=1e8,z1=-1e8;
        for(const auto c:cells){
            const Vec p=g.center(c)-mouth;const double a=dot(p,u),b=dot(p,v),z=dot(p,dir);
            u0=std::min(a,u0);u1=std::max(a,u1);v0=std::min(b,v0);v1=std::max(b,v1);
            z0=std::min(z,z0);z1=std::max(z,z1);
        }
        actualLo={u0,v0,z0};actualHi={u1,v1,z1};
        step=std::max({g.pitch,(u1-u0)/96.,(v1-v0)/96.});inv=1./step;
        x0=u0-2*step;y0=v0-2*step;
        nx=std::max(6,int(std::ceil((u1-u0)*inv))+5);ny=std::max(6,int(std::ceil((v1-v0)*inv))+5);
        column.resize(std::size_t(nx)*ny);bottom.assign(column.size(),1e8f);
        // Splat the exact original depth extent onto adjacent lattice points.
        // Taking the union of adjacent projections avoids jagged empty columns.
        // Each cell contributes to FOUR columns. Bin it once and merge
        // adjacent bins. Equal depths still pick the first cell in V5's
        // native lexicographic order, including ties across bins or workers.
        struct Bin{double lo=1e8,hi=-1e8;std::size_t first=SIZE_MAX;};
        const auto width=core::parallelWidth();
        std::vector<std::vector<Bin>> bins(width);
        core::parallelChunks(cells.size(),4096,[&](std::size_t first,std::size_t last,std::size_t chunk){
            auto& local=bins[chunk];local.resize(column.size());
            for(std::size_t i=first;i<last;++i){
                const Vec p=g.center(cells[i])-mouth;
                const int x=int(std::floor((dot(p,u)-x0)*inv)),y=int(std::floor((dot(p,v)-y0)*inv));
                const double z=dot(p,dir);auto& b=local[std::size_t(y)*nx+x];
                b.lo=std::min(b.lo,z);
                if(z>b.hi){b.hi=z;b.first=i;}
            }
        });
        for(int y=0;y<ny;++y)for(int x=0;x<nx;++x){
            auto& col=column[std::size_t(y)*nx+x];std::size_t winner=SIZE_MAX;
            for(const auto& chunk:bins)if(!chunk.empty()){
                for(int j=std::max(0,y-1);j<=y;++j)for(int i=std::max(0,x-1);i<=x;++i){
                    const auto& b=chunk[std::size_t(j)*nx+i];
                    col.lo=std::min(col.lo,b.lo);
                    if(b.hi>col.hi||(b.hi==col.hi&&b.first<winner)){col.hi=b.hi;winner=b.first;}
                }
            }
            if(winner!=SIZE_MAX){const Vec p=g.center(cells[winner]);col.deepest=src.tissue->sample(p.x,p.y,p.z)&63u;}
        }
        const auto seed=std::uint64_t(plan.seed);
        for(int y=0;y<ny;++y)for(int x=0;x<nx;++x){
            const auto ix=std::size_t(y)*nx+x;const Column n=column[ix];
            if(n.hi<n.lo)continue;
            const double a=x0+x*step,b=y0+y*step,span=n.hi-n.lo+g.pitch;
            const double broad=grain(a/.024,b/.022,seed);
            const double shear=grain(a/.014+.4*broad,b/.009,seed+7);
            const double fibres=grain(a/.018+.47*broad,b/.0036+.84*shear,seed+13);
            const double fat=grain(a/.0052,b/.0052,seed+19);
            const bool muscle=n.deepest==5||(n.deepest>=27&&n.deepest<=34);
            const bool bone=n.deepest==2||n.deepest==21||n.deepest==8;
            // Broad folds and smaller fissures use the same rest-space scales
            // as grenade bites. The deepest surviving tissue changes the floor:
            // muscle lengthens in the limb frame, connective fat gathers in
            // lobes, and bone stands farther out. Each ray still has one floor.
            const double groove=std::max(0.,1.-3.8*std::abs(shear+.34*broad+.19*fibres));
            const double lobe=1.-2.*std::abs(fat);
            const double retain=bone?.42+.20*broad+.12*shear:
                                .23+.20*broad+.12*shear+.11*(muscle?fibres:lobe)-.12*groove;
            const double relief=std::min(span*std::clamp(retain,.015,.70),std::max(.002,plan.rf*(bone?.46:.37)));
            // A thin native bite preserves its opening; relief has room only
            // along a column thick enough to hold a continuous attached floor.
            bottom[ix]=float(n.hi+.6*g.pitch-relief*std::clamp((span-2*g.pitch)/(4*g.pitch),0.,1.));
        }
        ready=true;
    }
    double depthAt(const Vec& pos)const{
        if(!ready)return 1e8;
        const Vec p=pos-mouth; const double x=(dot(p,u)-x0)*inv,y=(dot(p,v)-y0)*inv;
        const int i=int(std::floor(x)),j=int(std::floor(y));
        if(i<0||j<0||i+1>=nx||j+1>=ny)return 1e8;
        const double fx=x-i,fy=y-j;double d=0,wsum=0;
        for(int b=0;b<2;++b)for(int a=0;a<2;++a){
            const float depth=bottom[(j+b)*nx+i+a]; const double w=(a?fx:1-fx)*(b?fy:1-fy);
            if(depth<1e7){d+=depth*w;wsum+=w;}
        }
        return wsum>1e-8?d/wsum:1e8;
    }
    double scoreAt(const Vec& pos,double floorDepth,double& margin,bool ranking=false,CraterNoiseRow* row=nullptr,const double* known=nullptr)const{
        margin=-1e8;
        const Vec rel=pos-mouth;
        if(dot(pos-Vec{cutter->B[0],cutter->B[1],cutter->B[2]},
               pos-Vec{cutter->B[0],cutter->B[1],cutter->B[2]})>cutter->Rcut*cutter->Rcut)return -1e8;
        if(known)margin=*known;
        else{
            double w[3];if(row)row->warp(*cutter,pos,w);else cutter->warped(pos.x,pos.y,pos.z,w);
            margin=spatial->planes.at({w[0],w[1],w[2]});
        }
        const double dz=floorDepth-dot(rel,dir);
        if(ranking){
            if(margin< -std::min(6*pitch,.17*scale))return margin;
            if(margin>1.45*std::min(.013,.22*scale)&&dz>=0)return std::min(dz,margin);
        }
        const double s=dot(rel,u),t=dot(rel,v),z=dot(rel,dir);
        // The broad and fine fields follow the wall in 3D, including its
        // side-facing parts. A narrow branching groove rejoins the stock cut;
        // neighbouring lobes leave a real, sloping shoulder of original tissue.
        const double a=s+.41*z, b=t-.57*z;
        const double broad=relief[0].at(a*46.,b*46.);
        const double ridge=relief[1].at(a*92.+.37*broad,b*92.+.74*broad);
        const double nick=relief[2].at(a*198.,b*198.);
        const double branch=std::min(std::abs(ridge+.32*broad+.18*nick),
                                     std::abs(.56*broad-.33*ridge+.22*nick-.33));
        const double shoulder=std::clamp(4.6*branch,0.,1.);
        const double amplitude=std::clamp(.72+.55*broad+.35*ridge+.27*nick,.10,1.45);
        const double depth=std::min(.013,.22*scale)*shoulder*amplitude;
        return std::min(margin-depth,floorDepth-dot(rel,dir));
    }
    // Discard tissue that the ragged profile would leave suspended. Predict
    // the existing native edge repairs, then check only newly retained stock
    // cells against their fitted neighbours. Every correction trades one cut
    // cell for another already-admitted cell, keeping its native volume.
    template<class Map,class Has>
    bool support(Grid& original,const std::vector<Cell>& old,const Map& stock,Has has,
                 const std::vector<Cell>& additions,std::vector<Cell>& reerase,std::set<Cell>& giveBack)const{
        if(additions.empty()||!display||!display->available)return true;
        const unsigned ratio=original.pitch/display->pitch>1.5?2u:1u;
        const unsigned all=(1u<<(ratio*ratio*ratio))-1;
        Cell at{};
        if(ratio==2)for(int a=0;a<3;++a)
            (&at.x)[a]=int(std::llround((original.origin[a]-display->origin[a])/display->pitch));
        const auto maskAt=[&](Cell c){
            if(ratio==1){
                const Vec p=original.center(c);const unsigned n=display->sample(p.x,p.y,p.z);
                return unsigned(n!=0&&n!=16);
            }
            const int nx=at.x+2*c.x,ny=at.y+2*c.y,nz=at.z+2*c.z;
            if(nx>=0&&ny>=0&&nz>=0&&unsigned(nx+1)<display->size[0]&&unsigned(ny+1)<display->size[1]&&unsigned(nz+1)<display->size[2]&&
               (nx&7)!=7&&(ny&7)!=7&&(nz&7)!=7){
                const auto page=display->page[(std::size_t(nz>>3)*display->pages[1]+unsigned(ny>>3))*display->pages[0]+unsigned(nx>>3)+display->headerWords];
                if(page&display->uniformBit){const unsigned n=page&127u;return n!=0&&n!=16?255u:0u;}
            }
            unsigned bits=0;
            for(unsigned i=0;i<8;++i){
                const unsigned n=display->at(at.x+2*c.x+int(i>>2),at.y+2*c.y+int((i>>1)&1),at.z+2*c.z+int(i&1));
                if(n!=0&&n!=16)bits|=1u<<i;
            }
            return bits;
        };
        const auto face=[&](unsigned from,int axis,int sign){
            if(ratio==1)return from;
            const unsigned low[3]={0x0f,0x33,0x55},step=1u<<(2-axis);
            return sign>0?(from&low[axis])<<step:(from&~low[axis])>>step;
        };
        const auto grow=[&](unsigned bits,unsigned start){
            if(!start||bits==all||ratio==1)return start?bits:0u;
            unsigned reach=start&bits;
            for(;;){
                unsigned next=reach;
                for(int a=0;a<3;++a)next|=face(reach,a,1)|face(reach,a,-1);
                next&=bits;if(reach==next)return next;reach=next;
            }
        };
        Grid& trial=original; core::TileCache readOnly(trial);
        // The atlas is sampled only in the exchanged band. Retained subcells
        // first touch a survivor outside the stock cut, then connect locally.
        struct Part{Cell c;unsigned bits{},live{};};
        std::vector<Part> part;
        std::unordered_map<Cell,unsigned,core::CellHash> index;
        const bool compact=!region.flags.empty() && std::all_of(additions.begin(),additions.end(),[&](Cell c){return region.contains(c);}) && region.flags.size()<=std::min<std::size_t>(512u<<10,(old.size()+additions.size())*96);
        std::vector<int> cellIndex;if(compact)cellIndex.assign(region.flags.size(),0);
        auto findPart=[&](Cell c){
            if(compact)return region.contains(c)?cellIndex[region.index(c)]-1:-1;
            auto i=index.find(c);return i==index.end()?-1:int(i->second);
        };
        for(Cell c:old)if(!readOnly.removed(c)){
            const unsigned bits=maskAt(c);if(bits){if(compact)cellIndex[region.index(c)]=int(part.size()+1);else index.emplace(c,unsigned(part.size()));part.push_back({c,bits,0});}
        }
        auto around=[&](Cell c,auto visit){for(int a=0;a<3;++a)for(int s:{-1,1}){
            Cell n=c;(&n.x)[a]+=s;if(original.valid(n))visit(n,a,s);
        }};
        auto outsideWith=[&](Cell c,int a,int s,core::TileCache& access){
            if(has(stock,c)||access.removed(c))return 0u;
            return face(maskAt(c),a,s);
        };
        auto outside=[&](Cell c,int a,int s){return outsideWith(c,a,s,readOnly);};
        std::vector<unsigned> queue;
        std::vector<std::vector<unsigned>> seedList(core::parallelWidth());
        core::parallelChunks(part.size(),512,[&](std::size_t lo,std::size_t hi,std::size_t lane){
          core::TileCache localRead(trial);
          for(unsigned i=unsigned(lo);i<hi;++i){
            auto& p=part[i];unsigned seed=0;
            around(p.c,[&](Cell n,int a,int s){if(!seed||(p.bits!=all&&(seed&p.bits)!=p.bits))seed|=outsideWith(n,a,s,localRead);});
            p.live=grow(p.bits,seed&p.bits);if(p.live)seedList[lane].push_back(i);
          }
        });
        for(const auto& list:seedList)queue.insert(queue.end(),list.begin(),list.end());
        for(std::size_t head=0;head<queue.size();++head){
            const auto p=part[queue[head]];
            around(p.c,[&](Cell n,int a,int s){
                const int i=findPart(n);if(i<0)return;
                auto& q=part[std::size_t(i)];if(q.live==q.bits)return;
                const unsigned incoming=q.bits&~q.live&face(p.live,a,-s);
                if(incoming){q.live=grow(q.bits,q.live|incoming);queue.push_back(unsigned(i));}
            });
        }
        for(const auto& p:part)if(p.bits&&!p.live)reerase.push_back(p.c);
        if(reerase.empty())return true;
        // Choose leaves of the added cut's connected tree, at its outside
        // interface. Returning a leaf cannot isolate any remaining cut island.
        std::unordered_map<Cell,std::size_t,core::CellHash> seen;
        std::vector<int> parent(additions.size(),-1),children(additions.size());
        for(std::size_t i=0;i<additions.size();++i){
            bool root=false;
            around(additions[i],[&](Cell n,int,int){if(has(stock,n)&&readOnly.removed(n))root=true;});
            if(!root)around(additions[i],[&](Cell n,int,int){
                if(parent[i]>=0)return;
                if(compact){if(region.contains(n)){const int value=cellIndex[region.index(n)];if(value<0)parent[i]=-value-1;}}
                else{auto it=seen.find(n);if(it!=seen.end())parent[i]=int(it->second);}
            });
            if(!root&&parent[i]<0)return false;
            if(parent[i]>=0)++children[parent[i]];
            if(compact)cellIndex[region.index(additions[i])]=-int(i)-1;else seen.emplace(additions[i],i);
        }
        for(std::size_t i=additions.size();i-->0&&giveBack.size()<reerase.size();){
            if(children[i])continue;
            const Cell c=additions[i];const unsigned m=maskAt(c);unsigned seed=0;
            around(c,[&](Cell n,int a,int s){if(!seed)seed|=m&outside(n,a,s);});
            if(m&&!seed)continue;
            giveBack.insert(c);
            if(parent[i]>=0)--children[parent[i]];
        }
        if(giveBack.size()!=reerase.size())return false;
        return true;
    }

    using CellsByTile=std::unordered_map<Cell,core::Tile,core::CellHash>;
    CellsByTile originalTiles;
    LocalCraterCells region;
    CraterGrain relief[3];
    const CraterMargin* spatial=nullptr;
    std::vector<Cell> originalSet, additionsOrdered;
    decltype(SeverState::table) materialTable{};
    template<class SlotCounts>
    void finishSupport(Grid& g,std::vector<Cell>& fresh,std::set<Cell>& touched,
                       SlotCounts& bySlot,std::size_t& edges,std::size_t& solidEdges)const{
        if(originalSet.empty()||additionsOrdered.empty())return;
        auto has=[this](const CellsByTile& bits,Cell c){
            if(!region.flags.empty())return region.marked(c,1);
            const auto i=bits.find(Grid::tileKey(c));return i!=bits.end()&&core::tileBit(i->second,c);
        };
        std::vector<Cell> take;
        std::set<Cell> giveBack;
        if(!support(g,originalSet,originalTiles,has,additionsOrdered,take,giveBack)||take.empty())return;
        std::set<Cell> neighbourhood;
        auto changed=[&](Cell c){
            touched.insert(Grid::tileKey(c));
            for(int z=-1;z<=1;++z)for(int y=-1;y<=1;++y)for(int x=-1;x<=1;++x){
                const Cell n{c.x+x,c.y+y,c.z+z};if(g.valid(n)&&g.removed(n))neighbourhood.insert(n);
            }
        };
        for(Cell c:giveBack){
            const unsigned bit=Grid::local(c);
            g.writeTile(Grid::tileKey(c)).removed[bit>>6]&=~(std::uint64_t(1)<<(bit&63));
            if(auto lab=g.materialBase(c)){const unsigned slot=g.material_slots&&lab<g.material_slots->size()?unsigned((*g.material_slots)[lab]&63u):1u;--bySlot[slot];}
        }
        for(Cell c:take){
            g.erase(c);
            if(auto lab=g.materialBase(c)){const unsigned slot=g.material_slots&&lab<g.material_slots->size()?unsigned((*g.material_slots)[lab]&63u):1u;++bySlot[slot];}
        }
        fresh.erase(std::remove_if(fresh.begin(),fresh.end(),[&](Cell c){return giveBack.count(c)!=0;}),fresh.end());
        fresh.insert(fresh.end(),take.begin(),take.end());
        for(Cell c:giveBack)changed(c);
        for(Cell c:take)changed(c);
        // Only the two exchanged sides can alter a previously solved edge.
        // The existing solver grows its frontier until nearby contacts settle.
        const std::vector<Cell> local(neighbourhood.begin(),neighbourhood.end());
        edges+=resolveEdgeContacts(g,local,Vec{cutter->B[0],cutter->B[1],cutter->B[2]},
                                   [](Cell){return true;},touched,&solidEdges,true);
    }

    struct Ranked { double score; Cell cell; bool old;
        Ranked(Cell c,double value,bool original):score(value),cell(c),old(original){}
    };
    double threshold=0.;
    Vec boxLo{},boxHi{},actualLo{},actualHi{};
    bool inRange(const Vec& p,double slack)const{
        const Vec q=p-mouth,frame{dot(q,u),dot(q,v),dot(q,dir)};
        for(unsigned a=0;a<3;++a){
            if(p[a]<boxLo[a]-slack||p[a]>boxHi[a]+slack||
               frame[a]<actualLo[a]-slack||frame[a]>actualHi[a]+slack)return false;
        }
        return true;
    }
    // Change only boundary cells. A candidate outside the old near group must
    // be current, owned original tissue, inside both original cut bounding
    // boxes, in a supported column, and within a short distance of the cut.
    void balance(Grid& g,std::vector<Cell>& cells,const decltype(SeverState::table) table) {
        if(!ready||cells.empty())return;

        using Tiles=std::unordered_map<Cell,core::Tile,core::CellHash>;
        Tiles stock;
        region.initialize(cells);
        boxLo=g.center({region.lo.x+1,region.lo.y+1,region.lo.z+1});
        boxHi=g.center({region.hi.x-1,region.hi.y-1,region.hi.z-1});
        auto put=[](Tiles& m,Cell c){const auto n=Grid::local(c);m[Grid::tileKey(c)].removed[n>>6]|=std::uint64_t(1)<<(n&63);};
        auto has=[](const Tiles& m,Cell c){auto it=m.find(Grid::tileKey(c));return it!=m.end()&&core::tileBit(it->second,c);};
        std::set<Cell> touched;
        Cell lastTile{-1,-1,-1};core::Tile* nativeMask=nullptr;
        for(Cell c:cells){
            const auto key=Grid::tileKey(c);
            if(!(key==lastTile)){
                auto entry=stock.try_emplace(key);
                nativeMask=&entry.first->second;lastTile=key;
                if(entry.second)touched.insert(key);
            }
            const auto bit=Grid::local(c);nativeMask->removed[bit>>6]|=std::uint64_t(1)<<(bit&63);
            if(!region.flags.empty())region.flags[region.index(c)]|=1;
        }
        const double a0=actualLo.x+.41*actualLo.z,a1=actualHi.x+.41*actualHi.z;
        const double b0=actualLo.y-.57*actualHi.z,b1=actualHi.y-.57*actualLo.z;
        relief[0].initialize(46*a0,46*b0,46*a1,46*b1,pattern);
        relief[1].initialize(92*a0-.37,92*b0-.74,92*a1+.37,92*b1+.74,pattern+1);
        relief[2].initialize(198*a0,198*b0,198*a1,198*b1,pattern+11);
        const auto dirty=core::expandDirtyTiles(touched);
        std::vector<Cell> tiles(dirty.begin(),dirty.end());
        const std::size_t batchSize=12,batchCount=(tiles.size()+batchSize-1)/batchSize;
        std::vector<std::vector<Ranked>> batches(batchCount);
        std::atomic<std::size_t> cursor{0};
        // The stock mask defines the extent. Only a short adjacent band can
        // trade places with a shallower lobe. Never excavate another distant
        // body island or the protected opposite side of a narrow fitted limb.
        const double band=std::min(6*g.pitch,.17*scale);
        core::parallelChunks(std::min(core::parallelWidth(),batchCount),1,[&](std::size_t,std::size_t,std::size_t){
          CraterNoiseRow row; TissueLookup tissue{g,table}; core::TileCache removed(g);
          for(std::size_t id;(id=cursor.fetch_add(1,std::memory_order_relaxed))<batchCount;){
          const auto first=id*batchSize,last=std::min(tiles.size(),first+batchSize);
          auto& out=batches[id];out.reserve(cells.size()*(last-first)/tiles.size()+16);
          for(std::size_t i=first;i<last;++i){
            const Cell t=tiles[i],base{t.x*8,t.y*8,t.z*8};
            if(!g.valid(base))continue;
            const auto* page=spatial->get(t);
            // Original centres expand their bounds by precisely 1e-8 in
            // inRange. Test those same world coordinates before visiting the
            // native solid lanes, including the positive edge of partial tiles.
            const Vec tileLow=g.center(base),tileHigh=g.center({base.x+7,base.y+7,base.z+7});
            if(tileLow.x>boxHi.x+1e-8||tileHigh.x<boxLo.x-1e-8||
               tileLow.y>boxHi.y+1e-8||tileHigh.y<boxLo.y-1e-8||
               tileLow.z>boxHi.z+1e-8||tileHigh.z<boxLo.z-1e-8)continue;
            const auto owned=stock.find(t);
            const auto* originals=owned==stock.end()?nullptr:&owned->second;
            const bool native=table&&(!g.ownership||g.ownership->pureTile(base));
            const SeverState::TileMaterial* occupied=nullptr;
            if(native){auto it=table->find(t);if(it==table->end())continue;occupied=&it->second;}
            const auto* oldRemoval=g.findTile(t);
            for(int x=0;x<8;++x){std::uint64_t bits=native?occupied->solid[x]:~std::uint64_t(0);
             if(oldRemoval)bits&=~oldRemoval->removed[x];
             while(bits){const unsigned j=head_burst::ctz64(bits);bits&=bits-1;
                const int y=int(j>>3),z=int(j&7);
                const Cell c{base.x+x,base.y+y,base.z+z};
                const bool old=originals&&core::tileBit(*originals,c);
                const Vec p=g.center(c);
                if(!old&&(!inRange(p,1e-8)||(!native&&(!g.valid(c)||removed.removed(c)||!tissue.solid(c)))))continue;
                const double depth=depthAt(p);
                if(!old&&depth>1e7)continue;
                double originalMargin;
                const unsigned bit=Grid::local(c);
                const double* known=page&&(page->valid[bit>>6]&(std::uint64_t(1)<<(bit&63)))?&page->value[bit]:nullptr;
                const double score=scoreAt(p,depth,originalMargin,true,&row,known);
                if(!old&&originalMargin< -band)continue;
                out.push_back({c,score,old});
            }}
          }
          }
        });
        std::vector<Ranked> rank;std::size_t allocated=0;
        for(const auto& b:batches)allocated+=b.size();
        rank.reserve(allocated);
        for(auto& b:batches)rank.insert(rank.end(),b.begin(),b.end());
        if(rank.size()<=cells.size()){ready=false;return;}
        const auto requested=cells.size();
        auto better=[](const Ranked& a,const Ranked& b){
            if(a.score!=b.score)return a.score>b.score;
            if(a.old!=b.old)return a.old;
            return a.cell<b.cell;
        };
        std::nth_element(rank.begin(),rank.begin()+requested-1,rank.end(),better);
        threshold=rank[requested-1].score;
        // All exchanged cells remain part of the same local cut. Flood only
        // proposed additions, from native retained members of the original
        // opening. Both sets are true, still-present simulation tissue.
        Tiles chosen,linked;
        const bool dense=!region.flags.empty() && std::all_of(rank.begin(),rank.begin()+requested,
            [&](const Ranked& r){const Cell c=r.cell;
                return c.x>region.lo.x&&c.y>region.lo.y&&c.z>region.lo.z&&c.x<region.hi.x&&c.y<region.hi.y&&c.z<region.hi.z;});
        auto test=[&](const Tiles& sparse,Cell c,unsigned char flag){
            return dense?region.marked(c,flag):has(sparse,c);
        };
        auto set=[&](Tiles& sparse,Cell c,unsigned char flag){
            if(dense)region.flags[region.index(c)]|=flag;
            else put(sparse,c);
        };
        std::vector<Cell> pending,extras;
        for(std::size_t j=0;j<requested;++j){auto& r=rank[j];set(chosen,r.cell,2);if(!r.old)extras.push_back(r.cell);}
        auto neighbours=[&](Cell c,auto&& visit){for(int a=0;a<3;++a)for(int s:{-1,1}){Cell n=c;(&n.x)[a]+=s;if(g.valid(n))visit(n);}};
        if(dense){
            const std::ptrdiff_t delta[6]={-std::ptrdiff_t(region.ny*region.nz),std::ptrdiff_t(region.ny*region.nz),
                                          -std::ptrdiff_t(region.nz),std::ptrdiff_t(region.nz),-1,1};
            for(Cell c:extras){
                auto* at=&region.flags[region.index(c)];
                bool root=false;for(auto step:delta)root=root||((at[step]&3)==3);
                if(root){*at|=4;pending.push_back(c);}
            }
            for(std::size_t j=0;j<pending.size();++j){
                const Cell c=pending[j];auto* at=&region.flags[region.index(c)];
                for(int n=0;n<6;++n)if((at[delta[n]]&7)==2){
                    at[delta[n]]|=4;Cell adjacent=c;(&adjacent.x)[n/2]+=(n&1)?1:-1;
                    pending.push_back(adjacent);
                }
            }
        }else{
        for(const Cell c:extras){
            bool root=false;neighbours(c,[&](Cell n){if(test(stock,n,1)&&test(chosen,n,2))root=true;});
            if(root){set(linked,c,4);pending.push_back(c);}
        }
        for(std::size_t j=0;j<pending.size();++j)neighbours(pending[j],[&](Cell n){
            if(!test(stock,n,1)&&test(chosen,n,2)&&!test(linked,n,4)){set(linked,n,4);pending.push_back(n);}
        });
        }
        const std::size_t lost=extras.size()-pending.size();
        if(lost){
            // Spend the volume on original cut cells before admitting any
            // detached island as part of this hit.
            std::vector<Ranked> replacement;
            for(std::size_t j=requested;j<rank.size();++j)if(rank[j].old)replacement.push_back(rank[j]);
            std::sort(replacement.begin(),replacement.end(),better);
            for(std::size_t j=0;j<lost;++j)set(chosen,replacement[j].cell,2);
        }
        std::vector<Cell> exact;
        exact.reserve(requested);
        std::vector<Cell> retained;
        retained.reserve(extras.size());
        for(const Cell c:cells){
            if(test(chosen,c,2))exact.push_back(c);
            else retained.push_back(c);
        }
        exact.insert(exact.end(),pending.begin(),pending.end());
        if(exact.size()!=requested){ready=false;return;}
        orderUniqueCraterCells(exact);cells.swap(exact);
        originalSet=g.ownership?std::move(exact):std::move(retained);originalTiles=std::move(stock);additionsOrdered=std::move(pending);
        materialTable=table;
    }


    bool cuts(const Vec& p)const{
        if(!ready)return false;
        double margin;
        return scoreAt(p,depthAt(p),margin)>=threshold&&margin>=-std::min(6*pitch,.17*scale);
    }
    // The fitted exterior crosses between original air and tissue centres.
    // Raster only its original, visible body triangles over an actual freshly
    // removed column. Pure air mask bits expose the existing fitted tissue.
    std::size_t skin(Grid& g,const std::vector<Cell>& fresh,const std::set<Cell>& changed,
                     const head_burst_record::Function& fn,const BulletSurfaceSource& src,std::set<Cell>& touched)const{
        if(!ready||!src.tree||src.tree->nodes.empty()||g.ownership||fresh.empty())return 0;
        std::unordered_map<Cell,core::Tile,core::CellHash> cut;
        LocalCraterCells freshBits;
        freshBits.initialize(fresh);
        CraterBackedBlocks backedBlocks;backedBlocks.initialize(fresh,freshBits,dir);
        if(!freshBits.flags.empty())for(Cell c:fresh)freshBits.flags[freshBits.index(c)]=1;
        else for(const Cell c:fresh){auto n=Grid::local(c);cut[Grid::tileKey(c)].removed[n>>6]|=std::uint64_t(1)<<(n&63);}
        const auto dirty=core::expandDirtyTiles(changed);
        auto toCell=[&](const Vec& p){return Cell{int(std::floor((p.x-g.origin.x)/g.pitch)),int(std::floor((p.y-g.origin.y)/g.pitch)),int(std::floor((p.z-g.origin.z)/g.pitch))};};
        auto freshAt=[&](const Vec& p){const auto c=toCell(p);if(!freshBits.flags.empty())return freshBits.marked(c,1);if(!g.valid(c))return false;auto k=cut.find(Grid::tileKey(c));return k!=cut.end()&&core::tileBit(k->second,c);};
        std::vector<Cell> work(dirty.begin(),dirty.end());
        std::vector<std::vector<Cell>> proposed(work.size());
        std::atomic<std::size_t> cursor{0};
        constexpr std::size_t group=8;
        core::parallelChunks(std::min(core::parallelWidth(),(work.size()+group-1)/group),1,[&](std::size_t,std::size_t,std::size_t){
            for(std::size_t start;(start=cursor.fetch_add(group,std::memory_order_relaxed))<work.size();){
            const auto end=std::min(start+group,work.size());
            for(auto i=start;i<end;++i){
                const Cell k=work[i],base{k.x*8,k.y*8,k.z*8};if(!g.valid(base))continue;
                std::uint64_t unsupported[8]{};
                bool anySupported=false;
                for(int x=0;x<2;++x)for(int y=0;y<2;++y)for(int z=0;z<2;++z){
                    const Cell key{2*k.x+x,2*k.y+y,2*k.z+z};
                    if(backedBlocks.possible(key)){anySupported=true;continue;}
                    const std::uint64_t blockMask=std::uint64_t(0x0f0f0f0f)<<(32*y+4*z);
                    for(int j=4*x;j<4*x+4;++j)unsupported[j]|=blockMask;
                }
                if(!anySupported)continue;
                core::Bounds box;const double tol=.92*g.pitch;
                box.lo=g.center(base)-Vec{tol,tol,tol};
                box.hi=g.center(base)+Vec{7*g.pitch+tol,7*g.pitch+tol,7*g.pitch+tol};
                const Vec centre{fn.B[0],fn.B[1],fn.B[2]};
                const Vec nearest{std::clamp(centre.x,box.lo.x,box.hi.x),
                                  std::clamp(centre.y,box.lo.y,box.hi.y),
                                  std::clamp(centre.z,box.lo.z,box.hi.z)};
                const Vec gap=nearest-centre;
                const double paddedRadius=fn.Rcut+1e-10;
                if(dot(gap,gap)>paddedRadius*paddedRadius)continue;
                std::vector<std::uint32_t> faces;src.tree->query(0,box,faces);
                if(faces.empty()||faces.size()>512)continue;
                core::Tile candidates{},excluded{};
                const auto* removed=g.findTile(k);
                const SeverState::TileMaterial* native=nullptr;
                if(materialTable){auto it=materialTable->find(k);if(it!=materialTable->end())native=&it->second;}
                std::uint64_t blocked[8]{};
                for(int lane=0;lane<8;++lane)blocked[lane]=unsupported[lane]|(removed?removed->removed[lane]:0)|(native?native->solid[lane]:0);
                const auto bodyEnd=std::partition(faces.begin(),faces.end(),[&](auto f){return f<src.bodyFaces;});
                if(bodyEnd==faces.begin())continue;
                for(auto face:faces){
                    const auto& f=src.tree->source.f[face];
                    const auto a=src.tree->source.v[f[0]].p,b=src.tree->source.v[f[1]].p,c=src.tree->source.v[f[2]].p;
                    const Vec plane=cross(b-a,c-a);
                    const double planeLimit=(tol+1e-10)*(tol+1e-10)*dot(plane,plane);
                    core::Bounds tri;tri.add(a);tri.add(b);tri.add(c);
                    const auto lo=toCell(tri.lo-Vec{tol,tol,tol}),hi=toCell(tri.hi+Vec{tol,tol,tol});
                    for(int x=std::max(base.x,lo.x);x<=std::min(base.x+7,hi.x);++x)
                    for(int y=std::max(base.y,lo.y);y<=std::min(base.y+7,hi.y);++y)
                    for(int z=std::max(base.z,lo.z);z<=std::min(base.z+7,hi.z);++z){
                        const Cell cell{x,y,z};const unsigned bit=Grid::local(cell);
                        const auto mask=std::uint64_t(1)<<(bit&63);
                        if((blocked[bit>>6]&mask)||!g.valid(cell)||(!materialTable&&g.rawMaterialBase(cell)))continue;
                        auto& existing=candidates.removed[bit>>6];
                        if(face<src.bodyFaces?bool(existing&mask):!bool(existing&mask)||bool(excluded.removed[bit>>6]&mask))continue;
                        const Vec p=g.center(cell);
                        const double planeDistance=dot(p-a,plane);
                        if(planeDistance*planeDistance>planeLimit)continue;
                        const Vec surface=core::closestTriangle(p,a,b,c);
                        const Vec delta=surface-p;if(dot(delta,delta)>tol*tol)continue;
                        if(face>=src.bodyFaces){excluded.removed[bit>>6]|=mask;continue;}
                        const auto label=src.tissue->sample(surface.x,surface.y,surface.z);
                        if(!label||label==16||label>=64)continue;
                        bool backed=false;
                        for(unsigned j=0;j<=6&&!backed;++j)backed=freshAt(surface+dir*(j*g.pitch));
                        if(backed&&cuts(surface))existing|=mask;
                    }
                }
                for(int x=0;x<8;++x){auto w=candidates.removed[x]&~excluded.removed[x];
                    while(w){const unsigned j=head_burst::ctz64(w);w&=w-1;proposed[i].push_back({base.x+x,base.y+int(j>>3),base.z+int(j&7)});}}
            }
            }
        });
        std::size_t n=0;for(auto& list:proposed)for(Cell c:list)if(g.erase(c)){++n;touched.insert(Grid::tileKey(c));}
        return n;
    }
};
