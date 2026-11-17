#pragma once
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <set>
#include <vector>
#include "TissueTear.h"

struct DamageTearVerdict {
    bool tear{},dangling{};std::uint16_t bone{};Vec point,axis;double remainingTissue{1},remainingBone{1};
    std::uint16_t probeBone{65535};double probeTissue{1},probeBoneLeft{1}; // the weakest touched station, tear or not (2026-10-17 read-out)
    double growth{};std::size_t crossing{},cells{},mask{};std::array<std::size_t,64> bySlot{};
    std::vector<ResolvedTunnel> records;std::set<Cell> dirty;std::size_t edgeCells{};double ms{},estimateMs{},carveMs{};const char* refusal{};
    double planeOffset{},tearRadius{};int rounds{};std::size_t remainingPixels{},woundPixels{};
    double selectMs{},planeMs{},sideMs{},floodMs{}; // phase times (test report)
    std::size_t pits{},pitTries{},pitBone{};double pitMs{};bool sheet{},forced{},pitRepairRefused{}; // R61 read-out
    std::size_t tissueTorn{},tissueLoose{},tissueFresh{},tissueTriangles{};double tissueMs{};const char* tissueRefusal{}; // R62 tissue tear
    bool tissueRan{}; // T1: the anatomy tear ran for this tear (release log: built or refused, and why)
    // damagecut: the cells that still held the two sides (a station tear: its sheet's standing cells; a holding-cells sever: H), the
    // intact connection there (Hi), their bone cells, whether the holding-cells search made this tear, its search time
    std::size_t bridgeCells{},holding{},holdingIntact{},holdingBone{},holdingBoneIntact{};bool general{};double holdMs{};
};

#ifdef GFY_VOXEL_WORKER_TESTS
#include <iostream>
inline void damageTearReport(const DamageTearVerdict&v){
    std::cout<<"{\"damage_tear\":{\"tear\":"<<(v.tear?"true":"false")<<",\"bone\":"<<v.bone<<",\"channels\":"<<v.crossing<<",\"rounds\":"<<v.rounds
        <<",\"tear_radius_mm\":"<<v.tearRadius*1000<<",\"plane_offset_mm\":"<<v.planeOffset*1000<<",\"remaining_px\":"<<v.remainingPixels<<",\"wound_px\":"<<v.woundPixels
        <<",\"cells\":"<<v.cells<<",\"mask\":"<<v.mask<<",\"records\":"<<v.records.size()<<",\"tissue_left\":"<<v.remainingTissue<<",\"bone_left\":"<<v.remainingBone
        <<",\"select_ms\":"<<v.selectMs<<",\"plane_ms\":"<<v.planeMs<<",\"side_ms\":"<<v.sideMs<<",\"flood_ms\":"<<v.floodMs<<",\"estimate_ms\":"<<v.estimateMs<<",\"carve_ms\":"<<v.carveMs<<",\"pits\":"<<v.pits<<",\"pit_tries\":"<<v.pitTries<<",\"pit_bone\":"<<v.pitBone<<",\"pit_ms\":"<<v.pitMs
        <<",\"pit_repair_refused\":"<<(v.pitRepairRefused?"true":"false")<<",\"sheet\":"<<(v.sheet?"true":"false")<<",\"forced\":"<<(v.forced?"true":"false")
        <<",\"edge_cells\":"<<v.edgeCells<<",\"tissue_torn\":"<<v.tissueTorn<<",\"tissue_loose\":"<<v.tissueLoose<<",\"tissue_fresh\":"<<v.tissueFresh<<",\"tissue_triangles\":"<<v.tissueTriangles<<",\"tissue_ms\":"<<v.tissueMs<<",\"tissue_refusal\":\""<<(v.tissueRefusal?v.tissueRefusal:"")<<"\""<<",\"ms\":"<<v.ms<<",\"refusal\":\""<<(v.refusal?v.refusal:"")<<"\"}}"<<std::endl;}
#define GFY_DAMAGE_TEAR_REPORT(v) damageTearReport(v)
#else
#define GFY_DAMAGE_TEAR_REPORT(v) ((void)0)
#endif
namespace damage_tear {
inline double hash01(std::uint64_t seed,std::uint64_t a,std::uint64_t b){
    const std::uint64_t h=core::quantMix(seed^core::quantMix(a*0x9E3779B97F4A7C15ull^core::quantMix(b*0xC2B2AE3D27D4EB4Full)));
    return double(h>>11)*(1./9007199254740992.);
}
// One straight tear channel through (x0,y0) of the tear plane (basis u,v about C), open at both ends beyond the
// reach circle, with its in-plane half widths (the edge profile's minimum over depth on each side).
struct Channel {Tunnel cut;double c{},s{},plus{},minus{};std::uint64_t seed{};};
inline Channel makeChannel(Vec C,Vec n,Vec u,Vec v,double x0,double y0,double theta,double radius,double reach,std::uint64_t seed){
    Channel ch;ch.c=std::cos(theta);ch.s=std::sin(theta);ch.seed=seed;
    const Vec d=unit(u*ch.c+v*ch.s);
    const double b=x0*ch.c+y0*ch.s,q=x0*x0+y0*y0-reach*reach,root=std::sqrt(std::max(0.,b*b-q));
    const double t0=-b-root,t1=-b+root;
    ch.cut=Tunnel{C+u*x0+v*y0+d*t0,d,radius,t1-t0,radius};ch.cut.exit_distance=t1-t0;ch.cut.exit_mult=1;
    ch.cut.setEdgeProfile(seed);
    const Vec w=cross(n,d);
    auto side=[&](Vec dir){const double eu=dot(dir,ch.cut.edge_u),ev=dot(dir,ch.cut.edge_v);
        return ch.cut.edge.enabled()?std::min(ch.cut.edge.radiusMultiplier(eu,ev,0.),ch.cut.edge.radiusMultiplier(eu,ev,1.)):1.;};
    ch.plus=radius*side(w);ch.minus=radius*side(w*-1.);
    return ch;
}
// Exact raster of one cutter restricted to [low,high]: the block-native planner every cut uses, or exact
// per-cell membership when the planner refuses (owned grids, scratch). Journaled via Grid::erase/TileWindow.
inline void carve(Grid&g,const Tunnel&cut,Cell low,Cell high,const painter_detail::TissueField*countField,
    std::vector<Cell>&boundary,std::set<Cell>&touched,std::size_t&mask,std::size_t&solid,std::array<std::size_t,64>&bySlot){
    checkedCutterCells(low,high);
    auto plan=painter_detail::planTileTunnel(g,cut,low,high,countField,[&](std::size_t n,std::size_t grain,auto&&body){core::parallelChunks(n,grain,body);});
    if(plan.supported){
        if(!plan.freshMask)return;
        mask+=plan.freshMask;solid+=plan.freshSolid;for(unsigned s=0;s<64;++s)bySlot[s]+=plan.freshBySlot[s];
        std::vector<core::Tile*> destinations(plan.touched.size());core::TileWindow window(g,low,high);
        for(std::size_t i=0;i<plan.touched.size();++i)destinations[i]=&window.tile(plan.touched[i]);
        core::parallelChunks(plan.touched.size(),4096,[&](std::size_t first,std::size_t last,std::size_t){
            for(std::size_t i=first;i<last;++i)*destinations[i]=plan.stages[plan.index(plan.touched[i])].finalMask;});
        touched.insert(plan.touched.begin(),plan.touched.end());boundary.insert(boundary.end(),plan.boundary.begin(),plan.boundary.end());
        return;}
    for(int x=low.x;x<=high.x;++x)for(int y=low.y;y<=high.y;++y)for(int z=low.z;z<=high.z;++z){const Cell c{x,y,z};
        if(!cut.contains(g.center(c))||!g.erase(c))continue;boundary.push_back(c);touched.insert(Grid::tileKey(c));++mask;
        if(const auto label=g.materialBase(c)){++solid;const auto slot=g.material_slots&&label<g.material_slots->size()?unsigned((*g.material_slots)[label]&63u):1u;++bySlot[slot];}}
}
}

// cuts: every accepted cut of this body (committed history + this batch), oldest first (seeds the tear).
// Static tissue at a cell from the model's tile masks (SeverState::table) where ownership does not decide; one table
// lookup per tile run of cells. Exactly g.valid(c)&&g.solidBase(c).
struct TissueLookup {
    const Grid&g;const std::unordered_map<Cell,SeverState::TileMaterial,core::CellHash>*table;Cell key{INT_MIN,INT_MIN,INT_MIN};const SeverState::TileMaterial*m=nullptr;
    bool solid(const Cell&c){if(!g.valid(c))return false;
        if(!table||(g.ownership&&!g.ownership->pureTile(c)))return g.solidBase(c)!=0;
        const Cell k=Grid::tileKey(c);if(!(k==key)){key=k;const auto it=table->find(k);m=it==table->end()?nullptr:&it->second;}
        if(!m)return false;const auto l=Grid::local(c);return ((m->solid[l/64]>>(l%64))&1u)!=0;}
};
struct TearPlane {Vec point,normal,a,b;bool sheet=false;Vec entry{},dir{},side{};double half=0,width=0,reach=0,inlet=0;std::uint64_t seed=0;};
// R62 (grenade severs by force, BlastSevers.h): flesh_gone_only - a blast batch tears only stations whose flesh is all but gone
// (its designed severs are the forced tears of the sites it picked); tear_radius - the rip channels' radius for forced tears (0:
// the default), wider channels cross an intact limb with fewer cuts (the anatomy tear hides them behind the torn ends).
struct TearGuide {std::vector<TearPlane> planes;bool tilt=false,exact_only=false,flesh_gone_only=false;double tear_radius=0;};
// R61: a tear at a given station along a given blade (a torn blade that crossed the limb), without the failed-station
// selection; it still tears only where the sheet is open across the whole section.
struct TearForce {const SeverSegment*seg{};const SeverStation*st{};const TearPlane*plane{};
    // R62 head fractures (HeadFracture.h): the section keeps only the plane pixels inside every half-space (point, normal) and
    // within keepRadius of keepCenter (0: no limit); noTissueTear skips the anatomy tear (its rule that erases kept tissue not
    // joined to the cylinder's edge would erase whole skull pieces: they lie inside its cylinder).
    const std::vector<std::pair<Vec,Vec>>*keep{};Vec keepCenter{};double keepRadius{};bool noTissueTear{};double tissueDepthScale=1.;bool keepIslands=false;int headTear=0;
    // melee (2026-11-12, value 3): a blade's clean cut - the tear lies on the blade's plane (through plane->point; tilted at most 60
    // degrees off square), needs no open kerf, and its decide-once surface is flat with a shallow even face (TearDecideTear.h)
    bool clean=false;
    // rpgblast: a trunk tear (a blast's waist / chest split) - every tissue of its plane inside the cylinder is its footprint, also
    // what the decide-once tear calls another limb's ("foreign": on the chest the trunk's own sides under the arms), and the first
    // try already takes the full cylinder
    bool wholeSection=false;
    // raggedsever: a killing shot's tear through the damage (damage_tear::killPlane) - the decide-once tear makes one try on its own
    // plane (never wider, square, further out or with the ring seam) and, when that does not part the limb, refuses without a change
    bool planeOnly=false;}; // head fractures: keep whole pieces (the split drops crumbs), the burst skull's tear profile
namespace damage_tear {
// R68: the envelope of standingReach's scratch (columns of the station box, standing z runs); beyond it the serial flood answers.
struct ReachEnvelope {std::size_t columns,runs;};
#ifdef GFY_VOXEL_WORKER_TESTS
inline ReachEnvelope reachEnvelopeTest{0,0}; // TEST-ONLY: non-zero fields replace the envelope's
#endif
inline ReachEnvelope reachEnvelope(){ReachEnvelope e{std::size_t(1)<<21,std::size_t(1)<<21};
#ifdef GFY_VOXEL_WORKER_TESTS
 if(reachEnvelopeTest.columns)e.columns=reachEnvelopeTest.columns;if(reachEnvelopeTest.runs)e.runs=reachEnvelopeTest.runs;
 if(const char*k=std::getenv("GFY_REACH_ENVELOPE_RUNS"))e.runs=std::size_t(std::strtoull(k,nullptr,10)); // whole-run fallback gates
#endif
 return e;}
// Whether flood(from) over the standing cells of an nx*ny*nz box (cell j=(x*ny+y)*nz+z standing: side[j]!=-2 and not removed)
// would reach the other face: 1 a 6-connected component of standing cells holds a cell with side==from and one with side==-from,
// 0 none does, -1 outside the envelope (nothing large was allocated; the caller floods). startCells / goalCells: every cell
// with side==from / side==-from. makeRemoved() gives each thread its own removed(x,y,z) reader.
template<class MakeRemoved>int standingReach(int nx,int ny,int nz,const std::vector<signed char>&side,signed char from,
    const std::vector<std::size_t>&startCells,const std::vector<std::size_t>&goalCells,const MakeRemoved&makeRemoved,const ReachEnvelope&env){
    const std::size_t NY=std::size_t(ny),NZ=std::size_t(nz);
    {auto removed=makeRemoved();
     auto standing=[&](std::size_t j){if(side[j]==-2)return false;const int z=int(j%NZ),y=int((j/NZ)%NY),x=int(j/(NZ*NY));return !removed(x,y,z);};
     // flood(from) returns false at once without a standing start cell, and reaches only a standing goal cell
     if(std::none_of(startCells.begin(),startCells.end(),standing)||std::none_of(goalCells.begin(),goalCells.end(),standing))return 0;}
    const std::size_t columns=std::size_t(nx)*NY;if(columns>env.columns)return -1;
    // runs per column counted (nothing per run stored) and admitted before any run storage exists
    std::vector<std::uint32_t> first(columns+1,0);std::atomic<std::size_t> counted{0};std::atomic<bool> over{false};
    core::parallelChunks(std::size_t(nx),1,[&](std::size_t x0,std::size_t x1,std::size_t){auto removed=makeRemoved();
        for(int x=int(x0);x<int(x1)&&!over.load(std::memory_order_relaxed);++x){std::size_t row=0;
            for(int y=0;y<ny;++y){const std::size_t col=std::size_t(x)*NY+std::size_t(y),base=col*NZ;std::uint32_t c=0;bool in=false;
                for(int z=0;z<nz;++z){const bool st=side[base+std::size_t(z)]!=-2&&!removed(x,y,z);if(st&&!in)++c;in=st;}
                first[col+1]=c;row+=c;}
            if(counted.fetch_add(row,std::memory_order_relaxed)+row>env.runs)over.store(true,std::memory_order_relaxed);}});
    if(over.load())return -1;
    for(std::size_t c=0;c<columns;++c)first[c+1]+=first[c];
    const std::size_t R=first[columns];if(R>env.runs)return -1;
#ifdef GFY_VOXEL_WORKER_TESTS
    if(std::getenv("GFY_REACH_LOG"))std::cerr<<"reach_stats box="<<nx<<"x"<<ny<<"x"<<nz<<" columns="<<columns<<" runs="<<R<<std::endl;
#endif
    if(!R)return 0;
    // the runs written straight into their columns' slices (the same predicate on the same cells: the grid is not written here)
    struct Run{std::int32_t z0,z1;std::uint8_t start,goal;};std::vector<Run> runs(R);
    core::parallelChunks(std::size_t(nx),1,[&](std::size_t x0,std::size_t x1,std::size_t){auto removed=makeRemoved();
        for(int x=int(x0);x<int(x1);++x)for(int y=0;y<ny;++y){const std::size_t col=std::size_t(x)*NY+std::size_t(y),base=col*NZ;std::uint32_t k=first[col];
            for(int z=0;z<nz;){if(side[base+std::size_t(z)]==-2||removed(x,y,z)){++z;continue;}
                if(k>=first[col+1])throw std::runtime_error("tear_reach_runs");
                Run r{z,z,0,0};
                for(;z<nz;++z){const std::size_t q=base+std::size_t(z);if(side[q]==-2||removed(x,y,z))break;
                    r.z1=z;if(side[q]==from)r.start=1;else if(side[q]==-from)r.goal=1;}
                runs[k++]=r;}
            if(k!=first[col+1])throw std::runtime_error("tear_reach_runs");}});
    std::unique_ptr<std::atomic<std::uint32_t>[]> par(new std::atomic<std::uint32_t>[R]);
    core::parallelChunks(R,1u<<14,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t k=a;k<b;++k)par[k].store(std::uint32_t(k),std::memory_order_relaxed);});
    auto find=[&](std::uint32_t a){for(;;){std::uint32_t q=par[a].load(std::memory_order_relaxed);if(q==a)return a;const std::uint32_t g2=par[q].load(std::memory_order_relaxed);
        if(g2!=q)par[a].compare_exchange_weak(q,g2,std::memory_order_relaxed);a=g2;}};
    auto unite=[&](std::uint32_t a,std::uint32_t b){for(;;){a=find(a);b=find(b);if(a==b)return;if(a<b)std::swap(a,b);std::uint32_t e=a;
        if(par[a].compare_exchange_strong(e,b,std::memory_order_relaxed))return;}};
    auto link=[&](std::size_t c,std::size_t d){std::uint32_t a=first[c],b=first[d];const std::uint32_t ae=first[c+1],be=first[d+1];
        while(a<ae&&b<be){const Run&A=runs[a];const Run&B=runs[b];if(A.z1<B.z0){++a;continue;}if(B.z1<A.z0){++b;continue;}unite(a,b);if(A.z1<B.z1)++a;else ++b;}};
    core::parallelChunks(std::size_t(nx),1,[&](std::size_t x0,std::size_t x1,std::size_t){
        for(int x=int(x0);x<int(x1);++x)for(int y=0;y<ny;++y){const std::size_t col=std::size_t(x)*NY+std::size_t(y);if(first[col]==first[col+1])continue;
            if(x>0)link(col,col-NY);if(y>0)link(col,col-1);}});
    std::unique_ptr<std::atomic<std::uint8_t>[]> flags(new std::atomic<std::uint8_t>[R]);
    core::parallelChunks(R,1u<<14,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t k=a;k<b;++k)flags[k].store(0,std::memory_order_relaxed);});
    core::parallelChunks(R,1u<<12,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t k=a;k<b;++k){const std::uint8_t f=std::uint8_t((runs[k].start?1:0)|(runs[k].goal?2:0));
        if(f)flags[find(std::uint32_t(k))].fetch_or(f,std::memory_order_relaxed);}});
    std::atomic<bool> both{false};
    core::parallelChunks(R,1u<<14,[&](std::size_t a,std::size_t b,std::size_t){for(std::size_t k=a;k<b&&!both.load(std::memory_order_relaxed);++k)if(flags[k].load(std::memory_order_relaxed)==3)both.store(true,std::memory_order_relaxed);});
    return both.load()?1:0;}
// standingReach's answer (reach()), else the serial flood's (flood()): outside the envelope (-1) or when the new scratch cannot be
// allocated (std::bad_alloc only; every other exception travels on)
template<class Reach,class Flood>bool reachOrFlood(const Reach&reach,const Flood&flood){int r=-1;
    try{r=reach();}catch(const std::bad_alloc&){r=-1;}
    return r>=0?r==1:flood();}
}
#include "TearDecideTear.h" // mask-only wounds, slice 3
inline DamageTearVerdict damageTear(Grid&g,const std::vector<SeverSegment>&segments,SeverState&state,const std::set<Cell>&dirty,
    double severScale,const std::vector<const ResolvedTunnel*>&cuts,const painter_detail::TissueField*countField,
    const std::set<Cell>*changed=nullptr, // changed: tiles modified since the last call (default: dirty)
    std::vector<StationRef>*touchedCache=nullptr, // repeated calls over one dirty set: the touched stations, found once
    const TearGuide*guide=nullptr, // grenade batches: blade planes and the random tilt
    const TearForce*force=nullptr, // R61: a crossed limb's station and blade (no selection)
    const painter_detail::TissueField*closure=nullptr, // R62: the render closure field: the anatomy shapes the torn ends (TissueTear.h)
    const Vec*impact=nullptr){ // R62: the shot / blast direction (rest space) - it leans and drags the torn ends
    using namespace damage_tear;DamageTearVerdict v;const auto t0=core::Clock::now();
    if(segments.empty()||dirty.empty()||cuts.empty())return v;
    // ---- station selection: identical to severCheck (weakest failing station touched by this batch)
    std::size_t total=0;for(const auto&seg:segments)total+=seg.stations.size();
    if(state.failed.size()!=total)state.failed.assign(total,0);
    Cell dlo=*dirty.begin(),dhi=dlo;for(const Cell&t:dirty){dlo={std::min(dlo.x,t.x),std::min(dlo.y,t.y),std::min(dlo.z,t.z)};dhi={std::max(dhi.x,t.x),std::max(dhi.y,t.y),std::max(dhi.z,t.z)};}
    const double T=std::clamp(.45-.1*(severScale-1),.25,.6);
    const SeverSegment*seg=nullptr;const SeverStation*st=nullptr;bool dangling=false;double rt=1,rb=1,worst=1e30;
    std::size_t index=0;
    // Stations this batch touched; the ones never measured get their first (whole-box) measurement in parallel.
    std::vector<StationRef> touchedList;state.ensure(total);
    struct Failing {const SeverSegment*s;const SeverStation*st;double t,b;bool dang;};std::vector<Failing> failing;
    if(!force){ // R61: a forced blade tear skips the selection
    const int TX=(g.size.x+7)/8,TY=(g.size.y+7)/8,TZ=(g.size.z+7)/8;std::vector<std::uint64_t> dirtyBits;
    auto bitOf=[&](const Cell&t){return (std::size_t(t.x)*std::size_t(TY)+std::size_t(t.y))*std::size_t(TZ)+std::size_t(t.z);};
    auto isDirty=[&](const Cell&t){if(t.x<0||t.y<0||t.z<0||t.x>=TX||t.y>=TY||t.z>=TZ)return false;const auto i=bitOf(t);return ((dirtyBits[i>>6]>>(i&63))&1u)!=0;};
    if(dirty.size()>256&&!(touchedCache&&!touchedCache->empty())){dirtyBits.assign((std::size_t(TX)*TY*TZ+63)/64,0);
        for(const Cell&t:dirty)if(t.x>=0&&t.y>=0&&t.z>=0&&t.x<TX&&t.y<TY&&t.z<TZ){const auto i=bitOf(t);dirtyBits[i>>6]|=std::uint64_t(1)<<(i&63);}}
    if(touchedCache&&!touchedCache->empty()){for(const auto&ref:*touchedCache)if(!state.failed[ref.k])touchedList.push_back(ref);}
    else{
    for(const auto&s:segments)for(const auto&station:s.stations){const std::size_t k=index++;if(state.failed[k])continue;
        if(station.tileLo.x>dhi.x||station.tileHi.x<dlo.x||station.tileLo.y>dhi.y||station.tileHi.y<dlo.y||station.tileLo.z>dhi.z||station.tileHi.z<dlo.z)continue;
        bool touched=false;
        if(!dirtyBits.empty()){for(int tx=station.tileLo.x;tx<=station.tileHi.x&&!touched;++tx)for(int ty=station.tileLo.y;ty<=station.tileHi.y&&!touched;++ty)for(int tz=station.tileLo.z;tz<=station.tileHi.z;++tz)if(isDirty(Cell{tx,ty,tz})){touched=true;break;}}
        else for(const Cell&t:dirty)if(t.x>=station.tileLo.x&&t.x<=station.tileHi.x&&t.y>=station.tileLo.y&&t.y<=station.tileHi.y&&t.z>=station.tileLo.z&&t.z<=station.tileHi.z){touched=true;break;}
        if(touched)touchedList.push_back({&s,&station,k});}
    if(touchedCache)*touchedCache=touchedList;}
    const double listMs=core::ms(t0);
    updateStations(g,touchedList,state,changed?*changed:dirty);
    static const bool selectLog=std::getenv("GFY_TEAR_SELECT_LOG")!=nullptr;
    if(selectLog)std::cerr<<"tear_select_phase stations="<<touchedList.size()<<" dirty="<<dirty.size()<<" changed="<<(changed?changed->size():dirty.size())<<" list_ms="<<listMs<<" update_ms="<<core::ms(t0)-listMs<<std::endl;
    for(const auto&ref:touchedList){const auto&s=*ref.seg;const auto&station=*ref.st;const std::size_t k=ref.k;
        double removedTissue,removedBone;stationRemoved(state,k,removedTissue,removedBone);
        const double tissue=std::clamp(1-removedTissue/station.intactTissue,0.,1.);
        const double bone=station.intactBone>1e-7?std::clamp(1-removedBone/station.intactBone,0.,1.):tissue;
        if(v.probeBone==65535||tissue+bone<v.probeTissue+v.probeBoneLeft){v.probeBone=s.bone;v.probeTissue=tissue;v.probeBoneLeft=bone;}
        const bool fleshGone=tissue<=.15;
        const bool goneOnly=guide&&guide->flesh_gone_only;
        const bool newRules=!tear_decide::bridgeOff();
        const bool tear=newRules?(goneOnly?(fleshGone&&bone<=.4):(tissue<=T&&bone<=.4)):(goneOnly?fleshGone:((tissue<=T&&bone<=.4)||fleshGone)),
            dang=!newRules&&!goneOnly&&!tear&&bone<=.25&&tissue<=T+.15;
        if(!(tear||dang))continue;
        if(guide)failing.push_back({&s,&station,tissue,bone,dang});
        if(tissue+bone<worst){worst=tissue+bone;seg=&s;st=&station;dangling=dang;rt=tissue;rb=bone;}}
    }
    // A blade that crossed a failed station decides where the limb tears: the failed station nearest its plane, among
    // those its slot reaches (its entry line within a limb radius + 4 cm of the station).
    bool guided=false;Vec guideNormal{},guidePoint{};const TearPlane*guidePlane=nullptr;
    if(force){seg=force->seg;st=force->st;rt=rb=0;guided=true;guideNormal=force->plane->normal;guidePoint=force->plane->point;guidePlane=force->plane;v.forced=true;}
    else if(seg&&guide&&!guide->planes.empty()){double best=1e30;
        for(const auto&f:failing)for(const auto&pl:guide->planes){
            const double off=std::abs(dot(f.st->center-pl.point,pl.normal));if(off>f.s->radius)continue;
            const Vec ab=pl.b-pl.a;const double L2=dot(ab,ab);const double t=L2>1e-12?std::clamp(dot(f.st->center-pl.a,ab)/L2,0.,1.):0.;
            if(length(f.st->center-(pl.a+ab*t))>f.s->radius+.04)continue;
            if(off<best){best=off;seg=f.s;st=f.st;rt=f.t;rb=f.b;dangling=f.dang;guideNormal=pl.normal;guidePoint=pl.point;guided=true;guidePlane=&pl;}}}
    if(guide&&guide->exact_only&&!guided)seg=nullptr;
    if(!seg){
        // damagecut: no station failed - a bottleneck in any direction (HoldingCut.h: value 3, not a forced tear, not an exact blade sheet)
        if(!force&&holding_cut::wanted(guide,closure)&&holding_cut::sever(g,segments,state,dirty,severScale,cuts,guide,closure,impact,v,t0))return v;
#ifdef GFY_VOXEL_WORKER_TESTS
        if(std::getenv("GFY_TEAR_SELECT_LOG"))std::cout<<"tear_select touched="<<touchedList.size()<<" dirty="<<dirty.size()<<" ms="<<core::ms(t0)<<std::endl;
#endif
        return v;}
    v.selectMs=core::ms(t0);auto phaseStart=core::Clock::now();
    std::uint64_t seed=0x7EA5A11CEull;for(const ResolvedTunnel*r:cuts)if(r)seed=core::quantMix(seed^r->edge_seed^(r->sequence*0x9E3779B97F4A7C15ull));
    // The tear plane's normal: square to the bone (bullets), along the blade that crossed here (at most 60 degrees
    // from square, so the plane still crosses the limb), or a seeded 15-45 degree tilt (other blast tears).
    Vec n=seg->axis;
    if(guided){Vec m=unit(dot(guideNormal,n)<0?guideNormal*-1.:guideNormal);const double c=dot(m,n);
        if(c<.5&&!(guide&&guide->exact_only)){const Vec p=unit(m-n*c);m=unit(n*.5+p*.8660254037844386);}
        if(std::isfinite(m.x)&&std::isfinite(m.y)&&std::isfinite(m.z))n=m;}
    else if(guide&&guide->tilt){const Vec a0=unit(cross(n,std::abs(n.x)<.8?Vec{1,0,0}:Vec{0,1,0})),b0=cross(n,a0);
        const double th=(15.+30.*hash01(seed,77,1))*3.141592653589793/180,ph=6.283185307179586*hash01(seed,77,2);
        n=unit(n*std::cos(th)+(a0*std::cos(ph)+b0*std::sin(ph))*std::sin(th));}
    v.bone=seg->bone;v.point=st->center;v.axis=n;v.remainingTissue=rt;v.remainingBone=rb;v.dangling=dangling;
    const double band=.014,Rb=seg->radius+.008,pitch=g.pitch;
    const Vec u=unit(cross(n,std::abs(n.x)<.8?Vec{1,0,0}:Vec{0,1,0})),w2=cross(n,u);
    auto cellOf=[&](Vec p){return Cell{int(std::floor((p.x-g.origin.x)/pitch)),int(std::floor((p.y-g.origin.y)/pitch)),int(std::floor((p.z-g.origin.z)/pitch))};};
    // ---- the tear plane: the offset within the station slab where the wounds left the least tissue
    const int M=int(std::ceil(Rb/pitch));
    struct Pixel {float x,y;};std::vector<Pixel> remaining,wound;
    auto sample=[&](const Vec&C,std::vector<Pixel>*rem,std::vector<Pixel>*wnd){std::size_t count=0;TissueLookup tissue{g,state.table};core::TileCache removedCache(g); // speed step
        for(int i=-M;i<=M;++i)for(int j=-M;j<=M;++j){const double x=i*pitch,y=j*pitch;if(x*x+y*y>Rb*Rb)continue;
            const Cell c=cellOf(C+u*x+w2*y);if(!tissue.solid(c))continue;
            if(removedCache.removed(c)){if(wnd)wnd->push_back({float(x),float(y)});}
            else{++count;if(rem)rem->push_back({float(x),float(y)});}}
        return count;};
    const bool exactPlane=guided&&((guide&&guide->exact_only)||(force&&force->clean)); // melee: a clean blade cut lies on its plane
    const Vec planeCenter=exactPlane?st->center-n*dot(st->center-guidePoint,n):st->center;
    double offset=0;{std::vector<double> offsets;for(double o=-seg->halfWidth;o<=seg->halfWidth+1e-9;o+=.002)offsets.push_back(o);if(exactPlane)offsets.assign(1,0.);
        // The offsets' tissue counts read the grid only: counted side by side, scanned in order with the same strict `<`.
        std::vector<std::size_t> counts(offsets.size());
        core::parallelChunks(offsets.size(),1,[&](std::size_t first,std::size_t last,std::size_t){for(std::size_t i=first;i<last;++i)counts[i]=sample(planeCenter+n*offsets[i],nullptr,nullptr);});
        std::size_t best=~std::size_t(0);for(std::size_t i=0;i<offsets.size();++i)if(counts[i]<best){best=counts[i];offset=offsets[i];}}
    const Vec C=planeCenter+n*offset;v.point=C;v.planeOffset=offset;v.planeMs=core::ms(phaseStart);phaseStart=core::Clock::now();
    // R61: an exact blade tear reads its cross-section on the blade's torn sheet: the centre surface lies off the plane
    // by the sheet's relief (RaggedBlade::disp along the blade normal), inside the kerf wherever the blade reached.
    RaggedBlade sheetBlade;double sheetSign=1;
    if(exactPlane&&guidePlane&&guidePlane->sheet){sheetBlade=RaggedBlade(guidePlane->seed,guidePlane->half,guidePlane->width,guidePlane->reach);
        sheetSign=dot(guidePlane->normal,n)<0?-1.:1.;v.sheet=sheetBlade.active;}
    auto midAt=[&](double x,double y)->double{if(!sheetBlade.active)return 0.;const Vec q=C+u*x+w2*y-guidePlane->entry;
        return (sheetBlade.disp(dot(q,guidePlane->dir),dot(q,guidePlane->side))-dot(q,guidePlane->normal))*sheetSign;};
    // The limb's whole cross-section at the plane, not a fixed-radius disk (in-game 2026-09-27: a neck tear left the
    // tissue outside radius+8 mm standing, so the head stayed attached): tissue and wound pixels connected (4-way, in
    // the plane) to the central disk, out to 2.2 radii. Unconnected tissue in the plane (another body part) is not cut.
    const double Rs=std::max(Rb+.03,2.2*seg->radius+.008);const int Ms=int(std::ceil(Rs/pitch)),Wd=2*Ms+1;
    std::vector<std::uint8_t> plane(std::size_t(Wd)*Wd,0); // 1 standing tissue, 2 wound (removed tissue), |4 in the cross-section
    auto keepOut=[&](const Vec&p){if(!force)return false;if(force->keepRadius>0&&length(p-force->keepCenter)>force->keepRadius)return true;
        if(force->keep)for(const auto&h:*force->keep)if(dot(p-h.first,h.second)<0)return true;return false;};
    // Rows of the cross-section filled side by side (each cell written once; reads only).
    core::parallelChunks(std::size_t(Wd),8,[&](std::size_t first,std::size_t last,std::size_t){TissueLookup tissue{g,state.table};core::TileCache removedCache(g); // speed step
     for(int i=int(first);i<int(last);++i)for(int j=0;j<Wd;++j){const double x=(i-Ms)*pitch,y=(j-Ms)*pitch;if(x*x+y*y>Rs*Rs)continue;
        if(keepOut(C+u*x+w2*y))continue;
        const Cell c=cellOf(C+u*x+w2*y+n*midAt(x,y));if(!tissue.solid(c))continue;plane[std::size_t(i)*Wd+j]=removedCache.removed(c)?2:1;}});
    {std::vector<std::size_t> open;
     for(int i=0;i<Wd;++i)for(int j=0;j<Wd;++j){const double x=(i-Ms)*pitch,y=(j-Ms)*pitch;auto&p=plane[std::size_t(i)*Wd+j];
        if(p&&x*x+y*y<=Rb*Rb){p|=4;open.push_back(std::size_t(i)*Wd+j);}}
     while(!open.empty()){const std::size_t q=open.back();open.pop_back();const int i=int(q/Wd),j=int(q%Wd);
        const int nb[4][2]={{i-1,j},{i+1,j},{i,j-1},{i,j+1}};
        for(const auto&m:nb){if(m[0]<0||m[1]<0||m[0]>=Wd||m[1]>=Wd)continue;auto&p=plane[std::size_t(m[0])*Wd+m[1]];if(!p||(p&4))continue;p|=4;open.push_back(std::size_t(m[0])*Wd+m[1]);}}}
    double sectionRadius=0; // R62: how far the limb's connected section reaches from the tear centre (the tissue tear's cylinder)
    for(int i=0;i<Wd;++i)for(int j=0;j<Wd;++j){const auto p=plane[std::size_t(i)*Wd+j];if(!(p&4))continue;const Pixel px{float((i-Ms)*pitch),float((j-Ms)*pitch)};
        sectionRadius=std::max(sectionRadius,std::sqrt(double(px.x)*px.x+double(px.y)*px.y)+pitch);
        if((p&3)==1)remaining.push_back(px);else wound.push_back(px);}
    // BFS domain: the cross-section dilated by 6 mm (tissue just above/below the plane that is not in it).
    std::vector<std::uint8_t> domain(plane.size(),0);
    // speed step: value 3 builds the domain only when its decide-once tear falls back (only today's tear reads it)
    auto buildDomain=[&]{const int r=int(std::ceil(.006/pitch));std::vector<std::uint8_t> rows(plane.size(),0);
     for(int i=0;i<Wd;++i)for(int j=0;j<Wd;++j)if(plane[std::size_t(i)*Wd+j]&4)
        for(int b=std::max(0,j-r);b<=std::min(Wd-1,j+r);++b)rows[std::size_t(i)*Wd+b]=1;
     for(int i=0;i<Wd;++i)for(int j=0;j<Wd;++j)if(rows[std::size_t(i)*Wd+j])
        for(int a=std::max(0,i-r);a<=std::min(Wd-1,i+r);++a)domain[std::size_t(a)*Wd+j]=1;};
    const bool domainLate=mask_only_mode_value()==3;if(!domainLate)buildDomain();
    // core::roundHalfAway = std::lround without the library call (the domain test runs for every slab cell)
    auto inDomain=[&](const Vec&d){const int i=int(core::roundHalfAway(dot(d,u)/pitch))+Ms,j=int(core::roundHalfAway(dot(d,w2)/pitch))+Ms;
        return i>=0&&j>=0&&i<Wd&&j<Wd&&domain[std::size_t(i)*Wd+j];};
    const double sectionMs=core::ms(phaseStart);
    v.remainingPixels=remaining.size();v.woundPixels=wound.size();
    double x0=0,y0=0;if(!wound.empty()){for(const auto&p:wound){x0+=p.x;y0+=p.y;}x0/=double(wound.size());y0/=double(wound.size());}
    // Finite slabs already define the damage. Only certify a cut that is
    // actually open at that exact plane; no station kerf, random tilt or
    // square retry may remove extra tissue beyond it.
    if(exactPlane&&!(force&&force->clean)&&(!remaining.empty()||wound.empty())){ // melee: a clean blade cut needs no open kerf
        v.refusal="blade_plane_not_open";v.ms=core::ms(t0);GFY_DAMAGE_TEAR_REPORT(v);return v;}
    // Thin rips: the damage is the separation; what still held the limb tears along narrow channels (the user's
    // requirement: no added cut). Radius per build knob GFY_TEAR_RADIUS_MM (tests), default 5 mm.
    static const double tearRadius=[]{const char*e=std::getenv("GFY_TEAR_RADIUS_MM");const double r=e?std::atof(e)*.001:.005;return std::clamp(r,.003,.015);}();
    const double radius=guide&&guide->tear_radius>0?std::clamp(guide->tear_radius,.003,.015):tearRadius;v.tearRadius=radius;
    // Mask-only wounds, slice 3 (painter_voxel_mask_only 3): the decide-once tear at this station and plane (TearDecideTear.h);
    // false = not eligible or not separated by its sheet: today's tear below.
    // slice 4: every sever (exact blade sheets and head fractures too: keepOut limits a fracture to its own part of its plane)
    if(tear_decide::decideOnce(g,state,closure,C,n,std::min(2.2*seg->radius+.008,std::max(sectionRadius,seg->radius)+.03),
        guide?.10:.07,seed^0x7EA2C0DEull,*seg,*st,impact,force,radius,!remaining.empty(),segments,v,t0,keepOut,bool(force&&(force->keep||force->keepRadius>0))))return v;
    if(domainLate)buildDomain(); // speed step
    // ---- the station cylinder about the tear plane (|s| <= band, radius <= Rb), for the separation proof
    int lo[3],hi[3];
    // The box holds the cross-section (tissue and wounds joined to the limb) widened by the 6 mm domain in the plane and
    // the slab either side of it: every cell the flood proof can visit, and no more.
    double bmin[3]={1e30,1e30,1e30},bmax[3]={-1e30,-1e30,-1e30};bool section=false;
    for(int i=0;i<Wd;++i)for(int j=0;j<Wd;++j)if(plane[std::size_t(i)*Wd+j]&4){section=true;const Vec p=C+u*((i-Ms)*pitch)+w2*((j-Ms)*pitch);
        for(int k=0;k<3;++k){bmin[k]=std::min(bmin[k],p[k]);bmax[k]=std::max(bmax[k],p[k]);}}
    for(int k=0;k<3;++k){double a,b;
        if(section){const double pad=std::abs(n[k])*band+std::sqrt(std::max(0.,1-n[k]*n[k]))*.006+2*pitch;a=bmin[k]-pad;b=bmax[k]+pad;}
        else{const double extent=std::abs(n[k])*band+std::sqrt(std::max(0.,1-n[k]*n[k]))*Rs;a=C[k]-extent;b=C[k]+extent;}
        lo[k]=std::max(0,int(std::floor((a-g.origin[k])/pitch)));hi[k]=std::min((&g.size.x)[k]-1,int(std::floor((b-g.origin[k])/pitch)));
        if(lo[k]>hi[k]){v.refusal="station_outside_grid";v.ms=core::ms(t0);GFY_DAMAGE_TEAR_REPORT(v);return v;}}
    const int nx=hi[0]-lo[0]+1,ny=hi[1]-lo[1]+1,nz=hi[2]-lo[2]+1;const std::size_t N=std::size_t(nx)*ny*nz;
    // A cross-section with no standing tissue already separates the faces (every path crosses the plane): the wounds
    // did the whole cut, so neither the station cylinder nor the flood proof is built (blast severs: 33-84 ms each).
    const bool standingInPlane=!remaining.empty();
    // Row 7: the box sizes the flood proof's side array (1 B per cell) and visit bits (1 bit per cell), both freed when the
    // tear returns. 48 M cells: a blast's tilted chest slab boxes 55 M with the arms, and at 128 M an intact chest tore as a
    // fan of 96 radial channels without torn ends, so such stations stay refused. An empty standing section needs neither.
    static const std::size_t boxBudget=[]{const char*e=std::getenv("GFY_TEAR_BOX_CELLS_M");return std::size_t(e?std::max(1,std::atoi(e)):48)<<20;}();
    const bool fullRip=force&&standingInPlane&&N>boxBudget;
    if(standingInPlane&&N>boxBudget&&!fullRip){v.refusal="station_box_budget";v.ms=core::ms(t0);GFY_DAMAGE_TEAR_REPORT(v);return v;}
    const bool proof=standingInPlane&&!fullRip;
    std::vector<signed char> side(proof?N:0,-2); // -2 outside the cylinder or never tissue; else -1/0/+1 (stump face, interior, limb face)
    std::vector<std::vector<std::size_t>> stumpFace(core::parallelWidth()),limbFace(core::parallelWidth());
    if(proof)core::parallelChunks(std::size_t(nx),std::max<std::size_t>(4,std::size_t(nx)/8),[&](std::size_t fx,std::size_t lx,std::size_t chunk){
        TissueLookup tissue{g,state.table};auto&minus=stumpFace[chunk];auto&plus=limbFace[chunk];
        for(int x=lo[0]+int(fx);x<lo[0]+int(lx);++x)for(int y=lo[1];y<=hi[1];++y){int z0=lo[2],z1=hi[2];
            {const double s0=dot(g.center(Cell{x,y,lo[2]})-C,n),ds=n.z*pitch;
             if(std::abs(ds)>1e-12){double a=(-band-s0)/ds,b=(band-s0)/ds;if(a>b)std::swap(a,b);
              z0=std::max(lo[2],lo[2]+int(std::floor(a))-1);z1=std::min(hi[2],lo[2]+int(std::ceil(b))+1);}
             else if(std::abs(s0)>band+pitch)continue;}
            for(int z=z0;z<=z1;++z){
            const Cell c{x,y,z};const Vec d=g.center(c)-C;const double s=dot(d,n);if(std::abs(s)>band)continue;
            if(!inDomain(d)||!tissue.solid(c))continue;
            const std::size_t j=(std::size_t(x-lo[0])*ny+std::size_t(y-lo[1]))*nz+std::size_t(z-lo[2]);const signed char sd=s>band-1.5*pitch?1:(s<-band+1.5*pitch?-1:0);
            side[j]=sd;if(sd>0)plus.push_back(j);else if(sd<0)minus.push_back(j);}}});
    std::vector<std::size_t> faceCells[2];
    for(const auto&p:stumpFace)faceCells[0].insert(faceCells[0].end(),p.begin(),p.end());
    for(const auto&p:limbFace)faceCells[1].insert(faceCells[1].end(),p.begin(),p.end());
    v.sideMs=core::ms(phaseStart);
    if(std::getenv("GFY_TEAR_SELECT_LOG"))std::cerr<<"tear_side section_ms="<<sectionMs<<" slab_ms="<<v.sideMs-sectionMs<<" box="<<nx<<"x"<<ny<<"x"<<nz<<" plane="<<Wd<<std::endl;
    // Row 7: visit marks are one bit per box cell, cleared through the cells the previous flood marked (the retained per-thread
    // uint32 stamps held 4 B per cell for the thread's life: 220 MB after a chest station). Same flood order, same tests.
    std::vector<std::size_t> stack,marked;std::vector<std::uint64_t> seenBits(proof?(N+63)/64:0,0);
    auto seen=[&](std::size_t j){return ((seenBits[j>>6]>>(j&63))&1u)!=0;};
    auto mark=[&](std::size_t j){seenBits[j>>6]|=std::uint64_t(1)<<(j&63);marked.push_back(j);};
    auto xyz=[&](std::size_t j,int&x,int&y,int&z){z=int(j%nz);y=int((j/nz)%ny);x=int(j/(std::size_t(nz)*ny));};
    // Flood the standing tissue from one face; returns whether it reached the other face (removal read through a tile
    // cache: no grid mutation happens during a flood).
    std::vector<std::size_t> visited; // the cells the last flood reached
    auto flood=[&](signed char from)->bool{const auto f0=core::Clock::now();struct Add{double&t;decltype(f0) s;~Add(){t+=core::ms(s);}}add{v.floodMs,f0};for(const std::size_t j:marked)seenBits[j>>6]&=~(std::uint64_t(1)<<(j&63));marked.clear();stack.clear();visited.clear();bool reached=false;
        core::TileCache removedCache(g);
        auto standing=[&](std::size_t j){if(side[j]==-2)return false;int x,y,z;xyz(j,x,y,z);return !removedCache.removed(Cell{lo[0]+x,lo[1]+y,lo[2]+z});};
        for(const std::size_t j:faceCells[from>0?1:0])if(standing(j)){mark(j);stack.push_back(j);}
        while(!stack.empty()){const std::size_t j=stack.back();stack.pop_back();visited.push_back(j);if(side[j]==-from)reached=true;
            int x,y,z;xyz(j,x,y,z);const int q[6][3]={{x-1,y,z},{x+1,y,z},{x,y-1,z},{x,y+1,z},{x,y,z-1},{x,y,z+1}};
            for(const auto&p:q){if(p[0]<0||p[1]<0||p[2]<0||p[0]>=nx||p[1]>=ny||p[2]>=nz)continue;const std::size_t k=(std::size_t(p[0])*ny+p[1])*nz+p[2];
                if(seen(k)||!standing(k))continue;mark(k);stack.push_back(k);}}
        return reached;};
    // R68: whether flood(from) would reach the other face, from damage_tear::standingReach (side by side, bounded); outside its
    // envelope or when its scratch cannot be allocated, the unchanged flood answers. The callers below that need the visit list
    // still run flood.
    auto reaches=[&](signed char from)->bool{
        struct Removed{core::TileCache cache;const int*at;bool operator()(int x,int y,int z){return cache.removed(Cell{at[0]+x,at[1]+y,at[2]+z});}};
        return damage_tear::reachOrFlood([&]{const auto f0=core::Clock::now();struct Add{double&t;decltype(f0) s;~Add(){t+=core::ms(s);}}add{v.floodMs,f0};
            return damage_tear::standingReach(nx,ny,nz,side,from,faceCells[from>0?1:0],faceCells[from>0?0:1],[&]{return Removed{core::TileCache(g),lo};},damage_tear::reachEnvelope());},
            [&]{return flood(from);});};
    // ---- greedy cover of the standing tissue on the plane by tear channels through the wound centre
    std::vector<Channel> chosen;constexpr unsigned candidates=192;constexpr std::size_t maxChannels=96;
    std::vector<Cell> boundary;std::set<Cell> touched;std::size_t solid=0;
    const auto carveStart=core::Clock::now();double coverMs=0;
    auto cover=[&](const std::vector<Pixel>&target,int round,double margin){const auto c0=core::Clock::now();
        std::vector<Channel> pool;pool.reserve(candidates);
        auto candidate=[&](unsigned k){const double theta=3.141592653589793*(k+hash01(seed,round,k))/candidates;
            const double r=radius*(.9+.2*hash01(seed,round,1000+k));
            return makeChannel(C,n,u,w2,x0,y0,theta,r,Rs+2*pitch,core::quantMix(seed^(std::uint64_t(round)<<32)^k)|1);};
        const bool fast=true; // same candidates, coverage lists and picks, built in parallel / counted incrementally
        if(fast){pool.resize(candidates);core::parallelChunks(candidates,8,[&](std::size_t first,std::size_t last,std::size_t){for(std::size_t k=first;k<last;++k)pool[k]=candidate(unsigned(k));});}
        else for(unsigned k=0;k<candidates;++k)pool.push_back(candidate(k));
        // A picked channel spans only the pixels it newly covers (plus its radius): it tears what held the limb and
        // does not run on through other tissue in the plane.
        auto span=[&](const Channel&ch,double a,double b){Channel out=ch;const Vec d=ch.cut.direction;
            out.cut=Tunnel{C+u*x0+w2*y0+d*a,d,ch.cut.radius,b-a,ch.cut.radius};out.cut.exit_distance=b-a;out.cut.exit_mult=1;out.cut.setEdgeProfile(ch.seed);
            return out;};
        std::vector<std::vector<std::uint32_t>> covers(candidates);
        if(fast){ // candidate-major: each list still ascends in p; the test is the same expression
            core::parallelChunks(candidates,8,[&](std::size_t first,std::size_t last,std::size_t){for(std::size_t k=first;k<last;++k){const double s=pool[k].s,c=pool[k].c,pl=pool[k].plus-margin,mi=pool[k].minus-margin;auto&list=covers[k];
                for(std::uint32_t p=0;p<target.size();++p){const double rx=target[p].x-x0,ry=target[p].y-y0;const double o=-rx*s+ry*c;if(o>=0?o<=pl:-o<=mi)list.push_back(p);}}});}
        else
        for(std::uint32_t p=0;p<target.size();++p){const double rx=target[p].x-x0,ry=target[p].y-y0;
            for(unsigned k=0;k<candidates;++k){const double o=-rx*pool[k].s+ry*pool[k].c;
                if(o>=0?o<=pool[k].plus-margin:-o<=pool[k].minus-margin)covers[k].push_back(p);}}
        std::vector<char> done(target.size(),0);std::size_t left=target.size();std::vector<Channel> picked;
        // Fast path: gain[k] = covers[k] pixels not done, kept exact by decrementing through the pixel->candidates lists.
        std::vector<std::size_t> gainOf;std::vector<std::uint32_t> invStart,inv;
        if(fast){gainOf.resize(candidates);invStart.assign(target.size()+1,0);
            for(unsigned k=0;k<candidates;++k){gainOf[k]=covers[k].size();for(auto p:covers[k])++invStart[p+1];}
            for(std::size_t p=0;p<target.size();++p)invStart[p+1]+=invStart[p];
            inv.resize(invStart.back());std::vector<std::uint32_t> fill(invStart.begin(),invStart.end()-1);
            for(unsigned k=0;k<candidates;++k)for(auto p:covers[k])inv[fill[p]++]=k;}
        while(left&&chosen.size()+picked.size()<maxChannels){unsigned bestK=0;std::size_t gain=0;
            if(fast){for(unsigned k=0;k<candidates;++k)if(gainOf[k]>gain){gain=gainOf[k];bestK=k;}}
            else for(unsigned k=0;k<candidates;++k){std::size_t g1=0;for(auto p:covers[k])g1+=!done[p];if(g1>gain){gain=g1;bestK=k;}}
            if(!gain)break;const auto&ch=pool[bestK];double tlo=1e30,thi=-1e30;
            for(auto p:covers[bestK])if(!done[p]){done[p]=1;--left;const double t=(target[p].x-x0)*ch.c+(target[p].y-y0)*ch.s;tlo=std::min(tlo,t);thi=std::max(thi,t);
                if(fast)for(std::uint32_t q=invStart[p];q<invStart[p+1];++q)--gainOf[inv[q]];}
            const double m=ch.cut.radius+2*pitch;picked.push_back(span(ch,tlo-m,thi+m));covers[bestK].clear();}
        coverMs+=core::ms(c0);return picked;};
    // Rip channels are planned side by side against the grid as it stands and merged into the mask as one union (a
    // planner refusal carves that channel alone afterwards); cells two channels share count once per channel.
    auto carveAll=[&](const std::vector<Channel>&list){
        struct Job {const Channel*ch;Cell l,h;painter_detail::TileTunnelPlan<Grid> plan;};std::vector<Job> jobs;jobs.reserve(list.size());
        for(const auto&ch:list){const Tunnel&t=ch.cut;const Vec e0=t.entry,e1=t.entry+t.direction*t.penetration;const double rr=t.radius+2*pitch;
            Cell l,h;for(int k=0;k<3;++k){(&l.x)[k]=std::max(0,int(std::floor((std::min(e0[k],e1[k])-rr-g.origin[k])/pitch)));(&h.x)[k]=std::min((&g.size.x)[k]-1,int(std::floor((std::max(e0[k],e1[k])+rr-g.origin[k])/pitch)));}
            if(l.x>h.x||l.y>h.y||l.z>h.z)continue;jobs.push_back({&ch,l,h,{}});}
        for(const auto&j:jobs)checkedCutterCells(j.l,j.h);
        core::parallelChunks(jobs.size(),1,[&](std::size_t first,std::size_t last,std::size_t){
            for(std::size_t i=first;i<last;++i)jobs[i].plan=painter_detail::planTileTunnel(g,jobs[i].ch->cut,jobs[i].l,jobs[i].h,countField,
                [](std::size_t n,std::size_t,auto&&body){body(std::size_t(0),n,std::size_t(0));});});
        for(auto&j:jobs){auto&plan=j.plan;
            if(!plan.supported){carve(g,j.ch->cut,j.l,j.h,countField,boundary,touched,v.mask,solid,v.bySlot);chosen.push_back(*j.ch);continue;}
            if(plan.freshMask){v.mask+=plan.freshMask;solid+=plan.freshSolid;for(unsigned s=0;s<64;++s)v.bySlot[s]+=plan.freshBySlot[s];
                if(true){ // per touched key (same journaling, creation and insertion order) instead of a dense window over the box
                    for(const auto&key:plan.touched){auto&t=core::journaledTile(g,key);const auto&fresh=plan.stages[plan.index(key)].fresh;for(unsigned w=0;w<8;++w)t.removed[w]|=fresh.removed[w];}}
                else{core::TileWindow window(g,j.l,j.h);
                for(const auto&key:plan.touched){auto&t=window.tile(key);const auto&fresh=plan.stages[plan.index(key)].fresh;for(unsigned w=0;w<8;++w)t.removed[w]|=fresh.removed[w];}}
                touched.insert(plan.touched.begin(),plan.touched.end());boundary.insert(boundary.end(),plan.boundary.begin(),plan.boundary.end());}
            chosen.push_back(*j.ch);}};
    bool linked=true;
    if(fullRip){ // every standing section pixel's column through the plane, half either side in half-pitch steps, erased exactly
        const double half=radius+pitch;
        for(const auto&px:remaining){const Vec base=C+u*double(px.x)+w2*double(px.y)+n*midAt(px.x,px.y);
            for(double s=-half;s<=half+1e-9;s+=.5*pitch){const Cell c=cellOf(base+n*s);if(!g.valid(c)||!g.erase(c))continue;
                boundary.push_back(c);touched.insert(Grid::tileKey(c));++v.mask;
                if(const auto label=g.materialBase(c)){++solid;const auto slot=g.material_slots&&label<g.material_slots->size()?unsigned((*g.material_slots)[label]&63u):1u;++v.bySlot[slot];}}}
        Channel rip;rip.seed=core::quantMix(seed^0xF011A1Bull)|1;rip.cut=Tunnel{C-n*half,n,Rs,2*half,2*half};rip.cut.exit_distance=2*half;rip.cut.exit_mult=1;
        rip.cut.setEdgeProfile(rip.seed);chosen.push_back(rip);++v.rounds;linked=false;}
    else if(!remaining.empty()){
        carveAll(cover(remaining,0,.5*pitch));++v.rounds;
        // What still links the faces (standing cells reached from both), projected onto the plane: cover it again.
        for(int round=1;round<3;++round){
            if(!reaches(1)){linked=false;break;}
            flood(1); // still linked: the serial flood's visit list feeds the bridge
            std::vector<std::size_t> fromLimb;fromLimb.swap(visited);flood(-1);
            std::vector<Pixel> bridge;std::set<std::pair<int,int>> unique;
            for(const std::size_t j:fromLimb){if(!seen(j))continue;int x,y,z;xyz(j,x,y,z);
                const Vec d=g.center(Cell{lo[0]+x,lo[1]+y,lo[2]+z})-C;if(std::abs(dot(d,n))>radius)continue;
                const int px=int(core::roundHalfAway(dot(d,u)/pitch)),py=int(core::roundHalfAway(dot(d,w2)/pitch));
                if(unique.insert({px,py}).second)bridge.push_back({float(px*pitch),float(py*pitch)});}
            if(bridge.empty())break;
            auto more=cover(bridge,round,1.5*pitch);if(more.empty())break;carveAll(more);++v.rounds;}
        if(linked)linked=reaches(1);}
    else linked=false; // nothing stands in the cross-section
    v.estimateMs=coverMs;v.crossing=chosen.size();const double ripsMs=core::ms(carveStart); // speed: log-only phase marks
    if(linked)v.refusal=chosen.empty()?"no_tear_channel":"tear_did_not_separate"; // carved channels remain real, recorded damage
    // Checkerboard repair around everything the tear removed (the same repair every cut gets).
    std::size_t extraSolid=0;
    if(!boundary.empty()){
        try{v.edgeCells=resolveEdgeContacts(g,boundary,C,[&](Cell c){for(const auto&ch:chosen)if(ch.cut.intersectsCell(g,c))return true;return false;},touched,&extraSolid);}
        catch(const RecoverableImpactError&){if(!v.refusal)v.refusal="tear_repair_budget";}}
    v.cells=solid+extraSolid;v.mask+=v.edgeCells;const double edgeMs=core::ms(carveStart)-ripsMs;
    // R62: the separated ends tear back by tissue type (TissueTear.h): bone stands, muscle tears in bundles, the skin shell pulls
    // back; the torn cells join this tear's damage and its torn surface is one more record (mode 4 walls).
    ResolvedTunnel tissueRecord;bool tissueSurface=false;
    static const bool tissueTearOff=std::getenv("GFY_NO_TISSUE_TEAR")!=nullptr;
    if(!v.refusal&&closure&&closure->available&&!tissueTearOff&&!(force&&force->noTissueTear)){
        const double half=guide?.10:.07;
        // the limb's section at the plane plus 3 cm (it widens along the axis), never past the section search radius
        const double R=std::min(2.2*seg->radius+.008,std::max(sectionRadius,seg->radius)+.03);
        auto tt=tissue_tear::tear(g,*closure,countField,C,n,R,half,seed^0x7EA2C0DEull,st->center,seg->axis,impact?*impact:Vec{0,0,0},force?force->tissueDepthScale:1.,force&&force->keepIslands,force?force->headTear:0,radius); // T1: + the rip channels' radius
        v.tissueRan=true;v.tissueMs=tt.ms;v.tissueRefusal=tt.refusal;v.tissueTorn=tt.torn+tt.thin;v.tissueLoose=tt.loose;v.tissueFresh=tt.fresh;v.tissueTriangles=tt.triangles;
        if(tt.done){v.cells+=tt.solid;v.mask+=tt.fresh;for(unsigned s=0;s<64;++s)v.bySlot[s]+=tt.bySlot[s];touched.insert(tt.tiles.begin(),tt.tiles.end());
            if(tt.surface){const Vec a0=unit(cross(n,std::abs(n.x)<.8?Vec{1,0,0}:Vec{0,1,0})),b0=cross(n,a0);const double reach=half+.025;const Vec e=C-n*reach; // the columns plus the torn sheet's relief
                tissueRecord.entry={e.x,e.y,e.z};tissueRecord.direction={n.x,n.y,n.z};tissueRecord.edge_u={a0.x,a0.y,a0.z};tissueRecord.edge_v={b0.x,b0.y,b0.z};
                tissueRecord.radius=R;tissueRecord.penetration=2*reach;tissueRecord.cap=2*reach;tissueRecord.inlet=0;tissueRecord.exit_distance=2*reach;tissueRecord.exit_mult=1;
                tissueRecord.edge_seed=seed;tissueRecord.surface=std::move(tt.surface);tissueSurface=true;}}}
    const double afterTissueMs=core::ms(carveStart);
    v.dirty=core::expandDirtyTiles(touched);
    // Stations the tear carved into are measured afresh next time (their cached tile shares predate the rips).
    if(!v.dirty.empty()&&state.measured.size()==total){std::size_t k2=0;
        for(const auto&s2:segments)for(const auto&station:s2.stations){const std::size_t kk=k2++;if(!state.measured[kk])continue;
            for(const Cell&t:v.dirty)if(t.x>=station.tileLo.x&&t.x<=station.tileHi.x&&t.y>=station.tileLo.y&&t.y<=station.tileHi.y&&t.z>=station.tileLo.z&&t.z<=station.tileHi.z){state.measured[kk]=0;break;}}}
    for(std::size_t i=0;i<chosen.size();++i)v.records.push_back(resolvedTunnel(chosen[i].cut,chosen[i].seed,i==0?v.edgeCells:0));
    if(tissueSurface)v.records.push_back(std::move(tissueRecord));
    v.carveMs=core::ms(carveStart);v.tear=!v.refusal;
    if(std::getenv("GFY_TEAR_SELECT_LOG"))std::cerr<<"tear_carve_phases rips_ms="<<ripsMs<<" cover_ms="<<coverMs<<" flood_ms="<<v.floodMs<<" edge_contacts_ms="<<edgeMs
        <<" tissue_ms="<<v.tissueMs<<" dirty_and_records_ms="<<v.carveMs-afterTissueMs<<" channels="<<chosen.size()<<" boundary_cells="<<boundary.size()<<" touched_tiles="<<touched.size()<<" dirty_tiles="<<v.dirty.size()<<std::endl;
    // The failed station and its neighbours within one limb radius are done (as severCheck).
    if(v.tear){std::size_t k=0;for(const auto&s:segments)for(const auto&station:s.stations){if(&s==seg&&std::abs(station.s-st->s)<=seg->radius)state.failed[k]=1;++k;}}
    v.ms=core::ms(t0);GFY_DAMAGE_TEAR_REPORT(v);return v;
}
namespace damage_tear {
struct KillPlane {bool found=false;Vec point{},normal{};const SeverStation*station=nullptr;std::size_t cells=0;double onPlane=0,tilt=0,ms=0;};
inline KillPlane killPlane(const Grid&g,const SeverState&state,const SeverSegment&seg,std::size_t segBase,const SeverStation&st,const Vec&shotDir){
    KillPlane kp;const auto t0=core::Clock::now();const double pitch=g.pitch,pi=3.141592653589793;
    const Vec A=unit(seg.axis);const double R=seg.radius+.01,W=.045,reach=W+1.7320508075688772*R; // a 60-degree plane reaches tan(60) r along the limb
    Vec u0=shotDir-A*dot(shotDir,A);u0=length(u0)>1e-6?unit(u0):unit(seg.u);const Vec v0=cross(A,u0);
    auto log=[&](const char*why){if(tear_decide::logOn())std::cerr<<"kill_plane bone="<<seg.bone<<" cells="<<kp.cells<<" found="<<kp.found<<" on_plane="<<kp.onPlane
        <<" tilt_deg="<<kp.tilt<<" why="<<why<<" ms="<<core::ms(t0)<<std::endl;};
    int lo[3],hi[3];
    for(int k=0;k<3;++k){const double ext=std::abs(A[k])*reach+std::sqrt(std::max(0.,1-A[k]*A[k]))*R;
        lo[k]=std::max(0,int(std::floor((st.center[k]-ext-g.origin[k])/pitch)));hi[k]=std::min((&g.size.x)[k]-1,int(std::floor((st.center[k]+ext-g.origin[k])/pitch)));
        if(lo[k]>hi[k]){log("outside_grid");return kp;}}
    struct Pt{float t,a,b;};std::vector<Pt> pts;TissueLookup tissue{g,state.table};
    for(int tx=lo[0]>>3;tx<=hi[0]>>3;++tx)for(int ty=lo[1]>>3;ty<=hi[1]>>3;++ty)for(int tz=lo[2]>>3;tz<=hi[2]>>3;++tz){
        const core::Tile*tile=g.findTile(Cell{tx,ty,tz});if(!tile)continue;
        for(unsigned wd=0;wd<8;++wd){const std::uint64_t bits=tile->removed[wd];if(!bits)continue;
            for(unsigned b=0;b<64;++b){if(!((bits>>b)&1u))continue;const unsigned l=wd*64+b;const Cell c{tx*8+int(l>>6),ty*8+int((l>>3)&7u),tz*8+int(l&7u)};
                if(c.x<lo[0]||c.y<lo[1]||c.z<lo[2]||c.x>hi[0]||c.y>hi[1]||c.z>hi[2]||!tissue.solid(c))continue;
                const Vec r=g.center(c)-st.center;const double t=dot(r,A);if(std::abs(t)>reach)continue;const double a=dot(r,u0),bb=dot(r,v0);if(a*a+bb*bb>R*R)continue;
                pts.push_back({float(t),float(a),float(bb)});}}}
    kp.cells=pts.size();const std::size_t minCells=24;if(pts.size()<minCells){log("no_damage");return kp;}
    struct Dir{double th,ph;};std::vector<Dir> dirs{{0.,0.}};
    for(int ti=1;ti<=4;++ti)for(int k=0;k<8;++k)dirs.push_back({ti*15.*pi/180,k*pi/4});
    {const double ca=std::clamp(dot(unit(shotDir),A),-1.,1.),tilt=std::abs(std::asin(ca)); // the plane holding the shot's path
     if(tilt>1e-3&&tilt<=pi/3+1e-9)dirs.push_back({tilt,ca>0?pi:0.});}
    const double A0=pi*seg.radius*seg.radius/(pitch*pitch),s0=-(reach+R)-pitch;const int nb=int(std::ceil(2*(reach+R)/pitch))+3;
    struct Best{double score=0,count=0,s=0;bool ok=false;};std::vector<Best> bests(dirs.size());
    core::parallelChunks(dirs.size(),1,[&](std::size_t f,std::size_t l,std::size_t){std::vector<std::uint32_t> h(std::size_t(nb),0u);
        for(std::size_t k=f;k<l;++k){const double ct=std::cos(dirs[k].th),stt=std::sin(dirs[k].th),cp=std::cos(dirs[k].ph),sp=std::sin(dirs[k].ph);
            std::fill(h.begin(),h.end(),0u);
            for(const auto&p:pts){const double s=p.t*ct+(p.a*cp+p.b*sp)*stt;const int bi=int(std::floor((s-s0)/pitch));if(bi>=0&&bi<nb)++h[std::size_t(bi)];}
            auto&B=bests[k];
            for(int bi=1;bi+1<nb;++bi){const double s=s0+(bi+.5)*pitch;const double ta=s/ct;if(ta<-.002||ta>W)continue; // the plane crosses the limb's axis from the shot's station to 4.5 cm below it
                const double c3=double(h[std::size_t(bi-1)]+h[std::size_t(bi)]+h[std::size_t(bi+1)])/3.,score=c3-A0/ct;
                if(!B.ok||score>B.score){B.ok=true;B.score=score;B.count=c3;B.s=s;}}}});
    std::size_t bk=dirs.size();for(std::size_t k=0;k<dirs.size();++k)if(bests[k].ok&&(bk==dirs.size()||bests[k].score>bests[bk].score))bk=k;
    if(bk==dirs.size()||bests[bk].count<double(minCells)){log("too_little_on_plane");return kp;}
    const Vec n=unit(A*std::cos(dirs[bk].th)+(u0*std::cos(dirs[bk].ph)+v0*std::sin(dirs[bk].ph))*std::sin(dirs[bk].th)),P=st.center+n*bests[bk].s;
    kp.onPlane=bests[bk].count;kp.tilt=dirs[bk].th*180/pi;
    double gap=1e30;for(std::size_t q=0;q<seg.stations.size();++q){if(segBase+q<state.failed.size()&&state.failed[segBase+q])continue;
        const double dq=std::abs(dot(seg.stations[q].center-P,n));if(dq<gap){gap=dq;kp.station=&seg.stations[q];}}
    if(!kp.station||gap>seg.halfWidth+pitch){kp.station=nullptr;log("no_free_station");return kp;}
    kp.found=true;kp.point=P;kp.normal=n;kp.ms=core::ms(t0);log("through_damage");return kp;}
}
