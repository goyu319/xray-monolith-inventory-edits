#pragma once
#include <cmath>
#include <utility>
#include <vector>

#include "HeadBurst.h" // mask-only wounds, slice 4: the random break-up (value 3)
namespace head_fracture {
enum Event:unsigned {none=0,cap=1,pieces=2};
// One draw per shot: slugs explode the head 35 % / blow the cap off 25 %, rifle-class bursts 8 % / 12 %, other bursts cap 5 %;
// the menu's head blowout frequency scales both (0 = never). The rest keep the blowout the shot's forensic shape chose.
// klass (2026-11-11): round_class is Lua's class of the round (Impact::round_class: 0 small arms, 1 rifle, 2 heavy round or slug)
// and replaces the hole-size tests when given (>= 0); -1 keeps them. The rifle class keeps its reach test: this hit's channel
// must be at least 0.2 m long (it can cross the head) - that is the shot's own reach, not its class.
inline Event event(double radius,double penetration,double burst,double frequency,std::uint64_t seed,int round_class=-1){
    if(burst<.5||!(frequency>0))return none;
    const bool classed=round_class>=0;
    const bool slug=classed?round_class>=2:radius>=.011,rifle=penetration>=.2&&(classed?round_class==1:radius>=.0075);
    const double f=std::min(2.,frequency),pPieces=(slug?.35:(rifle?.08:0.))*f,pCap=(slug?.25:(rifle?.12:.05))*f;
    const double u=damage_tear::hash01(seed,0x4EADull,99);
    return u<pPieces?pieces:(u<pPieces+pCap?cap:none);}
struct Chance3 {double pieces,cap;};
constexpr Chance3 chance3[3]={{0.,.05},  // 0 small arms: pistol, buckshot pellet (5 %)
                              {.08,.12}, // 1 rifle (20 %)
                              {.35,.25}};// 2 slug / heavy round (60 %)
inline Event event3(int round_class,double radius,double frequency,std::uint64_t seed){
    if(!(frequency>0))return none;
    const auto&c=chance3[head_burst::roundClass(head_burst::Round{round_class,radius})];
    const double f=std::min(2.,frequency),pPieces=c.pieces*f,pCap=c.cap*f;
    const double u=damage_tear::hash01(seed,0x4EADull,99);
    return u<pPieces?pieces:(u<pPieces+pCap?cap:none);}
struct Result {bool done{};unsigned planes{},torn{};Vec centre{},up{};std::vector<ResolvedTunnel> records;std::set<Cell> dirty;
    std::size_t cells{},mask{},edgeCells{};std::array<std::size_t,64> bySlot{};double ms{};const char*refusal{};
    unsigned tissueRan{},tissueBuilt{};std::size_t tissueTriangles{};double tissueMs{};const char*tissueRefusal{};}; // T1: the fractures' torn ends (release log)
// The brain's centre and its extent along `up` (rest space) from the cranial measure's brain spans.
inline bool brain(const CranialMeasure&m,Vec up,Vec&centre,double&low,double&high){
    if(!m.valid||m.brain.blocks.empty())return false;double n=0;Vec sum{0,0,0};low=1e30;high=-1e30;
    for(const auto&b:m.brain.blocks)for(const auto&s:b.spans){const double len=double(s.hi-s.lo);if(len<=0)continue;
        const Vec a=m.brain.origin+Vec{s.x+.5,s.y+.5,s.lo+.5}*m.brain.pitch,c=m.brain.origin+Vec{s.x+.5,s.y+.5,s.hi-.5}*m.brain.pitch;
        sum=sum+(a+c)*(.5*len);n+=len;low=std::min(low,std::min(dot(a,up),dot(c,up)));high=std::max(high,std::max(dot(a,up),dot(c,up)));}
    if(n<1000)return false;centre=sum*(1./n);return true;}
// headsize: the head's floor (fracture() below): the neck row's top station, or 10 cm under the brain's centre without a neck row.
inline Vec headFloor(const SeverSegment*neck,const Vec&C){return neck?(neck->stations.empty()?neck->pivot+neck->axis*neck->length:neck->stations.back().center):C-Vec{0,.10,0};}
// headsize: has this round an opening on the head - above the head's floor (head_burst::opening, the test of the head's plan)?
// false: a NECK HIT, a hit through the neck that the game booked on the head bone. It draws no head event; LiveDamageWorker.cpp
// gives it the body's event, kept above neckFloor: the neck's base joint (the neck row's pivot; no neck row: 19 cm under the
// brain's centre) - never the shoulders. true when the model cannot tell (no closure field, no brain): the head path as before.
inline bool onHead(const std::vector<SeverSegment>&segments,const CranialMeasure&cranial,const painter_detail::TissueField*closure,Vec entry,Vec direction,double penetration,Vec&neckFloor){
    if(!closure||!closure->available||!(length(direction)>1e-9))return true;
    Vec C;double low,high;if(!brain(cranial,Vec{0,1,0},C,low,high))return true;
    const SeverSegment*row=nullptr;for(const auto&s:segments)if(std::abs(s.radius-.075)<1e-6&&s.axis.y>.5){row=&s;break;} // the neck row
    const Vec headAt=headFloor(row,C);neckFloor=row?row->pivot:C-Vec{0,.19,0};
    const Vec d=unit(direction),P0=entry+d*dot(C-entry,d);
    auto tissueAt=[&](const Vec&p){const unsigned l=closure->sample(p.x,p.y,p.z)&63u;return l!=0&&l!=16;};
    Vec X,mouth,o;return head_burst::opening(entry,P0,d,penetration,tissueAt,&headAt,Vec{0,1,0},false,X,mouth,o);}
// entry/direction: the bullet's path (rest space). seed: the shot's.
inline Result fracture(Grid&g,const std::vector<SeverSegment>&segments,SeverState&state,const CranialMeasure&cranial,
    const std::vector<const ResolvedTunnel*>&cuts,const painter_detail::TissueField*countField,Vec entry,Vec direction,std::uint64_t seed,std::uint16_t bone,
    Event kind=pieces,const painter_detail::TissueField*closure=nullptr,const head_burst::Round*round=nullptr, // headfix: the round sizes a value-3 burst
    bool ownScale=false,bool ballistic=false){ // headsize (value 3): the head's own scale and placement (the Extreme damage mode, a crushing weapon's blow)
    Result r;const auto t0=core::Clock::now();
    Vec up{0,1,0}; // rest pose stands upright; the neck row's axis when the profile has one
    const SeverSegment*neck=nullptr; // headsize: the neck row (its far end is the head joint)
    for(const auto&s:segments)if(std::abs(s.radius-.075)<1e-6&&s.axis.y>.5){up=unit(s.axis);neck=&s;break;}
    Vec C;double low,high;if(!brain(cranial,up,C,low,high)){r.refusal="head_fracture_no_brain";return r;}
    const Vec d=unit(direction);if(!(length(direction)>1e-9)){r.refusal="head_fracture_direction";return r;}
    // the bullet's line nearest the brain's centre; a shot that passed under the vault does not crack it
    const Vec P0=entry+d*dot(C-entry,d);const double height=high-low;
    // headfix (value 3): ONE exact partition of the head around a blast on the bullet's path, sized by the round (HeadBurst.h); any
    // path through the head (a jaw, cheek or brow shot too), not only one past the brain
    if(mask_only_mode_value()==3&&closure&&closure->available){
        if(length(P0-C)>.12){r.refusal="head_fracture_path_off_head";return r;}
        auto tissueAt=[&](const Vec&p){const unsigned l=closure->sample(p.x,p.y,p.z)&63u;return l!=0&&l!=16;};
        const head_burst::Round rd=round?*round:head_burst::Round{};
        const Vec floorAt=headFloor(neck,C);
        auto pl=head_burst::plan(C,entry,P0,d,seed,kind==cap,rd,tissueAt,ownScale?.5*(high-low)+.035:0.,&floorAt);
        pl.ballisticBone=ballistic;
        if(!pl.S){r.refusal="head_fracture_path_off_head";return r;}
        auto b=head_burst::burst(g,segments,state,pl);
        if(std::getenv("GFY_M3_LOG")||std::getenv("GFY_TEAR_SELECT_LOG")){const auto&p=b.plan;
            std::cerr<<"head_burst kind="<<unsigned(kind)<<" class="<<head_burst::roundClass(rd)<<" strength="<<p.sigma<<" rf_mm="<<p.rf*1000<<" near="<<p.nNear
             <<" groups="<<p.groups<<" sites="<<p.S<<" own="<<ownScale<<" floor_y="<<floorAt.y<<" rcut_mm="<<p.Rcut*1000<<" blast=("<<p.B.x<<","<<p.B.y<<","<<p.B.z<<") exit=("<<p.exit.x<<","<<p.exit.y<<","<<p.exit.z
             <<") tiles="<<b.tiles<<" mixed_tiles="<<b.mixedTiles<<" labelled="<<b.labelled<<" erased="<<b.erased<<" crumbs="<<b.crumbs<<" crumb_cells="<<b.crumbCells<<" crumb_ms="<<b.crumbMs<<" pre_cells="<<b.preCells<<" pre_ms="<<b.preMs<<" edge_cells="<<b.edgeCells<<" samples="<<b.samples<<" group_cells=";
            for(std::size_t k=0;k<b.groupCells.size();++k)std::cerr<<(k?",":"")<<b.groupCells[k];
            std::cerr<<" label_ms="<<b.labelMs<<" erase_ms="<<b.eraseMs<<" repair_ms="<<b.repairMs<<" ms="<<b.ms<<" refusal="<<(b.refusal?b.refusal:"")<<std::endl;}
        r.centre=C;r.up=up;r.planes=1;r.torn=b.done?1u:0u;r.cells=b.cells;r.mask=b.mask;r.edgeCells=b.edgeCells;r.bySlot=b.bySlot;
        r.dirty=std::move(b.dirty);r.records=std::move(b.records);r.tissueRan=1;r.tissueMs=b.ms;
        r.done=b.done;if(!r.done)r.refusal=b.refusal?b.refusal:"head_fracture_nothing_torn";r.ms=core::ms(t0);return r;}
    if(length(P0-C)>.6*height){r.refusal="head_fracture_path_off_brain";return r;}
    auto rnd=[&](unsigned k){return damage_tear::hash01(seed,0x4EADull,k);};
    // the base: under the vault, tilted up to 20 degrees toward a seeded side
    Vec a0=unit(cross(up,std::abs(up.x)<.8?Vec{1,0,0}:Vec{0,0,1})),b0=cross(up,a0);
    const double tilt=(20.*rnd(1))*3.141592653589793/180,turn=6.283185307179586*rnd(2);
    const Vec baseN=unit(up*std::cos(tilt)+(a0*std::cos(turn)+b0*std::sin(turn))*std::sin(tilt));
    const Vec B=C+up*((high-(kind==cap?.060:.075))-dot(C,up));
    // (a path under the cut still bursts the vault: the cap goes, the radial fractures are kept above the cut)
    r.centre=C;r.up=up;
    // radial fractures: 3 or 4 half-planes around the vault's own axis (up through the cut's centre under the brain), evenly turned
    // with a seeded jitter - the vault splits into wedge plates (skull, scalp, some brain) wherever the bullet went (the bullet's
    // path usually runs low through the head: half-planes around it missed the vault and freed one plate)
    const unsigned K=kind==pieces?3+unsigned(rnd(3)<.5):0;
    const Vec V=C-baseN*dot(C-B,baseN);
    Vec pa=unit(cross(baseN,std::abs(baseN.x)<.8?Vec{1,0,0}:Vec{0,0,1})),pb=cross(baseN,pa);
    const double phase=6.283185307179586*rnd(4);
    std::vector<std::pair<Vec,Vec>> baseKeep{{B,baseN}};
    struct Plan {Vec point,normal;std::vector<std::pair<Vec,Vec>> keep;Vec keepCenter{};double keepRadius{};int headTear=0;};std::vector<Plan> plans;
    { // the base fracture: the head only (within 16 cm of the brain's centre)
        Plan p;p.point=C-baseN*dot(C-B,baseN);p.normal=baseN;p.keepCenter=C;p.keepRadius=.16;plans.push_back(std::move(p));}
    for(unsigned k=0;k<K;++k){const double phi=phase+6.283185307179586*(k+.35*(rnd(10+k)-.5))/K;
        const Vec rk=unit(pa*std::cos(phi)+pb*std::sin(phi));Plan p;p.normal=unit(cross(baseN,rk));
        p.point=V+rk*.035+baseN*.03;
        p.keep={{V,rk},{B,baseN}};p.keepCenter=C;p.keepRadius=.16;plans.push_back(std::move(p));}
    // value 3 returned above (headfix: HeadBurst.h)
    std::set<Cell> touched;
    for(std::size_t i=0;i<plans.size();++i){const auto&pl=plans[i];
        SeverSegment seg;seg.bone=bone;seg.pivot=pl.point;seg.axis=pl.normal;seg.radius=.09;seg.halfWidth=.004;seg.length=.01;
        const Vec ref=std::abs(seg.axis.z)<.9?Vec{0,0,1}:Vec{1,0,0};seg.u=unit(cross(ref,seg.axis));seg.v=cross(seg.axis,seg.u);
        SeverStation st;st.center=pl.point;st.s=0;st.intactTissue=1;st.intactBone=1;seg.stations.push_back(st);
        TearPlane plane;plane.point=pl.point;plane.normal=pl.normal;plane.a=plane.b=pl.point;
        TearForce force{&seg,&seg.stations[0],&plane};force.keep=&pl.keep;force.keepCenter=pl.keepCenter;force.keepRadius=pl.keepRadius;force.keepIslands=true;force.headTear=pl.headTear?pl.headTear:(i==0?1:2); // the cap's fracture deep and lumpy, the radial ones light // the skull splinters, the scalp pulls back ~1 cm: pieces stay whole
        TearGuide guide;guide.tear_radius=.004;
        const std::set<Cell> where{Grid::tileKey(Cell{int(std::floor((pl.point.x-g.origin.x)/g.pitch)),int(std::floor((pl.point.y-g.origin.y)/g.pitch)),int(std::floor((pl.point.z-g.origin.z)/g.pitch))})};
        std::vector<const ResolvedTunnel*> all=cuts;for(const auto&rec:r.records)all.push_back(&rec);
        auto v=damageTear(g,segments,state,where,2.,all,countField,nullptr,nullptr,&guide,&force,closure,&d);
        ++r.planes;if(v.tear)++r.torn;
        if(v.tissueRan){++r.tissueRan;r.tissueMs+=v.tissueMs;if(v.tissueTriangles){++r.tissueBuilt;r.tissueTriangles+=v.tissueTriangles;}
         else if(!r.tissueRefusal)r.tissueRefusal=v.tissueRefusal?v.tissueRefusal:"tissue_tear_no_surface";}
        r.cells+=v.cells;r.mask+=v.mask;r.edgeCells+=v.edgeCells;for(unsigned s=0;s<64;++s)r.bySlot[s]+=v.bySlot[s];
        r.dirty.insert(v.dirty.begin(),v.dirty.end());for(auto&rec:v.records)r.records.push_back(std::move(rec));}
    // T2 (tear2): a later fracture's tear erases tissue an earlier fracture's torn surface bounded (the scalp between two
    // fractures, the crumbs its islands rule drops): those triangles were drawn with nothing behind them (floating scalp fat).
    // Every torn surface keeps only its triangles with tissue behind them in the final grid (a surface that would lose them all
    // stays as it was).
    if(closure&&closure->available)for(auto&rec:r.records)if(rec.surface){std::size_t dropped=0;auto kept=tissue_tear::keepBacked(g,*closure,*rec.surface,&dropped);
        if(dropped&&!kept->triangles.empty())rec.surface=std::move(kept);}
    r.done=r.torn>0;if(!r.done)r.refusal="head_fracture_nothing_torn";r.ms=core::ms(t0);return r;}
}
#ifdef GFY_VOXEL_WORKER_TESTS
#define GFY_HEAD_FRACTURE_REPORT(hf,kind) (std::cout<<"{\"head_fracture\":{\"event\":"<<unsigned(kind)<<",\"done\":"<<(hf.done?"true":"false")<<",\"planes\":"<<hf.planes<<",\"torn\":"<<hf.torn     <<",\"cells\":"<<hf.cells<<",\"records\":"<<hf.records.size()<<",\"ms\":"<<hf.ms<<",\"refusal\":\""<<(hf.refusal?hf.refusal:"")<<"\"}}"<<std::endl)
#else
#define GFY_HEAD_FRACTURE_REPORT(hf,kind) ((void)0)
#endif
