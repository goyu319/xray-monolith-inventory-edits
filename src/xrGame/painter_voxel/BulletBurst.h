#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <set>
#include <vector>
namespace bullet_burst {
enum Event:unsigned {none=0,blowout=1,burst=2}; // the head's cap / pieces
inline const char*name(unsigned e){return e==2?"burst":(e==1?"blowout":"none");}
// One draw per shot: the head's table (pieces -> burst, cap -> blowout) x the menu's body blowout frequency (0 = never, at most x2).
inline Event event(int round_class,double radius,double frequency,std::uint64_t seed){
 if(!(frequency>0))return none;
 const auto&c=head_fracture::chance3[head_burst::roundClass(head_burst::Round{round_class,radius})];
 const double f=std::min(2.,frequency),pBurst=c.pieces*f,pBlow=c.cap*f;
 const double u=damage_tear::hash01(seed,0xB0D1B5ull,99);
 return u<pBurst?burst:(u<pBurst+pBlow?blowout:none);}
constexpr double R0=head_burst::eventScale; // headsize: the one scale of every bullet event, head and body (HeadBurst.h: 0.05 m, as before)
// a corpse burst's fragments fly off as pieces from this blast radius up (a slug's 38 mm: 8-61 cm3 fragments, all separated); a smaller
// burst is blown out (a crater, as on the living)
constexpr double piecesMinRf=.030;
// burstshell: the switch of plan()'s shell rule below (probes: GFY_BURST_SHELL_OFF)
inline std::atomic<int>& shellOn(){static std::atomic<int> v{std::getenv("GFY_BURST_SHELL_OFF")?0:1};return v;}
// The shot's way into and out of the body along d from its first solid cell (rest space), on the closure field's tissue (clothing,
// gear and padding are not tissue): skin = the first tissue within 6 cm, out = the last tissue before 2 cm of air (as the head's
// exitPoint) or where its reach runs out; false: the round stops inside (no way out). (The tunnel's envelope exit can lie on a gear
// shell in front of the chest: measured on KVMA, a chest shot's envelope exit 13 cm in front of its back.)
template<class Tissue>
inline bool wayOut(const Vec&start,const Vec&d,double reach,const Tissue&tissueAt,Vec&skin,Vec&out){
 const double step=.001;double t0=-1;skin=start;out=start;
 for(double t=0;t<=.06&&t<=reach;t+=step)if(tissueAt(start+d*t)){t0=t;break;}
 if(t0<0)return false;skin=start+d*t0;
 double last=t0;double t=t0;
 for(;t<=reach;t+=step){if(tissueAt(start+d*t))last=t;else if(t-last>.02)break;}
 out=start+d*last;
 return t-last>.02||last<reach-2*step; // its travel went on past the last tissue it met
}
// The plan. mouth: where the shot leaves the body (its exit) or enters it, rest space; out: the outward direction there (the shot's
// travel at an exit, against it at an entry); tissueAt: the closure field's tissue test.
template<class Tissue>
inline head_burst::Plan plan(Event kind,const Vec&mouth,const Vec&out,std::uint64_t seed,const head_burst::Round&round,const Tissue&tissueAt){
 if(const char*e=std::getenv("GFY_BULLET_BURST_SEED"))seed^=core::quantMix(std::strtoull(e,nullptr,10)*0x9E3779B97F4A7C15ull+1); // test knob
 auto R=[&](unsigned k){return damage_tear::hash01(seed,0xB0D1ull,k);};
 const bool blow=kind==blowout;
 head_burst::Plan p;p.body=true;p.ballisticBone=true;p.cap=blow;p.seed=std::uint32_t(core::quantMix(seed^0xB0D1B57Bull));p.sigma=head_burst::strength(round);
 if(const char*e=std::getenv("GFY_BULLET_BURST_STRENGTH"))p.sigma=std::clamp(std::atof(e),.05,1.); // test knob
 const double sg=p.sigma;
 p.rf=R0*(.25+.90*sg)/1.3*(.85+.30*R(4))*(blow?1.15:1.)*(round.size>0?round.size:1.); // modes: Extreme doubles it
 const Vec o=length(out)>1e-9?unit(out):Vec{0,1,0};
 const Vec a0=unit(cross(o,std::abs(o.x)<.8?Vec{1,0,0}:Vec{0,1,0})),b0=cross(o,a0);
 // a burst's blast point inside the opening (its region opens through the skin, the fragments lie under it); a blowout's outside it
 p.B=mouth+o*(blow?.30*p.rf:-.45*p.rf)+a0*(.25*p.rf*(R(2)-.5))+b0*(.25*p.rf*(R(3)-.5));
 if(!blow)for(int k=1;k<=30&&!tissueAt(p.B);++k)p.B=p.B-o*.002; // the blast sits in tissue
 p.entry=mouth;p.exit=mouth;
 auto sphere=[&](unsigned k){const double z=2*R(k)-1,t=6.283185307179586*R(k+1),r=std::sqrt(std::max(0.,1-z*z));return Vec{r*std::cos(t),r*std::sin(t),z};};
 // near sites: a burst's fragments 1-6 from the round (the head's counts), leaning out of the body; a blowout 2-3 sites spread
 // along the skin, one group (its region is blown out whole)
 const unsigned n=blow?2u+unsigned(R(5)*1.999):std::min(head_burst::maxNear,1u+unsigned(sg*5.*(.8+.4*R(5))));
 p.nNear=n;
 for(unsigned k=0;k<n;++k){
  if(k==0){p.site[k]=p.B+sphere(10)*(.15*p.rf*R(12));continue;}
  Vec s=sphere(20+3*k);if(blow)s=s-o*(.6*dot(s,o));
  const Vec dir=unit(s*.6+o*(blow?-.15:.4));p.site[k]=p.B+dir*(p.rf*(.35+.5*R(22+3*k)));}
 for(unsigned k=0;k<n;++k)p.group[k]=blow?1u:k+1;p.groups=blow?2u:n+1;
 // far sites around them: the rest of the body (group 0, stays)
 {const double rot=6.283185307179586*R(60);
  for(unsigned k=0;k<head_burst::nFarSites;++k){const double z=1-2*(k+.5)/head_burst::nFarSites,t=2.399963229728653*k+rot,r=std::sqrt(std::max(0.,1-z*z));const Vec u{r*std::cos(t),r*std::sin(t),z};
   p.site[n+k]=p.B+unit(u+sphere(70+3*k)*.25)*(p.rf*(1.65+.35*R(72+3*k)));p.group[n+k]=0;}}
 if(shellOn().load(std::memory_order_relaxed)){
  constexpr int ND=42;const double keep=.010+.5*(.22+.05)*p.rf,stepS=.002,stepT=.003,reachMax=2.2*p.rf;
  Vec dirs[ND];double surf[ND];int nd=0;
  for(int k=0;k<ND;++k){const double z=1-2*(k+.5)/ND,ang=2.399963229728653*k,r=std::sqrt(std::max(0.,1-z*z));const Vec u{r*std::cos(ang),r*std::sin(ang),z};
   if(dot(u,o)>.2)continue; // the opening's side (within about 78 degrees of the way out: the mouth may be as wide as before)
   // the body's surface along u: the last tissue before 2 cm without any (none within reach: tissue carries on, no surface to keep)
   double last=-1,tt=0;bool seen=false,ended=false;
   for(;tt<=reachMax;tt+=stepS){if(tissueAt(p.B+u*tt)){last=tt;seen=true;}else if(seen&&tt-last>.02){ended=true;break;}}
   if(!seen||!ended)continue;
   dirs[nd]=u;surf[nd]=last;++nd;}
  // the region reaches into the last `keep` before the surface along a direction: a point there (the steps of a walk from the blast
  // point in stepT, those past `from` only) whose nearest site is a near one (the warp aside: `keep` holds half of it)
  auto reaches=[&](const Vec&u,double from,double upTo){for(double tt=stepT*std::floor(from/stepT+1.);tt<=upTo;tt+=stepT){const Vec q=p.B+u*tt;double best=1e30;unsigned g=0;
    for(unsigned i=0;i<n+head_burst::nFarSites;++i){const Vec dq=q-p.site[i];const double d2=dot(dq,dq);if(d2<best){best=d2;g=p.group[i];}}
    if(g!=0)return true;}return false;};
  for(int pass=0;pass<40&&nd;++pass){bool any=false;
   for(int k=0;k<nd;++k){const double allow=std::max(0.,surf[k]-keep);if(!reaches(dirs[k],allow,surf[k]+.004))continue;any=true;
    // the far site nearest that direction comes 8 % closer (not inside a quarter of the blast radius)
    unsigned bestI=n;double bestD=-2;for(unsigned i=n;i<n+head_burst::nFarSites;++i){const Vec v=p.site[i]-p.B;const double l=length(v);if(l<1e-9)continue;const double c=dot(v,dirs[k])/l;if(c>bestD){bestD=c;bestI=i;}}
    const Vec v=p.site[bestI]-p.B;const double l=length(v);if(l>.25*p.rf)p.site[bestI]=p.B+v*std::max(.92,.25*p.rf/l);}
   if(!any)break;}}
 p.S=n+head_burst::nFarSites;
 p.A1=.22*p.rf;p.L1=.95*p.rf;p.A2=.05*p.rf;p.L2=.32*p.rf;p.ripR=.003;p.dscale=.75+.25*sg;
 p.Rcut=std::min(2.6*p.rf+2*(p.A1+p.A2),.6); // provisional for a burst (its labelling sets the exact cut); 0.6 m bounds the labelled box (Extreme)
 return p;}

// ---- a crater: every tissue cell of the plan's near groups erased (a living body's burst, a blowout). Journaled through Grid::erase,
// repaired as every cut (resolveEdgeContacts), the stations under it measured again. No record: the cells are the wound.
#include "BulletTornFloor.h"
struct Crater {bool done=false;std::size_t cells=0,mask=0,edgeCells=0,erased=0,tiles=0,wholeTiles=0,labelled=0,boneKept=0;std::array<std::size_t,64> bySlot{};
 std::set<Cell> dirty;const char*refusal=nullptr;double ms=0,labelMs=0,eraseMs=0,repairMs=0;};
inline std::atomic<int>& boneResistOn(){return head_burst::boneResistOn();} // bonefix2: one pair of switches (HeadBurst.h)
inline std::atomic<int>& boneStrength(){return head_burst::boneStrength();}
template<class Table>
inline std::size_t keepResistingBone(Grid&g,const Table*table,std::vector<Cell>&cells,const head_burst::Plan&p){
 if(cells.empty()||(!table&&!g.material_slots)||!boneResistOn().load(std::memory_order_relaxed))return 0;
 const double s=std::clamp(double(boneStrength().load(std::memory_order_relaxed))/100.,0.,.95);
 if(!(s>0))return 0;
 // the crater as bits a tile, and its bone cells
 std::unordered_map<Cell,core::Tile,core::CellHash> cut;cut.reserve(cells.size()/48+16);
 std::vector<Cell> bone;
 {Cell last{INT_MIN,INT_MIN,INT_MIN};core::Tile*mask=nullptr;const SeverState::TileMaterial*m=nullptr;
  for(const Cell&c:cells){const Cell k=Grid::tileKey(c);
   if(!(k==last)){last=k;mask=&cut[k];if(table){const auto it=table->find(k);m=it==table->end()?nullptr:&it->second;}}
   const unsigned l=Grid::local(c);mask->removed[l>>6]|=std::uint64_t(1)<<(l&63);
   const bool skeletal=table ? m&&((m->bone[l>>6]>>(l&63))&1u) : g.renderMaterial(c)==2;
   if(skeletal&&!g.bonePreviouslyHit(c))bone.push_back(c);}}
 if(bone.empty())return 0;
 TissueLookup tissue{g,table};core::TileCache gone(g);
 auto inCut=[&](Cell c){const auto it=cut.find(Grid::tileKey(c));return it!=cut.end()&&core::tileBit(it->second,c);};
 // tissue that is still there and that this crater does not take
 auto survives=[&](Cell c){if(!g.valid(c))return false;return !inCut(c)&&tissue.solid(c)&&!gone.removed(c);};
 const double inv=1./g.pitch,stepL=.5*g.pitch,reach=std::max(p.Rcut,4*g.pitch);
 std::unordered_map<Cell,unsigned char,core::CellHash> kept; // a resisting bone cell: 1 once it holds on to the body
 for(const Cell&c:bone){const Vec x=g.center(c);Vec u=x-p.B;const double d=length(u);if(!(d>1e-6))continue;u=u*(1./d);
  // the crater's reach along u: the first point past the cell whose cell the crater does not take
  double R=d+reach;Cell lastC=c,lastK{INT_MIN,INT_MIN,INT_MIN};const core::Tile*lastT=nullptr;
  for(double t=d+stepL;t<=d+reach;t+=stepL){const Vec q=p.B+u*t;
   const Cell f{int(std::floor((q.x-g.origin.x)*inv)),int(std::floor((q.y-g.origin.y)*inv)),int(std::floor((q.z-g.origin.z)*inv))};
   if(f==lastC)continue;lastC=f;
   const Cell k=Grid::tileKey(f);if(!(k==lastK)){lastK=k;const auto it=cut.find(k);lastT=it==cut.end()?nullptr:&it->second;}
   if(!lastT||!core::tileBit(*lastT,f)){R=t;break;}}
  if(d>=(1.-s)*R)kept.emplace(c,(unsigned char)0);}
 if(kept.empty())return 0;
 static const int step[6][3]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
 std::vector<Cell> holding;holding.reserve(kept.size());
 for(auto&e:kept){for(const auto&st:step)if(survives(Cell{e.first.x+st[0],e.first.y+st[1],e.first.z+st[2]})){e.second=1;holding.push_back(e.first);break;}}
 for(std::size_t head=0;head<holding.size();++head){const Cell c=holding[head];
  for(const auto&st:step){const auto it=kept.find(Cell{c.x+st[0],c.y+st[1],c.z+st[2]});
   if(it!=kept.end()&&!it->second){it->second=1;holding.push_back(it->first);}}}
 if(holding.empty())return 0;
 std::unordered_map<Cell,core::Tile,core::CellHash> stay;
 for(const Cell&c:holding){g.noteBoneStrike(c);const unsigned l=Grid::local(c);stay[Grid::tileKey(c)].removed[l>>6]|=std::uint64_t(1)<<(l&63);}
 Cell last{INT_MIN,INT_MIN,INT_MIN};const core::Tile*t=nullptr;
 const auto end=std::remove_if(cells.begin(),cells.end(),[&](const Cell&c){const Cell k=Grid::tileKey(c);
  if(!(k==last)){last=k;const auto it=stay.find(k);t=it==stay.end()?nullptr:&it->second;}
  return t&&core::tileBit(*t,c);});
 const std::size_t n=std::size_t(cells.end()-end);cells.erase(end,cells.end());
 return n;}
inline Crater crater(Grid&g,const std::vector<SeverSegment>&segments,SeverState&state,const head_burst::Plan&p,const BulletSurfaceSource& source={}){
 const auto t0=core::Clock::now();Crater out;const double pitch=g.pitch;
 auto block=head_burst::record(p,pitch); // the record's own (float) numbers label the cells, as a burst's
 head_burst_record::Function fn;if(!head_burst_record::decode(*block,fn)){out.refusal="bullet_crater_record";return out;}
 const BulletTornFloor::CertifiedBoxes stockBoxes(fn);
 int tlo[3],thi[3];for(int k=0;k<3;++k){tlo[k]=std::max(0,int(std::floor((fn.B[k]-fn.Rcut-g.origin[k])/pitch))/8);thi[k]=std::min(((&g.size.x)[k]-1)/8,int(std::floor((fn.B[k]+fn.Rcut-g.origin[k])/pitch))/8);}
 if(tlo[0]>thi[0]||tlo[1]>thi[1]||tlo[2]>thi[2]){out.refusal="bullet_crater_outside_grid";return out;}
 const auto*table=state.table;
 // the tiles the cut's sphere meets
 std::vector<Cell> tiles;
 for(int z=tlo[2];z<=thi[2];++z)for(int y=tlo[1];y<=thi[1];++y)for(int x=tlo[0];x<=thi[0];++x){
  if(table&&!g.ownership&&table->find(Cell{x,y,z})==table->end())continue;
  const double bmin[3]={g.origin.x+(x*8+.5)*pitch,g.origin.y+(y*8+.5)*pitch,g.origin.z+(z*8+.5)*pitch};double c2=0;
  for(int a=0;a<3;++a){const double d=std::max(bmin[a]-fn.B[a],0.)+std::max(fn.B[a]-(bmin[a]+7*pitch),0.);c2+=d*d;}
  if(c2<=fn.Rcut*fn.Rcut)tiles.push_back(Cell{x,y,z});}
 CraterMargin spatial(fn,tiles,tlo,thi);
 const std::size_t W=std::max<std::size_t>(1,core::parallelWidth());std::vector<std::vector<Cell>> by(W);std::vector<std::size_t> nLab(W,0),nWhole(W,0),nSeen(W,0);
 std::atomic<std::size_t> cursor{0};
 constexpr std::size_t group=8;
 core::parallelChunks(std::min(W,(tiles.size()+group-1)/group),1,[&](std::size_t,std::size_t,std::size_t ch){CraterNoiseRow row;TissueLookup tissue{g,table};core::TileCache cache(g);std::uint64_t bits[8];
  for(std::size_t a;(a=cursor.fetch_add(group,std::memory_order_relaxed))<tiles.size();){
  const auto b=std::min(a+group,tiles.size());
  for(std::size_t i=a;i<b;++i){const Cell t=tiles[i];for(int q=0;q<8;++q)bits[q]=0;
   // the tile's kept tissue (TissueLookup's rule, as the burst's labelling reads it)
   if(table&&(!g.ownership||g.ownership->pureTile(Cell{t.x*8,t.y*8,t.z*8}))){const auto it=table->find(t);if(it==table->end())continue;const auto*tile=cache.find(t);
    for(int q=0;q<8;++q)bits[q]=it->second.solid[std::size_t(q)]&~(tile?tile->removed[std::size_t(q)]:0ull);}
   else for(unsigned l=0;l<512;++l){const Cell c{t.x*8+int(l>>6),t.y*8+int((l>>3)&7u),t.z*8+int(l&7u)};if(tissue.solid(c)&&!cache.removed(c))bits[l>>6]|=std::uint64_t(1)<<(l&63u);}
   std::uint64_t any=0;for(int q=0;q<8;++q)any|=bits[q];if(!any)continue;++nSeen[ch];
   // the whole tile's group when the record's own proof gives one (boxGroup: the warp's range over the box), else cell by cell
   const double bmin[3]={g.origin.x+(t.x*8+.5)*pitch,g.origin.y+(t.y*8+.5)*pitch,g.origin.z+(t.z*8+.5)*pitch},bmax[3]={bmin[0]+7*pitch,bmin[1]+7*pitch,bmin[2]+7*pitch};
   const unsigned tg=stockBoxes.test(bmin,bmax);
   if(tg==0)continue;
   if(tg!=255){++nWhole[ch];for(int q=0;q<8;++q)for(std::uint64_t w=bits[q];w;w&=w-1){const unsigned bit=head_burst::ctz64(w);by[ch].push_back(Cell{t.x*8+q,t.y*8+int(bit>>3),t.z*8+int(bit&7u)});}continue;}
   // a mixed tile: its eight 4^3 blocks the same way, then cell by cell
   for(int k=0;k<8;++k){const int bu=(k&1)*4,bv=((k>>1)&1)*4,bw=(k>>2)*4;
    std::uint64_t any4=0;for(int u=bu;u<bu+4;++u)for(int v=bv;v<bv+4;++v)any4|=(bits[u]>>(v*8+bw))&15u;if(!any4)continue;
    const double cmin[3]={bmin[0]+bu*pitch,bmin[1]+bv*pitch,bmin[2]+bw*pitch},cmax[3]={cmin[0]+3*pitch,cmin[1]+3*pitch,cmin[2]+3*pitch};
    const unsigned bg=stockBoxes.test(cmin,cmax);if(bg==0)continue;
    for(int u=bu;u<bu+4;++u)for(int v=bv;v<bv+4;++v){
      unsigned active=unsigned((bits[u]>>(v*8+bw))&15u);if(!active)continue;
      const Cell start{t.x*8+u,t.y*8+v,t.z*8+bw};
      if(bg==255){nLab[ch]+=std::size_t(__popcnt(active));active=stockingFour(spatial,g,fn,row,start,i,active);}
      while(active){const unsigned j=head_burst::ctz64(active);active&=active-1;
        by[ch].push_back({start.x,start.y,start.z+int(j)});}
    }}}}});

 std::vector<Cell> cells;for(std::size_t ch=0;ch<W;++ch){cells.insert(cells.end(),by[ch].begin(),by[ch].end());out.labelled+=nLab[ch];out.wholeTiles+=nWhole[ch];out.tiles+=nSeen[ch];}
 orderUniqueCraterCells(cells);
 BulletTornFloor floor;floor.spatial=&spatial;
 floor.prepare(g,cells,p,fn,source);
 floor.balance(g,cells,table);
 out.boneKept=keepResistingBone(g,table,cells,p); // bonefix2: bone outside the crater's middle stays standing
 out.labelMs=core::ms(t0);const auto e0=core::Clock::now();
 std::vector<Cell> boundary;std::set<Cell> touched;std::size_t solid=0; Cell lastTouched{-1,-1,-1};
 core::Tile* currentTile=nullptr;
 auto eraseLocal=[&](Cell c){
  if(!g.valid(c)||!g.ownerCuttable(c))return false;
  const Cell tile=Grid::tileKey(c);
  if(!(tile==lastTouched)){currentTile=&core::journaledTile(g,tile);lastTouched=tile;}
  const unsigned bit=Grid::local(c);auto& word=currentTile->removed[bit>>6];
  const std::uint64_t mask=std::uint64_t(1)<<(bit&63);
  if(word&mask)return false;
  word|=mask;return true;
 };
 CraterMaterialRow material;Cell lastReported{-1,-1,-1};
 boundary.reserve(cells.size());
 if(cells.size()>=3072){
  eraseCraterParallel(g,cells,boundary,touched,solid,out.bySlot);
  out.mask+=boundary.size();
 }else for(const Cell&c:cells){if(!eraseLocal(c))continue;boundary.push_back(c);const Cell tile=Grid::tileKey(c);if(!(tile==lastReported)){touched.insert(tile);lastReported=tile;}++out.mask;
  if(const auto label=material.get(g,c)){++solid;const auto slot=g.material_slots&&label<g.material_slots->size()?unsigned((*g.material_slots)[label]&63u):1u;++out.bySlot[slot];}}
 out.erased=boundary.size();out.eraseMs=core::ms(e0);const auto r0=core::Clock::now();
 std::size_t extraSolid=0;
 // (every cell of a checkerboard the erase left lies beside an erased cell: the nearest to the blast is taken, as for a crack)
 if(!boundary.empty()){try{out.edgeCells=resolveEdgeContacts(g,boundary,p.B,[](Cell){return true;},touched,&extraSolid,!g.ownership&&painter_detail::edgeWordsEnabled());}catch(const RecoverableImpactError&){out.refusal="bullet_crater_repair_budget";}}
 if(!out.refusal)try{floor.finishSupport(g,boundary,touched,out.bySlot,out.edgeCells,extraSolid);
    if(out.edgeCells>8192)throw RecoverableImpactError("edge_contact_regularization_budget");}
 catch(const RecoverableImpactError&){out.refusal="bullet_crater_repair_budget";}
 const auto skinMask=floor.skin(g,boundary,touched,fn,source,touched);
 out.mask+=skinMask;
 out.cells=solid+extraSolid;out.mask+=out.edgeCells;out.repairMs=core::ms(r0);
 out.dirty=core::expandDirtyTiles(touched);
 // the stations whose tiles changed are measured again (as every tear does)
 std::size_t total=0;for(const auto&s2:segments)total+=s2.stations.size();
 if(!out.dirty.empty()&&state.measured.size()==total){std::size_t k2=0;
  for(const auto&s2:segments)for(const auto&station:s2.stations){const std::size_t kk=k2++;if(!state.measured[kk])continue;
   for(const Cell&t:out.dirty)if(t.x>=station.tileLo.x&&t.x<=station.tileHi.x&&t.y>=station.tileLo.y&&t.y<=station.tileHi.y&&t.z>=station.tileLo.z&&t.z<=station.tileHi.z){state.measured[kk]=0;break;}}}
 out.done=out.erased>0;if(!out.done&&!out.refusal)out.refusal="bullet_crater_nothing_erased";
 out.ms=core::ms(t0);return out;}
} // namespace bullet_burst
