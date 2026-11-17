#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <bitset>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>
int mask_only_mode_value() noexcept; // gfy_live (LiveDamageWorker.cpp)
int sever_method_value() noexcept; // gfy_live (LiveDamageWorker.cpp): console painter_sever_method, 0 the old severs, 1 these (default)
int holding_cut_value() noexcept; // damagecut3: console painter_holding_cut (LiveDamageWorker.cpp), 0 (default) no holding-cells search, 1 on
namespace tear_decide {
constexpr double bridgeTaper=.012; // the torn depth's ramp from a wound pixel of the surface
// the old severs (console painter_sever_method 0, read at every sever: the next hit; or the probe knob GFY_DAMAGECUT_OFF)
inline bool bridgeOff(){static const bool o=std::getenv("GFY_DAMAGECUT_OFF")!=nullptr;return o||sever_method_value()==0;}
inline double bridgeRip(double pitch){return 1.5*pitch;} // a bridge tear's rip gap (the thin sheet lies inside it)
// The drawing maps of a bridge tear on the decision's section grid (call before the bridge cells are erased): BRIDGE, TAPER, CAP
// (Decision, TearDecide.h). sectionBridge: the CPU footprint (gpuBlock's section flags) is BRIDGE too. Returns the tear's slab
// (P_H): the deepest torn depth any drawn cell allows + 3 cm.
inline double bridgeMaps(Decision&D,const Grid&g,const SeverState&state,const std::vector<Cell>&bridge,double ripR,bool sectionBridge,std::size_t*woundOut=nullptr){
 const int N=D.N;const std::size_t NN=std::size_t(N)*std::size_t(N);const double*P=D.P;const double O=P[P_SEC],cl=P[P_SEC+1],pitch=g.pitch,RL=P[P_RL];
 const Vec C{P[P_C],P[P_C+1],P[P_C+2]},n{P[P_N],P[P_N+1],P[P_N+2]},u{P[P_U],P[P_U+1],P[P_U+2]},w{P[P_W],P[P_W+1],P[P_W+2]};
 D.bridge=true;D.sectionBridge=sectionBridge;D.BRIDGE.assign(NN,0);D.TAPER.assign(NN,1.f);D.CAP.assign(2*NN,0.f);
 std::vector<std::uint8_t> seed(NN,0);
 for(const Cell&c:bridge){const Vec d=g.center(c)-C;const int qa=int(std::floor((dot(d,u)-O)/cl)),qb=int(std::floor((dot(d,w)-O)/cl));
  if(qa>=0&&qb>=0&&qa<N&&qb<N)seed[std::size_t(qb)*std::size_t(N)+std::size_t(qa)]=1;}
 auto at=[&](int i){return O+(i+.5)*cl;};
 for(int y=0;y<N;++y)for(int x=0;x<N;++x){bool on=false;
  for(int dy=-1;dy<=1&&!on;++dy)for(int dx=-1;dx<=1;++dx){const int X=x+dx,Y=y+dy;if(X>=0&&Y>=0&&X<N&&Y<N&&seed[std::size_t(Y)*std::size_t(N)+std::size_t(X)]){on=true;break;}}
  const std::size_t q=std::size_t(y)*std::size_t(N)+std::size_t(x);const double A=at(x),B=at(y);
  if(on&&A*A+B*B<=RL*RL&&(sectionBridge||D.LIMB[q]))D.BRIDGE[q]=1;}
 D.bridgeShare=1;if(!sectionBridge){std::size_t nb=0,nl=0;for(std::size_t q=0;q<NN;++q){nb+=D.BRIDGE[q]?1u:0u;nl+=D.LIMB[q]?1u:0u;}if(nl)D.bridgeShare=double(nb)/double(nl);}
 // the surface's wound pixels (any of 2 x 2 points on it in removed tissue) and each drawn cell's standing depth per side
 std::vector<std::uint8_t> wnd(NN,0);const double step=.5*pitch,lim=P[P_MAXD]+ripR+pitch,R2=(RL+cl)*(RL+cl);
 // damagecut3 (speed; the same samples, the same answers): a surface without relief (every station and seam bridge) is the plane itself -
 // its offset is not evaluated; along a column a sample in the cell of the last one, or in a tile whose 512 cells all stand, needs
 // no lookup (the march runs 77 mm each way through mostly whole tissue)
 const bool flatB=P[P_SEEDC]==0&&P[P_SEEDC+1]==0&&P[P_SEEDC+2]==0&&P[P_SEEDC+3]==0;
 core::parallelChunks(std::size_t(N),1,[&](std::size_t y0,std::size_t y1,std::size_t){TissueLookup tl{g,state.table};core::TileCache rc(g);
  auto cellAt=[&](const Vec&p){return Cell{int(std::floor((p.x-g.origin.x)/pitch)),int(std::floor((p.y-g.origin.y)/pitch)),int(std::floor((p.z-g.origin.z)/pitch))};};
  // a tile all of whose cells stand: inside the grid, ownership decides nothing in it, the model's tissue everywhere, nothing removed
  // (then valid, TissueLookup::solid and !TileCache::removed hold for each of its cells); 16 remembered per thread
  Cell knownKey[16];bool knownFull[16];for(int i=0;i<16;++i){knownKey[i]=Cell{INT_MIN,INT_MIN,INT_MIN};knownFull[i]=false;}
  auto standingTile=[&](const Cell&k)->bool{const unsigned slot=(unsigned(k.x)*7u+unsigned(k.y)*13u+unsigned(k.z)*29u)&15u;if(knownKey[slot]==k)return knownFull[slot];
   bool full=false;const Cell c0{k.x*8,k.y*8,k.z*8},c1{c0.x+7,c0.y+7,c0.z+7};
   if(state.table&&g.valid(c0)&&g.valid(c1)&&(!g.ownership||g.ownership->pureTile(c0))){const auto it=state.table->find(k);
    if(it!=state.table->end()){full=true;for(int i=0;i<8&&full;++i)full=it->second.solid[std::size_t(i)]==~std::uint64_t(0);
     if(full)if(const core::Tile*tile=g.findTile(k))for(int i=0;i<8&&full;++i)full=tile->removed[std::size_t(i)]==0;}}
   knownKey[slot]=k;knownFull[slot]=full;return full;};
  for(std::size_t y=y0;y<y1;++y)for(int x=0;x<N;++x){const std::size_t q=y*std::size_t(N)+std::size_t(x);const double A=at(x),B=at(int(y));if(A*A+B*B>R2)continue;
   for(int k=0;k<4;++k){const double a=A+((k&1)?.25:-.25)*cl,b=B+((k&2)?.25:-.25)*cl;const Cell c=cellAt(flatB?C+u*a+w*b:C+u*a+w*b+n*D.offset(a,b));
    if(g.valid(c)&&tl.solid(c)&&rc.removed(c)){wnd[q]=1;break;}}
   if(!D.BRIDGE[q])continue; // the others keep the floor (CAP 0)
   const Vec base=flatB?C+u*A+w*B:C+u*A+w*B+n*D.offset(A,B);
   for(int side=0;side<2;++side){const double sg=side?1.:-1.;double f=-1;Cell last{INT_MIN,INT_MIN,INT_MIN},fullKey{INT_MIN,INT_MIN,INT_MIN};
    for(double t=0;t<=lim;t+=step){const Cell c=cellAt(base+n*(sg*t));
     if(!(c==last)){if(!g.valid(c))break;const Cell key=Grid::tileKey(c);
      if(!(key==fullKey)){if(!tl.solid(c)||rc.removed(c))break;if(standingTile(key))fullKey=key;}
      last=c;}
     f=t;}
    D.CAP[std::size_t(side)*NN+q]=float(std::max(0.,f+.5*step-ripR));}}});
 std::size_t wound=0;std::vector<double> dist(NN,1e20);for(std::size_t q=0;q<NN;++q)if(wnd[q]){dist[q]=0;++wound;}
 if(wound){edt2(dist,N);for(std::size_t q=0;q<NN;++q)D.TAPER[q]=float(sstep(0.,bridgeTaper,std::sqrt(dist[q])*cl-.5*cl));}
 double emax=eFloor(P[P_DSCALE]);for(std::size_t q=0;q<NN;++q)if(D.BRIDGE[q])for(int side=0;side<2;++side)emax=std::max(emax,double(D.CAP[std::size_t(side)*NN+q]));
 emax=std::min(emax,P[P_MAXD]);if(woundOut)*woundOut=wound;return emax+.03;}
} // namespace tear_decide

namespace holding_cut {
inline bool logOn(){static const bool l=std::getenv("GFY_HOLD_LOG")!=nullptr||std::getenv("GFY_M3_LOG")!=nullptr||std::getenv("GFY_TEAR_SELECT_LOG")!=nullptr;return l;}
constexpr double padM=.03,maxSideM=.40,alpha=.8,peakMinM=.005,minPieceM3=50e-6,estMax=.85;
constexpr int bandBlocks=2,maxCandidates=3;constexpr std::size_t maxBlocks=std::size_t(1)<<20,maxBand=600000,maxFlood=24000000;
inline unsigned pc(std::uint64_t v){return unsigned(std::bitset<64>(v).count());}
// a tile's static tissue, bone and removed bits: exactly TissueLookup::solid, the model's bone mask and TileCache::removed
struct TileBits {std::uint64_t solid[8],removed[8],bone[8];};
inline void tileBits(const Grid&g,const SeverState&st,const Cell&key,TileBits&b){
 for(int i=0;i<8;++i){b.solid[i]=0;b.removed[i]=0;b.bone[i]=0;}
 const Cell c0{key.x*8,key.y*8,key.z*8};const bool pure=!g.ownership||g.ownership->pureTile(c0);
 if(st.table&&pure){const auto it=st.table->find(key);if(it!=st.table->end())for(int i=0;i<8;++i){b.solid[i]=it->second.solid[i];b.bone[i]=it->second.bone[i];}}
 else for(unsigned l=0;l<512;++l){const Cell c{c0.x+int(l>>6),c0.y+int((l>>3)&7u),c0.z+int(l&7u)};if(!g.valid(c))continue;const auto lab=g.materialBase(c);if(!lab)continue;
  b.solid[l>>6]|=std::uint64_t(1)<<(l&63u);if(g.material_slots&&lab<g.material_slots->size()&&(*g.material_slots)[lab]==2)b.bone[l>>6]|=std::uint64_t(1)<<(l&63u);}
 if(const core::Tile*t=g.findTile(key))for(int i=0;i<8;++i)b.removed[i]=t->removed[i];
 if(!pure)for(unsigned l=0;l<512;++l){const Cell c{c0.x+int(l>>6),c0.y+int((l>>3)&7u),c0.z+int(l&7u)};if(g.valid(c)&&!g.ownerCuttable(c))b.removed[l>>6]|=std::uint64_t(1)<<(l&63u);}}
// one line of an exact squared distance transform (Felzenszwalb; TearDecide.h edt2's line): f[k * stride], entries >= 1e20 are no source
inline void edtLine(double*f,std::size_t n,std::size_t stride,std::vector<double>&fin,std::vector<double>&zb,std::vector<int>&vb){
 const double INF=1e20;fin.resize(n);zb.resize(n+1);vb.resize(n);for(std::size_t q=0;q<n;++q)fin[q]=f[q*stride];
 int k=-1;
 for(int q=0;q<int(n);++q){if(!(fin[std::size_t(q)]<INF))continue;double s=-1e30;
  while(k>=0){const int p=vb[std::size_t(k)];s=((fin[std::size_t(q)]+double(q)*q)-(fin[std::size_t(p)]+double(p)*p))/(2.*(q-p));if(s>zb[std::size_t(k)])break;--k;}
  ++k;vb[std::size_t(k)]=q;zb[std::size_t(k)]=k==0?-1e30:s;zb[std::size_t(k)+1]=1e30;}
 if(k<0){for(std::size_t q=0;q<n;++q)f[q*stride]=INF;return;}
 int j=0;for(int q=0;q<int(n);++q){while(zb[std::size_t(j)+1]<q)++j;const int p=vb[std::size_t(j)];f[std::size_t(q)*stride]=double(q-p)*(q-p)+fin[std::size_t(p)];}}
// unit-capacity max-flow on a split graph (Dinic, iterative): every source-sink path crosses a unit vertex edge, so each push is one unit
struct Flow {
 std::vector<int> head,nxt,to,cap,level,it,queue,path;int n=0;
 void init(int nodes,std::size_t edges){n=nodes;head.assign(std::size_t(nodes),-1);nxt.clear();to.clear();cap.clear();nxt.reserve(2*edges);to.reserve(2*edges);cap.reserve(2*edges);}
 void add(int a,int b,int c){to.push_back(b);cap.push_back(c);nxt.push_back(head[std::size_t(a)]);head[std::size_t(a)]=int(to.size())-1;
  to.push_back(a);cap.push_back(0);nxt.push_back(head[std::size_t(b)]);head[std::size_t(b)]=int(to.size())-1;}
 bool bfs(int S,int T){level.assign(std::size_t(n),-1);queue.clear();queue.push_back(S);level[std::size_t(S)]=0;
  for(std::size_t i=0;i<queue.size();++i){const int a=queue[i];for(int e=head[std::size_t(a)];e>=0;e=nxt[std::size_t(e)])if(cap[std::size_t(e)]>0&&level[std::size_t(to[std::size_t(e)])]<0){level[std::size_t(to[std::size_t(e)])]=level[std::size_t(a)]+1;queue.push_back(to[std::size_t(e)]);}}
  return level[std::size_t(T)]>=0;}
 long long run(int S,int T,long long limit){long long flow=0;
  while(flow<limit&&bfs(S,T)){it=head;
   for(;;){path.clear();int a=S;bool found=false;
    for(;;){if(a==T){found=true;break;}
     int&e=it[std::size_t(a)];while(e>=0&&!(cap[std::size_t(e)]>0&&level[std::size_t(to[std::size_t(e)])]==level[std::size_t(a)]+1))e=nxt[std::size_t(e)];
     if(e<0){if(a==S)break;level[std::size_t(a)]=-1;const int pe=path.back();path.pop_back();a=to[std::size_t(pe^1)];continue;}
     path.push_back(e);a=to[std::size_t(e)];}
    if(!found)break;
    for(const int e:path){cap[std::size_t(e)]-=1;cap[std::size_t(e^1)]+=1;}
    if(++flow>=limit)return flow;}}
  return flow;}
 std::vector<std::uint8_t> reach(int S)const{std::vector<std::uint8_t> r(std::size_t(n),0);std::vector<int> q{S};r[std::size_t(S)]=1;
  for(std::size_t i=0;i<q.size();++i){const int a=q[i];for(int e=head[std::size_t(a)];e>=0;e=nxt[std::size_t(e)])if(cap[std::size_t(e)]>0&&!r[std::size_t(to[std::size_t(e)])]){r[std::size_t(to[std::size_t(e)])]=1;q.push_back(to[std::size_t(e)]);}}
  return r;}
};
// one bit per cell, sparse by tile, with a one-tile cache (the proof's visit marks; the cut's cells)
struct CellBits {
 std::unordered_map<Cell,std::array<std::uint64_t,8>,core::CellHash> t;Cell ck{INT_MIN,INT_MIN,INT_MIN};std::array<std::uint64_t,8>*cp=nullptr;
 bool test(const Cell&c){const Cell k=Grid::tileKey(c);if(!(k==ck)){const auto it=t.find(k);if(it==t.end())return false;ck=k;cp=&it->second;}
  const unsigned l=Grid::local(c);return (((*cp)[l>>6]>>(l&63u))&1u)!=0;}
 void set(const Cell&c){const Cell k=Grid::tileKey(c);if(!(k==ck)||!cp){ck=k;cp=&t[k];}const unsigned l=Grid::local(c);(*cp)[l>>6]|=std::uint64_t(1)<<(l&63u);}
};
struct Report {std::size_t clusters{},tiles{},blocks{},basins{},pairs{},candidates{},measured{},bandCells{};double ms{},blockMs{},edtMs{},shedMs{},flowMs{},floodMs{},tearMs{};
 long long holding=-1,intact=-1,hb=0,hib=0,smallCells=0,largeCells=0;double t=1,b=1,est=1;const char*why="none";};
inline bool wanted(const TearGuide*guide,const painter_detail::TissueField*closure){
 // damagecut3: the search only with console painter_holding_cut 1 (default 0: sever() is never called - no cost; method 1 is then
 // the bridge tears of the station severs and of the seam rule)
 return holding_cut_value()!=0&&!tear_decide::bridgeOff()&&mask_only_mode_value()==3&&closure&&closure->available&&!(guide&&guide->exact_only);}

// The proof: with cutCells removed, floods from aSeeds and bSeeds (alternating; it stops when one is exhausted) never meet, and the
// exhausted side holds >= 50 cm3. why == nullptr: proven (side: 0 A / 1 B exhausted, cells: all of that side's cells).
struct Proof {const char*why="none";int side=-1;CellBits cells;Cell meet{INT_MIN,0,0};};
inline Proof separates(const Grid&g,const SeverState&state,const std::vector<Cell>&cutCells,const std::vector<Cell>&aSeeds,const std::vector<Cell>&bSeeds,Report&rep){
 const auto p0=core::Clock::now();CellBits cut,va,vb;for(const Cell&c:cutCells)cut.set(c);
 std::vector<Cell> sa,sb;std::size_t na=0,nbv=0;bool meetSeed=false;Cell meetCell{INT_MIN,0,0};
 TissueLookup tla{g,state.table},tlb{g,state.table};core::TileCache rca(g),rcb(g);
 auto standingA=[&](const Cell&c){return g.valid(c)&&tla.solid(c)&&!rca.removed(c)&&!cut.test(c);};
 auto standingB=[&](const Cell&c){return g.valid(c)&&tlb.solid(c)&&!rcb.removed(c)&&!cut.test(c);};
 for(const Cell&c:aSeeds)if(!va.test(c)&&standingA(c)){va.set(c);sa.push_back(c);++na;}
 for(const Cell&c:bSeeds)if(!vb.test(c)&&standingB(c)){if(va.test(c)){meetSeed=true;break;}vb.set(c);sb.push_back(c);++nbv;}
 bool met=meetSeed||sa.empty()||sb.empty(),doneA=false,doneB=false;
 auto grow=[&](std::vector<Cell>&st,CellBits&mine,CellBits&other,std::size_t&cnt,auto&&standing)->int{ // 0 more, 1 exhausted, 2 met
  for(int k=0;k<4096;++k){if(st.empty())return 1;const Cell c=st.back();st.pop_back();
   const Cell nb[6]={{c.x-1,c.y,c.z},{c.x+1,c.y,c.z},{c.x,c.y-1,c.z},{c.x,c.y+1,c.z},{c.x,c.y,c.z-1},{c.x,c.y,c.z+1}};
   for(const Cell&e:nb){if(mine.test(e)||!standing(e))continue;if(other.test(e)){meetCell=e;return 2;}mine.set(e);st.push_back(e);++cnt;}}
  return st.empty()?1:0;};
 while(!met&&!doneA&&!doneB&&na+nbv<maxFlood){ // the side with fewer cells grows (the work stays about twice the smaller part)
  if(na<=nbv){const int ra=grow(sa,va,vb,na,standingA);if(ra==2){met=true;break;}if(ra==1){doneA=true;break;}}
  else{const int rb=grow(sb,vb,va,nbv,standingB);if(rb==2){met=true;break;}if(rb==1){doneB=true;break;}}}
 rep.floodMs+=core::ms(p0);
 const char*why=nullptr;const std::size_t smallN=doneA?na:nbv,largeN=doneA?nbv:na;
 if(met)why="still_joined";else if(!(doneA||doneB))why="flood_budget";else if(double(smallN)*g.pitch*g.pitch*g.pitch<minPieceM3)why="small_piece";
 else{rep.smallCells=(long long)smallN;rep.largeCells=(long long)largeN;}
 if(logOn())std::cerr<<"damagecut_hold proof="<<(why?why:"separated")<<" cut_cells="<<cutCells.size()<<" a="<<na<<" b="<<nbv<<" small_cm3="<<double(smallN)*g.pitch*g.pitch*g.pitch*1e6<<" ms="<<core::ms(p0)<<std::endl;
 Proof out;out.why=why;out.meet=meetCell;if(!why){out.side=doneA?0:1;out.cells=std::move(doneA?va:vb);}return out;}

// The cut through the holding cells H (standing; the A side to the B side along nAB): the parting surface fitted through H (the
// decision's offset: tilt and bow, least squares); the cells it removes = the standing cells of that surface's thin sheet inside the
// band (bandStanding) when they part the two sides (proof), else H itself (proof); one bridge tear record; the cells erased (+ edge
// repair). false: neither parts them (nothing changed).
inline bool generalTear(Grid&g,SeverState&state,const painter_detail::TissueField*closure,const std::vector<Cell>&Hc,const std::vector<Cell>&aSide,const std::vector<Cell>&bSide,
    const std::vector<Cell>&bandStanding,const Vec&nAB,const Vec&fibre,std::uint64_t seed64,const Vec*impact,const std::vector<SeverSegment>&segments,DamageTearVerdict&v,Report&rep){
 using namespace tear_decide;const double pitch=g.pitch;
 Vec C{0,0,0};for(const Cell&c:Hc)C=C+g.center(c);C=C*(1./double(Hc.size()));
 Vec n=unit(nAB);if(!(std::isfinite(n.x)&&std::isfinite(n.y)&&std::isfinite(n.z)))n=Vec{0,0,1};
 // the parting surface's normal: the holding cells' least-variance axis (they spread over the surface), oriented A -> B
 {double M[3][3]={};for(const Cell&c:Hc){const Vec d=g.center(c)-C;for(int i=0;i<3;++i)for(int j=0;j<3;++j)M[i][j]+=d[i]*d[j];}
  double V[3][3]={{1,0,0},{0,1,0},{0,0,1}};
  for(int sweep=0;sweep<32;++sweep){double off=0;for(int i=0;i<3;++i)for(int j=i+1;j<3;++j)off+=M[i][j]*M[i][j];if(off<1e-30)break;
   for(int pq=0;pq<3;++pq){const int i=pq==2?1:0,j=pq==0?1:2;if(std::abs(M[i][j])<1e-30)continue;
    const double th=.5*std::atan2(2*M[i][j],M[j][j]-M[i][i]),c=std::cos(th),s=std::sin(th);
    for(int k=0;k<3;++k){const double a=M[k][i],b=M[k][j];M[k][i]=c*a-s*b;M[k][j]=s*a+c*b;}
    for(int k=0;k<3;++k){const double a=M[i][k],b=M[j][k];M[i][k]=c*a-s*b;M[j][k]=s*a+c*b;}
    for(int k=0;k<3;++k){const double a=V[k][i],b=V[k][j];V[k][i]=c*a-s*b;V[k][j]=s*a+c*b;}}}
  int lo=0;for(int i=1;i<3;++i)if(M[i][i]<M[lo][lo])lo=i;
  Vec e{V[0][lo],V[1][lo],V[2][lo]};const double el=length(e);
  // a cut that is a surface (two axes much wider than the third): its least-variance axis; a strand or a blob keeps nAB
  double second=1e30;for(int i=0;i<3;++i)if(i!=lo)second=std::min(second,M[i][i]);
  if(el>1e-9&&Hc.size()>=8&&second>4*std::max(M[lo][lo],1e-12)){e=e*(1./el);n=dot(e,n)<0?e*-1.:e;}}
 const Vec u=unit(cross(n,std::abs(n.x)<.8?Vec{1,0,0}:Vec{0,1,0})),w=cross(n,u);
 double rH=0;for(const Cell&c:Hc){const Vec d=g.center(c)-C;const double pa=dot(d,u),pb=dot(d,w);rH=std::max(rH,std::sqrt(pa*pa+pb*pb));}
 const double R=std::max(.03,rH+.02),ripR=bridgeRip(pitch),half=.07,maxD=tissue_tear::limbDepth;
 const std::uint32_t seed=std::uint32_t(seed64^(seed64>>32));
 Decision D;D.pitch=pitch;D.seed=seed;double*P=D.P;
 for(int k=0;k<3;++k){P[P_C+k]=C[k];P[P_N+k]=n[k];P[P_U+k]=u[k];P[P_W+k]=w[k];}
 P[P_RL]=R;P[P_H]=std::max(half,maxD+.03);P[P_MAXD]=maxD;P[P_RIP]=ripR;P[P_DSCALE]=1.;
 // decideOnce's impact terms and fibre frames (the part's fibres along its bone axis)
 double ia=0,ib=0,along=0;{const double L=impact?length(*impact):0.;if(L>1e-9){const Vec d=*impact*(1./L);ia=dot(d,u);ib=dot(d,w);along=dot(d,n);}
  double l2=std::sqrt(ia*ia+ib*ib);if(l2<.2){const double th=6.283185307179586*tissue_tear::u01(seed,5,5,0x77u);ia=std::cos(th);ib=std::sin(th);l2=1;}ia/=l2;ib/=l2;}
 const double tilt=(2*tissue_tear::u01(seed,6,6,0x78u)-1)*.016;
 double mbase[2];for(int sd=0;sd<2;++sd){const std::uint32_t s=seed^(sd?0x51u:0xA3u);double m=0;for(unsigned q=0;q<8;++q)m+=tissue_tear::u01(s,int(100+q),7,0x3C1u);
  mbase[sd]=tissue_tear::limbK::groupFloor+tissue_tear::limbK::groupSpan*m/8;}
 P[P_IMP]=ia;P[P_IMP+1]=ib;P[P_IMP+2]=along;P[P_IMP+3]=tilt;P[P_IMP+4]=std::max(R,.02);P[P_IMP+5]=maxD;P[P_IMP+6]=mbase[0];P[P_IMP+7]=mbase[1];
 {Vec A0=unit(fibre);if(!(std::isfinite(A0.x)&&std::isfinite(A0.y)&&std::isfinite(A0.z)))A0=n;
  Vec p1=cross(A0,Vec{0,0,1});const double l1=dot(p1,p1);p1=l1>.04?p1*(1./std::sqrt(l1)):unit(cross(A0,Vec{1,0,0}));const Vec p2=cross(A0,p1);
  static const double tiltDeg[8]={2.5,17.5,7.5,35,25,12.5,42.5,5},azDeg[8]={0,40,200,110,300,150,250,80};const double rad=3.141592653589793/180;
  const Vec dimp=u*ia+w*ib;
  for(unsigned c=0;c<NCLS;++c){bool fib=false;Vec d=A0;
   if(c>=MV0&&c<MV0+8){const double t=tiltDeg[c-MV0]*rad,z=azDeg[c-MV0]*rad;d=unit(A0*std::cos(t)+p1*(std::sin(t)*std::cos(z))+p2*(std::sin(t)*std::sin(z)));fib=true;}
   else if(c==MUSCLE)fib=true;
   Vec f1=u,f2=w;double ca=ia,cb=ib;
   if(fib){const Vec r=std::abs(d.x)<.6?Vec{1,0,0}:(std::abs(d.y)<.6?Vec{0,1,0}:Vec{0,0,1});f1=unit(cross(d,r));f2=cross(d,f1);ca=dot(dimp,f1);cb=dot(dimp,f2);}
   const int k=P_FE+8*int(c);for(int a=0;a<3;++a){P[k+a]=f1[a];P[k+3+a]=f2[a];}P[k+6]=ca;P[k+7]=cb;}}
 auto sample=[&](double x,double y,double z){return closure->sample(x,y,z);};
 auto parallel=[&](std::size_t count,const auto&body){core::parallelChunks(count,1,[&](std::size_t a,std::size_t b,std::size_t){body(a,b);});};
 SectionCache cache;buildSectionFast(D,sample,parallel,[&](double a,double b){return a*a+b*b<=R*R;},cache);
 // the parting surface through the holding cells: the offset's tilt and bow fitted to them (least squares), no seeded relief
 for(int q=0;q<7;++q)P[P_SEEDC+q]=0.;
 {const double RL=P[P_LIMB+3];double M[3][3]={},r[3]={};
  for(const Cell&c:Hc){const Vec d=g.center(c)-C;const double x=dot(d,u)/RL,y=dot(d,w)/RL,s=dot(d,n);const double f[3]={RL*x,RL*y,x*x+y*y-.5};
   for(int i=0;i<3;++i){r[i]+=f[i]*s;for(int j=0;j<3;++j)M[i][j]+=f[i]*f[j];}}
  auto det3=[](const double m[3][3]){return m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);};
  const double dt=det3(M);
  if(std::abs(dt)>1e-18)for(int i=0;i<3;++i){double m2[3][3];for(int a=0;a<3;++a)for(int b=0;b<3;++b)m2[a][b]=b==i?r[a]:M[a][b];const double val=det3(m2)/dt;if(std::isfinite(val))P[P_SEEDC+i]=val;}}
 finishSeeds(D,parallel);
 // the sheet of the fitted surface: the standing cells of its thin layer joined (26 neighbours, inside the layer) to the holding cells
 // (a station sheet on a free surface over the whole section it crosses; the thinnest that parts: 0.75 cell, then 1.25 cells),
 // else the band's own cells near the surface
 std::vector<Cell> cutCells;const char*kind="sheet";Proof proof;
 {auto sOf=[&](const Cell&c){const Vec d=g.center(c)-C;return dot(d,n)-D.offset(dot(d,u),dot(d,w));};
  for(int pass=0;pass<4&&cutCells.empty();++pass){const double hs=(pass&1)?1.25*pitch:.75*pitch;std::vector<Cell> sheet,sa,sb;const std::size_t cap=2*Hc.size()+200;
   if(pass<2){CellBits seen;std::vector<Cell> st;TissueLookup tl{g,state.table};core::TileCache rc(g);bool over=false;
    auto standing=[&](const Cell&c){return g.valid(c)&&tl.solid(c)&&!rc.removed(c);};
    for(const Cell&c:Hc){seen.set(c);st.push_back(c);if(std::abs(sOf(c))<=hs)sheet.push_back(c);}
    while(!st.empty()&&!over){const Cell c=st.back();st.pop_back();
     for(int dx=-1;dx<=1&&!over;++dx)for(int dy=-1;dy<=1&&!over;++dy)for(int dz=-1;dz<=1;++dz){const Cell e{c.x+dx,c.y+dy,c.z+dz};if(seen.test(e))continue;seen.set(e);
      if(!standing(e)||std::abs(sOf(e))>hs)continue;sheet.push_back(e);st.push_back(e);if(sheet.size()>cap){over=true;break;}}}
    if(over)continue; // a surface through far more than the holding cells: not this neck's
    CellBits in;for(const Cell&c:sheet)in.set(c);
    for(const Cell&c:sheet){const Cell nb[6]={{c.x-1,c.y,c.z},{c.x+1,c.y,c.z},{c.x,c.y-1,c.z},{c.x,c.y+1,c.z},{c.x,c.y,c.z-1},{c.x,c.y,c.z+1}};
     for(const Cell&e:nb){if(in.test(e)||!standing(e))continue;(sOf(e)<0?sa:sb).push_back(e);}}}
   else for(const Cell&c:bandStanding){const double s=sOf(c);if(std::abs(s)<=hs)sheet.push_back(c);else if(s<0&&s>=-hs-2*pitch)sa.push_back(c);else if(s>0&&s<=hs+2*pitch)sb.push_back(c);}
   const std::vector<Cell>*pool=pass<2?nullptr:&bandStanding;
   if(sheet.empty()||sheet.size()>cap)continue;
   Proof pr=separates(g,state,sheet,sa,sb,rep);if(!pr.why){cutCells=std::move(sheet);proof=std::move(pr);if(pool)kind="band_sheet";}
   else if(logOn()&&pr.meet.x!=INT_MIN){const Vec d=g.center(pr.meet)-C;const double pa=dot(d,u),pb=dot(d,w); // where the two sides still meet
    std::cerr<<"damagecut_hold sheet_leak hs_mm="<<hs*1e3<<" s_mm="<<(dot(d,n)-D.offset(pa,pb))*1e3<<" r_mm="<<std::sqrt(pa*pa+pb*pb)*1e3<<" R_mm="<<P[P_RL]*1e3
     <<" at=("<<g.center(pr.meet).x<<","<<g.center(pr.meet).y<<","<<g.center(pr.meet).z<<") C=("<<C.x<<","<<C.y<<","<<C.z<<") sheet="<<sheet.size()<<" seeds="<<sa.size()<<"/"<<sb.size()<<std::endl;}}
  if(cutCells.empty()){Proof pr=separates(g,state,Hc,aSide,bSide,rep);if(pr.why){rep.why="no_proof";return false;}cutCells=Hc;kind="min_cut";proof=std::move(pr);}}
 std::size_t woundPx=0;D.bridgeH=bridgeMaps(D,g,state,cutCells,ripR,true,&woundPx);P[P_H]=D.bridgeH;
 // the region keeps the columns where the cut lies on the fitted surface (2 cells) and the cells 2.5 / 4.5 / 7.5 cells off it on each
 // side belong to that side's proven part (the pieces' vote and the programs' side test then name the real parts); the cut cells
 // of a dropped column are drawn as the wounds are (the removal fields)
 std::size_t pruned=0,kept=0;
 {const int N=D.N;const std::size_t NN=std::size_t(N)*std::size_t(N);const double O=P[P_SEC],cl=P[P_SEC+1];std::vector<float> worst(NN,0.f);
  auto cellAt=[&](const Vec&q){return Cell{int(std::floor((q.x-g.origin.x)/pitch)),int(std::floor((q.y-g.origin.y)/pitch)),int(std::floor((q.z-g.origin.z)/pitch))};};
  for(const Cell&c:cutCells){const Vec d=g.center(c)-C;const double pa=dot(d,u),pb=dot(d,w);const int qa=int(std::floor((pa-O)/cl)),qb=int(std::floor((pb-O)/cl));
   if(qa>=0&&qb>=0&&qa<N&&qb<N){auto&wv=worst[std::size_t(qb)*std::size_t(N)+std::size_t(qa)];wv=std::max(wv,float(std::abs(dot(d,n)-D.offset(pa,pb))));}}
  TissueLookup tl{g,state.table};core::TileCache rc(g);
  for(int y=0;y<N;++y)for(int x=0;x<N;++x){const std::size_t q=std::size_t(y)*std::size_t(N)+std::size_t(x);if(!D.BRIDGE[q])continue;
   bool bad=worst[q]>2*pitch;const double A=O+(x+.5)*cl,B=O+(y+.5)*cl,o=D.offset(A,B);
   for(const double e:{2.5*pitch,4.5*pitch,7.5*pitch})for(const int sd:{-1,1}){if(bad)break;const Cell c=cellAt(C+u*A+w*B+n*(o+sd*e));
    if(!g.valid(c)||!tl.solid(c)||rc.removed(c))continue;const bool exhaustedSide=proof.side==0?sd<0:sd>0;if(exhaustedSide!=proof.cells.test(c))bad=true;}
   if(bad){D.BRIDGE[q]=0;++pruned;}else ++kept;}}
 {const int N=D.N;const double O=P[P_SEC],cl=P[P_SEC+1];
  for(int y=0;y<N;++y)for(int x=0;x<N;++x)if(D.BRIDGE[std::size_t(y)*std::size_t(N)+std::size_t(x)]){const double A=O+(x+.5)*cl,B=O+(y+.5)*cl;
   D.footLo[0]=std::min(D.footLo[0],A-cl/2);D.footHi[0]=std::max(D.footHi[0],A+cl/2);D.footLo[1]=std::min(D.footLo[1],B-cl/2);D.footHi[1]=std::max(D.footHi[1],B+cl/2);}}
 // the cut cells go into the grid (journaled erase, the same repair every cut gets)
 std::vector<Cell> boundary;std::set<Cell> touched;std::size_t solid=0;
 for(const Cell&c:cutCells){if(!g.valid(c)||!g.erase(c))continue;boundary.push_back(c);touched.insert(Grid::tileKey(c));++v.mask;
  if(const auto label=g.materialBase(c)){++solid;const auto slot=g.material_slots&&label<g.material_slots->size()?unsigned((*g.material_slots)[label]&63u):1u;++v.bySlot[slot];}}
 std::size_t extraSolid=0;
 if(!boundary.empty()){const std::set<Cell> hs(cutCells.begin(),cutCells.end());
  try{v.edgeCells=resolveEdgeContacts(g,boundary,C,[&](Cell c){for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)for(int dz=-1;dz<=1;++dz)if(hs.count(Cell{c.x+dx,c.y+dy,c.z+dz}))return true;return false;},touched,&extraSolid);}
  catch(const RecoverableImpactError&){v.refusal="tear_repair_budget";}}
 v.cells=solid+extraSolid;v.mask+=v.edgeCells;v.dirty=core::expandDirtyTiles(touched);
 std::size_t total=0;for(const auto&s2:segments)total+=s2.stations.size();
 if(!v.dirty.empty()&&state.measured.size()==total){std::size_t k2=0;
  for(const auto&s2:segments)for(const auto&station:s2.stations){const std::size_t kk=k2++;if(!state.measured[kk])continue;
   for(const Cell&t:v.dirty)if(t.x>=station.tileLo.x&&t.x<=station.tileHi.x&&t.y>=station.tileLo.y&&t.y<=station.tileHi.y&&t.z>=station.tileLo.z&&t.z<=station.tileHi.z){state.measured[kk]=0;break;}}}
 // the drawing (decideOnce's: the GPU block, the bake beside the rest of the job when the job keeps the closure field, the record)
 auto block=gpuBlock(D);block->reach=10*pitch; // the pieces' vote and the coverage box near the parting surface (fitted, not a limb's plane)
 static const double bakeMm=[]{const char*e=std::getenv("GFY_M3_BAKE_MM");const double x=e?std::atof(e):4.;return std::clamp(x,1.,8.);}();
 static const double mapMm=[]{const char*e=std::getenv("GFY_M3_MAP_MM");const double x=e?std::atof(e):4.;return std::clamp(x,1.,8.);}();
 {const auto&keep=tear_bake::jobKeep();
  if(!tear_bake::syncOnly()&&keep&&keep.get()==static_cast<const void*>(closure)){const auto*cl=closure;auto sampleV=[cl](double x,double y,double z){return cl->sample(x,y,z);};
   auto p=bakeTearChunks(std::make_shared<const Decision>(D),bakeMm*.001,mapMm*.001,sampleV,block,std::function<void(const Bake2Stats&,double)>());p->keep=keep;tear_bake::start(block.get(),p);}
  else{Bake2Stats bst;bakeTear(D,bakeMm*.001,mapMm*.001,sample,parallel,*block,bst);}}
 {ResolvedTunnel rec;const double reach=half+.025;const Vec e=C-n*reach;
  rec.entry={e.x,e.y,e.z};rec.direction={n.x,n.y,n.z};rec.edge_u={u.x,u.y,u.z};rec.edge_v={w.x,w.y,w.z};rec.radius=R;rec.penetration=2*reach;rec.cap=2*reach;rec.inlet=0;
  rec.exit_distance=2*reach;rec.exit_mult=1;rec.edge_seed=seed64;auto surf=std::make_shared<ResolvedTearSurface>();surf->decide=block;rec.surface=std::move(surf);v.records.push_back(std::move(rec));}
 ++state.decided;v.point=C;v.axis=n;v.tissueRan=true;v.crossing=0;v.rounds=0;v.bridgeCells=cutCells.size();v.general=true;v.tear=!v.refusal;
 // the stations in the torn region are done (decideOnce's rule): within 2 cm of the parting surface over its footprint
 if(v.tear&&state.failed.size()==total){std::size_t k=0;
  for(const auto&s2:segments)for(const auto&station:s2.stations){const Vec d=station.center-C;const double pa=dot(d,u),pb=dot(d,w);
   if(std::abs(dot(d,n)-D.offset(pa,pb))<=.02&&D.inFoot(pa,pb))state.failed[k]=1;++k;}}
 if(logOn())std::cerr<<"damagecut_bridge general cells="<<cutCells.size()<<" erased="<<boundary.size()<<" edge_cells="<<v.edgeCells<<" wound_px="<<woundPx<<" emax_mm="<<(D.bridgeH-.03)*1e3
  <<" radius_mm="<<R*1e3<<" N="<<D.N<<" normal=("<<n.x<<","<<n.y<<","<<n.z<<") fibre_cos="<<std::abs(dot(n,unit(fibre)))<<" bow_mm="<<P[P_SEEDC+2]*1e3<<" kind="<<kind<<" holding="<<Hc.size()<<" region_cols="<<kept<<" dropped_cols="<<pruned<<std::endl;
 return v.tear;}

// A holding-cells sever in the region of tiles [tlo, thi] (inclusive). Returns true when it cut (v holds the tear).
// The coarse graph is cell-exact: a block holds the mask of its standing cells; two face neighbours are joined only through a
// standing cell pair across their shared face; a block whose standing cells fall apart inside it is a gap (a thin cut through it).
inline bool cutIn(Grid&g,SeverState&state,const int tlo[3],const int thi[3],double severScale,bool goneOnly,const painter_detail::TissueField*closure,
    const Vec*impact,const std::vector<SeverSegment>&segments,std::uint64_t seed64,DamageTearVerdict&v,Report&rep){
 const double pitch=g.pitch;const int b=pitch<.0015?4:2,bpt=8/b,b3=b*b*b;
 const int BX=(thi[0]-tlo[0]+1)*bpt,BY=(thi[1]-tlo[1]+1)*bpt,BZ=(thi[2]-tlo[2]+1)*bpt;const std::size_t NB=std::size_t(BX)*BY*BZ;
 if(NB>maxBlocks){rep.why="region_blocks";return false;}
 rep.blocks+=NB;rep.tiles+=std::size_t(thi[0]-tlo[0]+1)*(thi[1]-tlo[1]+1)*(thi[2]-tlo[2]+1);
 auto bi=[&](int x,int y,int z){return (std::size_t(x)*std::size_t(BY)+std::size_t(y))*std::size_t(BZ)+std::size_t(z);};
 auto bx3=[&](std::size_t i,int&x,int&y,int&z){x=int(i/(std::size_t(BY)*BZ));y=int((i/std::size_t(BZ))%std::size_t(BY));z=int(i%std::size_t(BZ));};
 // ---- blocks: the mask of their standing cells (bit (x b + y) b + z), their wound (removed tissue) cell count
 auto b0=core::Clock::now();
 std::vector<std::uint64_t> sm(NB,0);std::vector<std::uint8_t> gone(NB,0);std::atomic<std::size_t> goneTotal{0};
 const int ntx=thi[0]-tlo[0]+1,nty=thi[1]-tlo[1]+1,ntz=thi[2]-tlo[2]+1;
 core::parallelChunks(std::size_t(ntx),1,[&](std::size_t x0,std::size_t x1,std::size_t){TileBits tb;std::size_t mine=0;
  for(int tx=int(x0);tx<int(x1);++tx)for(int ty=0;ty<nty;++ty)for(int tz=0;tz<ntz;++tz){tileBits(g,state,Cell{tlo[0]+tx,tlo[1]+ty,tlo[2]+tz},tb);
   for(int x=0;x<8;++x){const std::uint64_t sw=tb.solid[x]&~tb.removed[x],gw=tb.solid[x]&tb.removed[x];if(!sw&&!gw)continue;
    for(unsigned l=0;l<64;++l){const std::uint64_t bit=std::uint64_t(1)<<l;const bool s=(sw&bit)!=0,q=(gw&bit)!=0;if(!s&&!q)continue;const int y=int(l>>3),z=int(l&7u);
     const std::size_t i=bi(tx*bpt+x/b,ty*bpt+y/b,tz*bpt+z/b);
     if(s)sm[i]|=std::uint64_t(1)<<(((x%b)*b+(y%b))*b+(z%b));else{gone[i]=std::uint8_t(gone[i]+1);++mine;}}}}
  goneTotal+=mine;});
 if(!goneTotal.load()){rep.blockMs+=core::ms(b0);rep.why="no_wound";return false;}
 std::uint64_t xLo=0,xHi=0,yLo=0,yHi=0,zLo=0,zHi=0;
 for(int x=0;x<b;++x)for(int y=0;y<b;++y)for(int z=0;z<b;++z){const std::uint64_t bit=std::uint64_t(1)<<((x*b+y)*b+z);
  if(x==0)xLo|=bit;if(x==b-1)xHi|=bit;if(y==0)yLo|=bit;if(y==b-1)yHi|=bit;if(z==0)zLo|=bit;if(z==b-1)zHi|=bit;}
 const int sx=b*b,sy=b;
 auto dil=[&](std::uint64_t m){return m|((m&~zHi)<<1)|((m&~zLo)>>1)|((m&~yHi)<<sy)|((m&~yLo)>>sy)|((m&~xHi)<<sx)|((m&~xLo)>>sx);};
 auto split=[&](std::uint64_t m){if(!m)return false;std::uint64_t cur=m&(~m+1);for(;;){const std::uint64_t nx=dil(cur)&m;if(nx==cur)break;cur=nx;}return cur!=m;};
 std::vector<std::uint8_t> stand(NB,0);const std::uint64_t full=b3>=64?~std::uint64_t(0):((std::uint64_t(1)<<b3)-1);
 core::parallelChunks(NB,4096,[&](std::size_t a,std::size_t e,std::size_t){for(std::size_t i=a;i<e;++i){const std::uint64_t m=sm[i];stand[i]=m&&(m==full||!split(m))?1:0;}});
 // joined across the face from block i to its +axis neighbour j
 auto conn=[&](std::size_t i,std::size_t j,int axis){const std::uint64_t P=sm[i],Q=sm[j];
  if(axis==0)return (((P&xHi)>>(sx*(b-1)))&(Q&xLo))!=0;if(axis==1)return (((P&yHi)>>(sy*(b-1)))&(Q&yLo))!=0;return (((P&zHi)>>(b-1))&(Q&zLo))!=0;};
 const std::size_t step[3]={std::size_t(BY)*BZ,std::size_t(BZ),1};
 // the six face neighbours of block i joined to it (standing both, a standing face pair)
 auto linked=[&](std::size_t i,int x,int y,int z,int k,std::size_t&j)->bool{const int axis=k>>1,sg=(k&1)?1:-1;const int c[3]={x,y,z};const int lim[3]={BX,BY,BZ};
  const int q=c[axis]+sg;if(q<0||q>=lim[axis])return false;j=sg>0?i+step[axis]:i-step[axis];if(!stand[j])return false;return sg>0?conn(i,j,axis):conn(j,i,axis);};
 rep.blockMs+=core::ms(b0);
 // ---- the distance of every standing block to the nearest block that is not (outside the region: unknown, no source)
 b0=core::Clock::now();
 std::vector<double> d2(NB);for(std::size_t i=0;i<NB;++i)d2[i]=stand[i]?1e20:0.;
 {const std::size_t SX=std::size_t(BY)*BZ,SY=std::size_t(BZ);
  core::parallelChunks(std::size_t(BX)*BY,64,[&](std::size_t a,std::size_t e,std::size_t){std::vector<double> fin,zb;std::vector<int> vb;for(std::size_t l=a;l<e;++l)edtLine(&d2[l*BZ],std::size_t(BZ),1,fin,zb,vb);});
  core::parallelChunks(std::size_t(BX)*BZ,64,[&](std::size_t a,std::size_t e,std::size_t){std::vector<double> fin,zb;std::vector<int> vb;
   for(std::size_t l=a;l<e;++l){const std::size_t x=l/std::size_t(BZ),z=l%std::size_t(BZ);edtLine(&d2[x*SX+z],std::size_t(BY),SY,fin,zb,vb);}});
  core::parallelChunks(std::size_t(BY)*BZ,64,[&](std::size_t a,std::size_t e,std::size_t){std::vector<double> fin,zb;std::vector<int> vb;for(std::size_t l=a;l<e;++l)edtLine(&d2[l],std::size_t(BX),SX,fin,zb,vb);});}
 rep.edtMs+=core::ms(b0);
 // ---- watershed from the deepest blocks down (counting sort on the integer squared distances), over joined faces only
 b0=core::Clock::now();
 const double dcap=double(std::size_t(BX)*BX+std::size_t(BY)*BY+std::size_t(BZ)*BZ); // no real squared distance is larger (no source: the cap)
 std::uint32_t dmax=0;for(std::size_t i=0;i<NB;++i)if(stand[i])dmax=std::max(dmax,std::uint32_t(std::min(d2[i],dcap)));
 std::vector<std::uint32_t> count(std::size_t(dmax)+2,0),order;
 for(std::size_t i=0;i<NB;++i)if(stand[i])++count[std::size_t(std::min(d2[i],dcap))];
 {std::uint32_t acc=0;for(std::size_t k=count.size();k-->0;){const std::uint32_t c=count[k];count[k]=acc;acc+=c;}order.assign(acc,0);}
 for(std::size_t i=0;i<NB;++i)if(stand[i])order[count[std::size_t(std::min(d2[i],dcap))]++]=std::uint32_t(i);
 std::vector<std::int32_t> lab(NB,-1),par;std::vector<double> peak;std::vector<std::uint32_t> vol;std::vector<std::uint8_t> touch;
 auto find=[&](std::int32_t a){while(par[std::size_t(a)]!=a){par[std::size_t(a)]=par[std::size_t(par[std::size_t(a)])];a=par[std::size_t(a)];}return a;};
 const double peakMin=peakMinM/(b*pitch);
 for(const std::uint32_t i:order){int x,y,z;bx3(i,x,y,z);const double d=std::sqrt(d2[i]);
  std::int32_t roots[6];int nr=0;
  for(int k=0;k<6;++k){std::size_t j;if(!linked(i,x,y,z,k,j))continue;const std::int32_t l=lab[j];if(l<0)continue;
   const std::int32_t r=find(l);bool seen=false;for(int q=0;q<nr;++q)seen=seen||roots[q]==r;if(!seen)roots[nr++]=r;}
  const bool edge=x==0||y==0||z==0||x==BX-1||y==BY-1||z==BZ-1;
  if(!nr){const std::int32_t id=std::int32_t(par.size());par.push_back(id);peak.push_back(d);vol.push_back(1);touch.push_back(edge?1:0);lab[i]=id;continue;}
  std::int32_t p=roots[0];for(int k=1;k<nr;++k)if(peak[std::size_t(roots[k])]>peak[std::size_t(p)])p=roots[k];
  for(int k=0;k<nr;++k){const std::int32_t r=roots[k];if(r==p)continue;const double lo=std::min(peak[std::size_t(r)],peak[std::size_t(p)]);
   if(lo<peakMin||d>alpha*lo){par[std::size_t(r)]=p;vol[std::size_t(p)]+=vol[std::size_t(r)];touch[std::size_t(p)]|=touch[std::size_t(r)];}}
  lab[i]=p;++vol[std::size_t(p)];if(edge)touch[std::size_t(p)]=1;}
 for(std::size_t i=0;i<NB;++i)if(lab[i]>=0)lab[i]=find(lab[i]);
 rep.basins+=par.size();
 // the wounds' removed tissue (blocks with no joined standing cells and some removed tissue) takes the label of the nearest basin
 {std::vector<std::uint32_t> q;for(std::size_t i=0;i<NB;++i)if(lab[i]>=0)q.push_back(std::uint32_t(i));
  for(std::size_t h=0;h<q.size();++h){const std::uint32_t i=q[h];int x,y,z;bx3(i,x,y,z);const int c[3]={x,y,z},lim[3]={BX,BY,BZ};
   for(int k=0;k<6;++k){const int axis=k>>1,sg=(k&1)?1:-1;if(c[axis]+sg<0||c[axis]+sg>=lim[axis])continue;const std::size_t j=sg>0?i+step[axis]:i-step[axis];
    if(lab[j]>=0||stand[j]||!gone[j])continue;lab[j]=lab[i];q.push_back(std::uint32_t(j));}}}
 // contact faces between basins: held = standing and joined across the face; wound = anything else (removed tissue, a thin gap)
 std::unordered_map<std::uint64_t,std::array<std::uint32_t,2>> pairs;
 for(int x=0;x<BX;++x)for(int y=0;y<BY;++y)for(int z=0;z<BZ;++z){const std::size_t i=bi(x,y,z);const std::int32_t L=lab[i];if(L<0)continue;const int c[3]={x,y,z},lim[3]={BX,BY,BZ};
  for(int axis=0;axis<3;++axis){if(c[axis]+1>=lim[axis])continue;const std::size_t j=i+step[axis];const std::int32_t M=lab[j];if(M<0||M==L)continue;
   const std::uint64_t key=(std::uint64_t(std::uint32_t(std::min(L,M)))<<32)|std::uint32_t(std::max(L,M));auto&f=pairs[key];++f[(stand[i]&&stand[j]&&conn(i,j,axis))?0:1];}}
 rep.shedMs+=core::ms(b0);rep.pairs+=pairs.size();
 const double blockVol=std::pow(b*pitch,3.);
 auto substantial=[&](std::int32_t r){return peak[std::size_t(r)]>=peakMin&&(touch[std::size_t(r)]||double(vol[std::size_t(r)])*blockVol>=minPieceM3);};
 struct Cand {std::int32_t A,B;std::uint32_t held,wound;double est;};std::vector<Cand> cands;
 for(const auto&p:pairs){const std::int32_t A=std::int32_t(p.first>>32),B=std::int32_t(p.first&0xFFFFFFFFu);const auto h=p.second[0],w=p.second[1];
  if(!h||!w||!substantial(A)||!substantial(B))continue;const double est=double(h)/double(h+w);if(est>estMax)continue;cands.push_back({A,B,h,w,est});}
 std::sort(cands.begin(),cands.end(),[](const Cand&a,const Cand&c){return a.est<c.est||(a.est==c.est&&(a.A<c.A||(a.A==c.A&&a.B<c.B)));});
 if(cands.size()>std::size_t(maxCandidates))cands.resize(std::size_t(maxCandidates));
 rep.candidates+=cands.size();if(cands.empty()){rep.why="no_neck";return false;}
 if(logOn())for(const auto&cd:cands)std::cerr<<"damagecut_hold candidate basins="<<cd.A<<"/"<<cd.B<<" peak_mm="<<peak[std::size_t(cd.A)]*b*pitch*1e3<<"/"<<peak[std::size_t(cd.B)]*b*pitch*1e3
  <<" cm3="<<double(vol[std::size_t(cd.A)])*blockVol*1e6<<"/"<<double(vol[std::size_t(cd.B)])*blockVol*1e6<<" edge="<<int(touch[std::size_t(cd.A)])<<int(touch[std::size_t(cd.B)])<<" est="<<cd.est<<std::endl;
 const double T=std::clamp(.45-.1*(severScale-1),.25,.6);
 const Cell cell0{tlo[0]*8,tlo[1]*8,tlo[2]*8};
 for(const auto&cd:cands){
  const auto f0=core::Clock::now();++rep.measured;rep.est=cd.est;
  // ---- the band: blocks within bandBlocks of the contact
  std::vector<std::uint8_t> band(NB,0);std::vector<std::uint32_t> q;
  for(int x=0;x<BX;++x)for(int y=0;y<BY;++y)for(int z=0;z<BZ;++z){const std::size_t i=bi(x,y,z);const std::int32_t L=lab[i];if(L!=cd.A&&L!=cd.B)continue;
   const std::int32_t O=L==cd.A?cd.B:cd.A;bool contact=false;const int c[3]={x,y,z},lim[3]={BX,BY,BZ};
   for(int k=0;k<6&&!contact;++k){const int axis=k>>1,sg=(k&1)?1:-1;if(c[axis]+sg<0||c[axis]+sg>=lim[axis])continue;if(lab[sg>0?i+step[axis]:i-step[axis]]==O)contact=true;}
   if(contact){band[i]=1;q.push_back(std::uint32_t(i));}}
  int blo[3]={INT_MAX,INT_MAX,INT_MAX},bhi[3]={INT_MIN,INT_MIN,INT_MIN};
  {std::size_t h=0;for(int ring=0;ring<=bandBlocks;++ring){const std::size_t end=q.size();
    for(;h<end;++h){const std::uint32_t i=q[h];int x,y,z;bx3(i,x,y,z);
     blo[0]=std::min(blo[0],x);blo[1]=std::min(blo[1],y);blo[2]=std::min(blo[2],z);bhi[0]=std::max(bhi[0],x);bhi[1]=std::max(bhi[1],y);bhi[2]=std::max(bhi[2],z);
     if(ring==bandBlocks)continue;const int c[3]={x,y,z},lim[3]={BX,BY,BZ};
     for(int k=0;k<6;++k){const int axis=k>>1,sg=(k&1)?1:-1;if(c[axis]+sg<0||c[axis]+sg>=lim[axis])continue;const std::size_t j=sg>0?i+step[axis]:i-step[axis];if(band[j])continue;band[j]=1;q.push_back(std::uint32_t(j));}}}}
  // ---- the intact connection Hi: a block-level vertex cut over the band's tissue blocks (x b^2 cells; the wounds' tissue standing)
  long long hiBlocks=0;
  {std::vector<std::int32_t> node(NB,-1);std::vector<std::uint32_t> nodes;
   for(const std::uint32_t i:q)if(sm[i]||gone[i]){node[i]=std::int32_t(nodes.size());nodes.push_back(i);}
   const int V=int(nodes.size()),S=2*V,Tn=2*V+1;Flow F;F.init(2*V+2,std::size_t(V)*8);const int INF=1<<29;
   for(int n2=0;n2<V;++n2){const std::uint32_t i=nodes[std::size_t(n2)];int x,y,z;bx3(i,x,y,z);F.add(2*n2,2*n2+1,1);bool src=false,snk=false;const int c[3]={x,y,z},lim[3]={BX,BY,BZ};
    for(int k=0;k<6;++k){const int axis=k>>1,sg=(k&1)?1:-1;if(c[axis]+sg<0||c[axis]+sg>=lim[axis])continue;const std::size_t j=sg>0?i+step[axis]:i-step[axis];
     if(node[j]>=0){F.add(2*n2+1,2*node[j],INF);continue;}if(!(sm[j]||gone[j]))continue;
     if(lab[j]==cd.A&&!src){src=true;F.add(S,2*n2,INF);}else if(lab[j]==cd.B&&!snk){snk=true;F.add(2*n2+1,Tn,INF);}}}
   hiBlocks=V?F.run(S,Tn,(long long)1<<40):0;}
  const long long hiCells=hiBlocks*b*b;
  if(hiCells<=0){rep.flowMs+=core::ms(f0);rep.why="no_intact_link";continue;}
  // the prefilter: the same block cut through the band's blocks that are at least half standing (joined faces) - it can only
  // undercount the standing connection - already past the limit: the neck holds (no cell flow)
  {std::vector<std::int32_t> node(NB,-1);std::vector<std::uint32_t> nodes;const unsigned halfB=unsigned(b3/2);
   auto major=[&](std::size_t i){return stand[i]&&pc(sm[i])>=halfB;};
   for(const std::uint32_t i:q)if(major(i)){node[i]=std::int32_t(nodes.size());nodes.push_back(i);}
   const int V=int(nodes.size()),S=2*V,Tn=2*V+1;Flow F;F.init(2*V+2,std::size_t(V)*8);const int INF=1<<29;
   for(int n2=0;n2<V;++n2){const std::uint32_t i=nodes[std::size_t(n2)];int x,y,z;bx3(i,x,y,z);F.add(2*n2,2*n2+1,1);bool src=false,snk=false;
    for(int k=0;k<6;++k){std::size_t j;if(!linked(i,x,y,z,k,j))continue;
     if(node[j]>=0){F.add(2*n2+1,2*node[j],INF);continue;}if(!major(j))continue;
     if(lab[j]==cd.A&&!src){src=true;F.add(S,2*n2,INF);}else if(lab[j]==cd.B&&!snk){snk=true;F.add(2*n2+1,Tn,INF);}}}
   const double tMaxP=goneOnly?.15:T;const long long lim=(long long)std::ceil(1.1*tMaxP*double(hiCells)/double(b*b))+1;
   const long long hm=V?F.run(S,Tn,lim):0;
   if(hm>=lim){rep.flowMs+=core::ms(f0);rep.holding=hm*b*b;rep.intact=hiCells;rep.why="holds";
    if(logOn())std::cerr<<"damagecut_hold neck basins="<<cd.A<<"/"<<cd.B<<" est="<<cd.est<<" prefilter H>="<<hm*b*b<<" Hi="<<hiCells<<" verdict=holds flow_ms="<<core::ms(f0)<<std::endl;
    continue;}}
  // ---- the band's cells (+ one cell around): bit 0 tissue, 1 removed, 2 bone, 3 in the band, 4 / 5 its block's basin is A / B,
  // 6 its block stands (joined: its standing cells are one component, joined to its basin)
  int clo[3],cn[3];for(int a=0;a<3;++a){clo[a]=std::max(0,(&cell0.x)[a]+blo[a]*b-1);const int chi=std::min((&g.size.x)[a]-1,(&cell0.x)[a]+(bhi[a]+1)*b);cn[a]=chi-clo[a]+1;}
  const std::size_t NC=std::size_t(cn[0])*cn[1]*cn[2];if(NC>6*maxBand){rep.flowMs+=core::ms(f0);rep.why="band_box";continue;}
  auto ci=[&](int x,int y,int z){return (std::size_t(x-clo[0])*std::size_t(cn[1])+std::size_t(y-clo[1]))*std::size_t(cn[2])+std::size_t(z-clo[2]);};
  std::vector<std::uint8_t> fl(NC,0);
  {const int t0[3]={clo[0]>>3,clo[1]>>3,clo[2]>>3},t1[3]={(clo[0]+cn[0]-1)>>3,(clo[1]+cn[1]-1)>>3,(clo[2]+cn[2]-1)>>3};
   core::parallelChunks(std::size_t(t1[0]-t0[0]+1),1,[&](std::size_t a0,std::size_t a1,std::size_t){TileBits tb;
    for(int tx=t0[0]+int(a0);tx<t0[0]+int(a1);++tx)for(int ty=t0[1];ty<=t1[1];++ty)for(int tz=t0[2];tz<=t1[2];++tz){tileBits(g,state,Cell{tx,ty,tz},tb);
     for(unsigned l=0;l<512;++l){const int x=tx*8+int(l>>6),y=ty*8+int((l>>3)&7u),z=tz*8+int(l&7u);
      if(x<clo[0]||y<clo[1]||z<clo[2]||x>=clo[0]+cn[0]||y>=clo[1]+cn[1]||z>=clo[2]+cn[2])continue;const std::uint64_t bit=std::uint64_t(1)<<(l&63u);
      std::uint8_t f=0;if(tb.solid[l>>6]&bit)f|=1;if(tb.removed[l>>6]&bit)f|=2;if(tb.bone[l>>6]&bit)f|=4;
      const int bx=(x-cell0.x)/b,by=(y-cell0.y)/b,bz=(z-cell0.z)/b;
      if(x>=cell0.x&&y>=cell0.y&&z>=cell0.z&&bx<BX&&by<BY&&bz<BZ){const std::size_t j=bi(bx,by,bz);if(band[j])f|=8;const std::int32_t L=lab[j];if(L==cd.A)f|=16;else if(L==cd.B)f|=32;if(stand[j])f|=64;}
      fl[ci(x,y,z)]=f;}}});}
  // ---- a cell-level vertex cut between the band's two faces; bone: only bone cells; intact: removed cells count as standing
  struct Cut {long long flow=-1;bool capped=false;std::vector<Cell> cells,aSide,bSide;Vec aSum{0,0,0},bSum{0,0,0};std::size_t nodes=0;};
  auto cutOf=[&](bool boneOnly,bool intact,long long limit,bool wantCells)->Cut{Cut out;
   auto pass=[&](std::uint8_t f){return (f&1)&&(!boneOnly||(f&4))&&(intact||!(f&2));};
   auto face=[&](std::uint8_t f,std::uint8_t side){return (f&side)&&(intact||(f&64));}; // an outside cell of a basin (standing in a joined block)
   std::vector<std::int32_t> node(NC,-1);std::vector<Cell> cellOfNode;
   for(int x=clo[0];x<clo[0]+cn[0];++x)for(int y=clo[1];y<clo[1]+cn[1];++y)for(int z=clo[2];z<clo[2]+cn[2];++z){const std::size_t k=ci(x,y,z);const auto f=fl[k];
    if((f&8)&&pass(f)){node[k]=std::int32_t(cellOfNode.size());cellOfNode.push_back(Cell{x,y,z});}}
   out.nodes=cellOfNode.size();if(cellOfNode.size()>maxBand){out.flow=limit;out.capped=true;return out;}
   const int V=int(cellOfNode.size()),S=2*V,Tn=2*V+1;Flow F;F.init(2*V+2,std::size_t(V)*8);const int INF=1<<29;
   for(int i=0;i<V;++i){const Cell c=cellOfNode[std::size_t(i)];F.add(2*i,2*i+1,1);bool src=false,snk=false;
    const int nb[6][3]={{c.x-1,c.y,c.z},{c.x+1,c.y,c.z},{c.x,c.y-1,c.z},{c.x,c.y+1,c.z},{c.x,c.y,c.z-1},{c.x,c.y,c.z+1}};
    for(const auto&e:nb){if(e[0]<clo[0]||e[1]<clo[1]||e[2]<clo[2]||e[0]>=clo[0]+cn[0]||e[1]>=clo[1]+cn[1]||e[2]>=clo[2]+cn[2])continue;const std::size_t k=ci(e[0],e[1],e[2]);
     if(node[k]>=0){F.add(2*i+1,2*node[k],INF);continue;}
     const auto f=fl[k];if((f&8)||!pass(f))continue;
     if(face(f,16)&&!src){src=true;F.add(S,2*i,INF);}else if(face(f,32)&&!snk){snk=true;F.add(2*i+1,Tn,INF);}}}
   out.flow=V?F.run(S,Tn,limit):0;out.capped=out.flow>=limit;
   if(!out.capped&&wantCells&&out.flow>0){const auto r=F.reach(S);
    for(int i=0;i<V;++i)if(r[std::size_t(2*i)]&&!r[std::size_t(2*i+1)])out.cells.push_back(cellOfNode[std::size_t(i)]);
    // the cut's two sides next to it: the proof's seeds and the cut's direction across
    const std::set<Cell> hs(out.cells.begin(),out.cells.end());std::set<Cell> seenA,seenB;
    for(const Cell&c:out.cells){const int nb[6][3]={{c.x-1,c.y,c.z},{c.x+1,c.y,c.z},{c.x,c.y-1,c.z},{c.x,c.y+1,c.z},{c.x,c.y,c.z-1},{c.x,c.y,c.z+1}};
     for(const auto&e:nb){if(e[0]<clo[0]||e[1]<clo[1]||e[2]<clo[2]||e[0]>=clo[0]+cn[0]||e[1]>=clo[1]+cn[1]||e[2]>=clo[2]+cn[2])continue;const std::size_t k=ci(e[0],e[1],e[2]);
      const std::int32_t j=node[k];const Cell ce{e[0],e[1],e[2]};if(j<0||hs.count(ce))continue;const Vec p=g.center(ce);
      if(r[std::size_t(2*j+1)]){if(seenA.insert(ce).second){out.aSide.push_back(ce);out.aSum=out.aSum+p;}}else if(seenB.insert(ce).second){out.bSide.push_back(ce);out.bSum=out.bSum+p;}}}}
   return out;};
  const double tMax=goneOnly?.15:T;
  Cut H=cutOf(false,false,(long long)std::ceil(tMax*double(hiCells))+1,true);rep.bandCells=std::max(rep.bandCells,H.nodes);
  if(H.capped||H.flow<=0||H.cells.empty()){rep.flowMs+=core::ms(f0);rep.holding=H.flow;rep.intact=hiCells;rep.why=H.flow==0?"no_standing_link_in_band":"holds";
   if(logOn())std::cerr<<"damagecut_hold neck basins="<<cd.A<<"/"<<cd.B<<" est="<<cd.est<<" band_cells="<<H.nodes<<" H="<<H.flow<<(H.capped?"+":"")<<" Hi="<<hiCells<<" verdict="<<rep.why<<" flow_ms="<<core::ms(f0)<<std::endl;
   continue;}
  const Cut Bd=cutOf(true,false,(long long)1<<40,false),Bi=cutOf(true,true,(long long)1<<40,false);
  rep.flowMs+=core::ms(f0);rep.holding=H.flow;rep.intact=hiCells;rep.hb=Bd.flow;rep.hib=Bi.flow;
  const double t=double(H.flow)/double(hiCells),bl=Bi.flow>0?std::min(1.,double(std::max<long long>(0,Bd.flow))/double(Bi.flow)):t;rep.t=t;rep.b=bl;
  const bool tear=goneOnly?(t<=.15&&bl<=.4):(t<=T&&bl<=.4),dang=false;
  if(logOn())std::cerr<<"damagecut_hold neck basins="<<cd.A<<"/"<<cd.B<<" est="<<cd.est<<" held_faces="<<cd.held<<" wound_faces="<<cd.wound<<" band_cells="<<H.nodes
   <<" H="<<H.flow<<" Hi="<<hiCells<<" bone="<<Bd.flow<<"/"<<Bi.flow<<" t="<<t<<" b="<<bl<<" T="<<T<<" gone_only="<<goneOnly
   <<" verdict="<<(tear?"tear":(dang?"dangling":"holds"))<<" flow_ms="<<core::ms(f0)<<std::endl;
  if(!(tear||dang)){rep.why="holds";continue;}
  // ---- the cut (generalTear: the parting surface's sheet or H, after its proof)
  const auto c0=core::Clock::now();
  Vec C{0,0,0};for(const Cell&c:H.cells)C=C+g.center(c);C=C*(1./double(H.cells.size()));
  const Vec nAB=(!H.aSide.empty()&&!H.bSide.empty())?H.bSum*(1./double(H.bSide.size()))-H.aSum*(1./double(H.aSide.size())):Vec{0,0,1};
  const SeverSegment*nearSeg=nullptr;double best=1e30;
  for(const auto&s:segments){const double tt=std::clamp(dot(C-s.pivot,s.axis),0.,s.length);const double dd=length(C-(s.pivot+s.axis*tt))-s.radius;if(dd<best){best=dd;nearSeg=&s;}}
  std::vector<Cell> bandStanding;
  for(int x=clo[0];x<clo[0]+cn[0];++x)for(int y=clo[1];y<clo[1]+cn[1];++y)for(int z=clo[2];z<clo[2]+cn[2];++z){const auto f=fl[ci(x,y,z)];if((f&8)&&(f&1)&&!(f&2))bandStanding.push_back(Cell{x,y,z});}
  DamageTearVerdict w=v; // the verdict changes only when it cut
  w.bone=nearSeg?nearSeg->bone:std::uint16_t(0);w.remainingTissue=t;w.remainingBone=bl;w.dangling=dang;w.probeBone=w.bone;w.probeTissue=t;w.probeBoneLeft=bl;
  w.holding=std::size_t(H.flow);w.holdingIntact=std::size_t(hiCells);w.holdingBone=std::size_t(std::max<long long>(0,Bd.flow));w.holdingBoneIntact=std::size_t(std::max<long long>(0,Bi.flow));
  const bool done=generalTear(g,state,closure,H.cells,H.aSide,H.bSide,bandStanding,nAB,nearSeg?nearSeg->axis:nAB,seed64,impact,segments,w,rep);
  rep.tearMs+=core::ms(c0);if(!done)continue;v=std::move(w);rep.why="cut";return v.tear;}
 return false;}

// damageTear's call when no station failed (value 3; not a forced tear, not an exact blade sheet): the clusters of the dirty tiles
inline bool sever(Grid&g,const std::vector<SeverSegment>&segments,SeverState&state,const std::set<Cell>&dirty,double severScale,
    const std::vector<const ResolvedTunnel*>&cuts,const TearGuide*guide,const painter_detail::TissueField*closure,const Vec*impact,
    DamageTearVerdict&v,const core::Clock::time_point&t0){
 if(dirty.empty())return false;const auto h0=core::Clock::now();const double pitch=g.pitch;Report rep;
 const int TS[3]={(g.size.x+7)/8,(g.size.y+7)/8,(g.size.z+7)/8};
 const int maxT=std::max(4,int(std::ceil(maxSideM/(8*pitch)))),padT=std::max(1,int(std::ceil(padM/(8*pitch))));
 std::uint64_t seed=0x7EA5A11CEull;for(const ResolvedTunnel*r:cuts)if(r)seed=core::quantMix(seed^r->edge_seed^(r->sequence*0x9E3779B97F4A7C15ull));seed^=0x401D0C07ull;
 // clusters of the dirty tiles (26-connected)
 std::vector<Cell> dt;dt.reserve(dirty.size());for(const Cell&t:dirty)if(t.x>=0&&t.y>=0&&t.z>=0&&t.x<TS[0]&&t.y<TS[1]&&t.z<TS[2])dt.push_back(t);
 std::vector<std::int32_t> up(dt.size());for(std::size_t i=0;i<dt.size();++i)up[i]=std::int32_t(i);
 auto root=[&](std::int32_t a){while(up[std::size_t(a)]!=a){up[std::size_t(a)]=up[std::size_t(up[std::size_t(a)])];a=up[std::size_t(a)];}return a;};
 for(std::size_t i=0;i<dt.size();++i)for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)for(int dz=-1;dz<=1;++dz){const Cell k{dt[i].x+dx,dt[i].y+dy,dt[i].z+dz};
  const auto it=std::lower_bound(dt.begin(),dt.end(),k);if(it==dt.end()||!(*it==k))continue;const std::int32_t a=root(std::int32_t(i)),c=root(std::int32_t(it-dt.begin()));if(a!=c)up[std::size_t(std::max(a,c))]=std::min(a,c);}
 std::unordered_map<std::int32_t,std::vector<Cell>> groups;for(std::size_t i=0;i<dt.size();++i)groups[root(std::int32_t(i))].push_back(dt[i]);
 std::vector<std::vector<Cell>> clusters;for(auto&gp:groups)clusters.push_back(std::move(gp.second));
 std::sort(clusters.begin(),clusters.end(),[](const std::vector<Cell>&a,const std::vector<Cell>&c){return a.size()>c.size()||(a.size()==c.size()&&a.front()<c.front());});
 if(clusters.size()>4)clusters.resize(4);
 bool cut=false;
 for(const auto&cl:clusters){++rep.clusters;
  int lo[3]={INT_MAX,INT_MAX,INT_MAX},hi[3]={INT_MIN,INT_MIN,INT_MIN};
  for(const Cell&t:cl)for(int a=0;a<3;++a){lo[a]=std::min(lo[a],t[a]);hi[a]=std::max(hi[a],t[a]);}
  for(int a=0;a<3;++a)if(hi[a]-lo[a]+1>maxT){const int mid=(lo[a]+hi[a])/2;lo[a]=mid-maxT/2;hi[a]=lo[a]+maxT-1;}
  // grown through the damaged tiles joined to the cluster (the box stays within maxT a side)
  {std::set<Cell> seen(cl.begin(),cl.end());std::vector<Cell> st(cl.begin(),cl.end());std::size_t steps=0;
   while(!st.empty()&&steps<200000){const Cell t=st.back();st.pop_back();++steps;
    for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)for(int dz=-1;dz<=1;++dz){const Cell k{t.x+dx,t.y+dy,t.z+dz};
     if(k.x<0||k.y<0||k.z<0||k.x>=TS[0]||k.y>=TS[1]||k.z>=TS[2]||seen.count(k))continue;
     bool inBox=true;for(int a=0;a<3;++a)inBox=inBox&&std::max(hi[a],k[a])-std::min(lo[a],k[a])+1<=maxT;if(!inBox)continue;
     const core::Tile*tile=g.findTile(k);bool dmg=false;if(tile)for(const auto wd:tile->removed)dmg=dmg||wd!=0;if(!dmg)continue;
     seen.insert(k);st.push_back(k);for(int a=0;a<3;++a){lo[a]=std::min(lo[a],k[a]);hi[a]=std::max(hi[a],k[a]);}}}}
  for(int a=0;a<3;++a){lo[a]=std::max(0,lo[a]-padT);hi[a]=std::min(TS[a]-1,hi[a]+padT);}
  if(cutIn(g,state,lo,hi,severScale,guide&&guide->flesh_gone_only,closure,impact,segments,seed,v,rep)){cut=true;break;}}
 rep.ms=core::ms(h0);v.holdMs=rep.ms;
 if(logOn())std::cerr<<"damagecut_hold call clusters="<<rep.clusters<<" tiles="<<rep.tiles<<" blocks="<<rep.blocks<<" basins="<<rep.basins<<" pairs="<<rep.pairs
  <<" candidates="<<rep.candidates<<" measured="<<rep.measured<<" band_cells="<<rep.bandCells<<" est="<<rep.est<<" H="<<rep.holding<<" Hi="<<rep.intact<<" t="<<rep.t<<" b="<<rep.b
  <<" small_cells="<<rep.smallCells<<" why="<<rep.why<<" block_ms="<<rep.blockMs<<" edt_ms="<<rep.edtMs<<" shed_ms="<<rep.shedMs<<" flow_ms="<<rep.flowMs
  <<" flood_ms="<<rep.floodMs<<" tear_ms="<<rep.tearMs<<" ms="<<rep.ms<<std::endl;
 if(cut){v.ms=core::ms(t0);GFY_DAMAGE_TEAR_REPORT(v);}
 return cut;}
} // namespace holding_cut
