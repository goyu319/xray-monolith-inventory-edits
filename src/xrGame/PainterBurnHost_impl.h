// Included once in Level.cpp. All native world and animation reads stay on the game
// thread. The heat worker only uses snapshots, copied matrices and value records.
#include "PainterBurnHost.h"
#include "painter_voxel/PainterPieceContinuation.h"
#include "../xrCore/PainterBurnTrace.h"
#include "PainterBurnReceiverGuard.h"
#include "../xrCore/PainterBurnTrace.h"
extern bool PainterPrimeBurnGeometry(u16,u64);
#include "PainterBurnSourceEvent.h"
#include "../Include/xrRender/PainterBurnRender.h"
#include "../Include/xrRender/PainterBurnSessions.h"
#include "../Layers/xrRender/PainterBurnRoles.h"

#include "../Include/xrRender/Kinematics.h"
#include "ai/stalker/ai_stalker.h"
#include "CharacterPhysicsSupport.h"
#include <atomic>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <deque>
#include <map>
#include <tuple>
#include "PainterBurnSampling.h"
#pragma push_macro("near")
#pragma push_macro("far")
#undef near
#undef far
#include "PainterBurnGrowth.h"
namespace painter_burn_host {
using gfy_live::Point;using gfy_live::Matrix;using painter_burn::OwnerKey;
using painter_burn_source::Event;using painter_burn_source::Shape;
inline void traceReceiver(const Event& e,unsigned id,const char* result,unsigned count=0){
 if(PainterBurnTraceEnabled())Msg("[painter_burn_trace] stage=receiver source=%s serial=%llu pulse=%llu emitter=%u body=%u result=%s count=%u",painter_burn_source::causeName(e.cause),e.sourceSerial,e.pulse,unsigned(e.emitterId),id,result,count);
}
// A line in the log whenever what happens to a body in a fire changes (never a line a tick). Two callers: the game
// thread (who = 0) and the heat worker (who = 1), each with its own table.
inline void noteBody(unsigned who,unsigned id,unsigned state,const char* text,size_t a=0,size_t b=0){
 static std::mutex guard;static std::map<unsigned,unsigned> last[2];
 std::lock_guard<std::mutex> lk(guard);auto& t=last[who&1u];
 if(t.size()>512)t.clear();
 auto it=t.find(id);if(it!=t.end()&&it->second==state)return;
 t[id]=state;Msg("[painter_burn] body %u in a fire: %s (%u, %u)",id,text,unsigned(a),unsigned(b));
}
inline Point from(Fvector p){return {p.x,p.y,p.z};}
inline Point add(Point a,Point b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
inline Point sub(Point a,Point b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
inline Point scale(Point a,double f){return {a.x*f,a.y*f,a.z*f};}
inline double dot(Point a,Point b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline Point cross(Point a,Point b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline double length(Point p){return std::sqrt(dot(p,p));}
inline Point norm(Point p){auto n=length(p);return n>1e-9?scale(p,1./n):Point{};}
inline Matrix matrix(const Fmatrix& m){Matrix a;a.row_major={m._11,m._21,m._31,m._41,m._12,m._22,m._32,m._42,m._13,m._23,m._33,m._43,m._14,m._24,m._34,m._44};return a;}
static std::shared_ptr<const painter_fire::Rig> captureFireRig(IKinematics& k){
 static const char* const names[]={"bip01_l_upperarm","bip01_l_forearm","bip01_l_hand",
   "bip01_r_upperarm","bip01_r_forearm","bip01_r_hand","bip01_l_thigh","bip01_l_calf","bip01_l_foot",
   "bip01_r_thigh","bip01_r_calf","bip01_r_foot"};
 auto out=std::make_shared<painter_fire::Rig>();
 for(unsigned i=0;i<12;i++){const auto bone=k.LL_BoneID(names[i]);if(bone>=k.LL_BoneCount())continue;
   Fmatrix bind;bind.invert(k.LL_GetData(bone).m2b_transform);out->joints[i]={bind.c.x,bind.c.y,bind.c.z};out->valid|=1u<<i;}
 return out;
}
struct Binding {std::shared_ptr<const painter_fire::Rig> fireRig;
 u16 id{};u64 birth{},world{},lineage{};OwnerKey key{};bool birthPosted=false,facesPending=false,facesFailed=false;u64 capturedGeometry{};
 std::weak_ptr<gfy_live::Session> worker;
 std::shared_ptr<const painter_burn::SourceFaces> faces,exterior;
 std::shared_ptr<const gfy_live::DamageSnapshot> last;
};
struct Body {painter_fire::Pose firePose;Binding binding;Point centre;double radius;std::vector<Matrix> bones;};
struct Triangle {Point a,b,c;};
struct Exposure {bool limitedWorld{};Fvector worldLo{},worldHi{};u64 attachedBirth{};u32 queuedAt{};Event e;std::vector<Body> bodies;std::vector<Triangle> world;};
using BurnCandidate=painter_burn_growth::Candidate;
struct Footprint {
 std::shared_ptr<const painter_fire::Shape> spreadShape;u64 spreadRevision=~u64(0),spreadGeneration=~u64(0);
 u16 sourceBody=0xffff;u64 sourceBirth{};
 u32 areaTick{},lastTick{};u64 tracedGeneration{},confirmedGeneration=~u64(0),confirmedRevision=~u64(0);painter_burn_growth::Sites sites;
 struct Contact {BurnCandidate direction; painter_burn::ContactCell cell;};
 std::vector<Contact> halo;
};
struct Handle {std::shared_ptr<painter_burn::SourceFaces> faces;std::map<u64,Footprint> footprints;};
inline std::shared_ptr<painter_burn::SourceFaces> wallMap(const std::shared_ptr<const painter_burn::SourceFaces>& original){
 auto t=std::make_shared<painter_burn::SourceFaces>();t->authored=original;return t;
}
struct WallUpdate {u16 id;u64 birth,world;OwnerKey key;u64 revision;std::shared_ptr<const painter_burn::SourceFaces> faces;};
struct Worker {
 std::mutex mutex;
 std::condition_variable wake;std::deque<std::function<void()>> jobs;
 std::deque<WallUpdate> wallUpdates,primes,faceReady;
 u64 sequence{};
 bool stopping=false;unsigned waitingHeat{};
 std::thread thread;
 painter_burn::Sessions sessions;
 std::map<std::pair<u64,u64>,Handle> handles;
 std::map<std::pair<u64,u64>,std::pair<u64,u64>> primed;
 std::map<u64,unsigned> nextWall;
 std::deque<std::shared_ptr<const painter_burn::Delta>> upload;
 Worker(){thread=std::thread([this]{run();});}
 void run(){
   SetThreadPriority(GetCurrentThread(),THREAD_PRIORITY_BELOW_NORMAL);
   for(;;){std::function<void()> task;
     {std::unique_lock<std::mutex> lk(mutex);if(upload.empty())wake.wait(lk,[&]{return stopping||!jobs.empty();});else wake.wait_for(lk,std::chrono::milliseconds(15),[&]{return stopping||!jobs.empty();});
      if(stopping)return;if(!jobs.empty()){task=std::move(jobs.front());jobs.pop_front();}}
     try{if(task)task();
       if(!(PainterBodyMasterStamp()&1u))continue;
       while(auto d=sessions.tryPrepare())upload.push_back(std::move(d));
       while(!upload.empty()&&::Render&&::Render->PainterBurnPublish(upload.front()))upload.pop_front();
     }catch(const std::exception& e){Msg("[painter_burn] worker: %s",e.what());}
   }
 }
 void post(std::function<void()> f){std::lock_guard<std::mutex> lk(mutex);if(!stopping){jobs.push_back(std::move(f));wake.notify_one();}}
 void stop(){ {std::lock_guard<std::mutex> lk(mutex);stopping=true;wake.notify_one();} if(thread.joinable())thread.join(); }
};
struct GroundWitness {Event source;std::vector<std::pair<u16,u64>> tried;};
struct ConfirmedTick {u64 birth{},world{};u32 tick{};};
struct Main {std::map<u16,ConfirmedTick> confirmed;std::map<u64,GroundWitness> ground;std::mutex mutex;std::map<u16,Binding> bodies;std::deque<Event> events;std::unique_ptr<Worker> work;u64 epoch{};std::deque<std::vector<Exposure>> deferred;bool stopped=false;};
static std::atomic<Main*> activeHost{nullptr};
static std::atomic<bool> hostStopped{false};
bool active() noexcept{return (PainterBodyMasterStamp()&1u) && activeHost.load(std::memory_order_acquire)!=nullptr;}
inline Main& mainState(){
 static auto* m=[]{auto* host=new Main;activeHost.store(host,std::memory_order_release);return host;}();
 return *m;
}
inline Worker& worker(Main& m){if(!m.work)m.work.reset(new Worker);return *m.work;}
inline std::shared_ptr<painter_burn::SourceFaces> faceTable(const std::vector<unsigned char>& roles,const std::shared_ptr<gfy_live::Session>& s){
 auto source=s->burn_faces();if(!source)return {};
 auto table=std::make_shared<painter_burn::SourceFaces>(*source);
 for(size_t child=0;child<table->byChild.size()&&child<roles.size();++child)
  for(auto token:table->byChild[child])if(token&&token<=table->roles.size())table->roles[token-1]=roles[child];
 return table;
}
void bind(CObject& o,const gfy_live::Identity& id,const std::shared_ptr<gfy_live::Session>& session,bool piece,bool exposure) noexcept {try{
 if(!(PainterBodyMasterStamp()&1u) || (!exposure&&(!piece||!active())))return;
 auto* k=smart_cast<IKinematics*>(o.Visual());if(!session||!k)return;
 auto& m=mainState();std::lock_guard<std::mutex> lk(m.mutex);if(m.stopped)return;
 auto orig=m.bodies.find(o.ID());if(orig!=m.bodies.end()&&orig->second.worker.lock()==session)return;
 // Normal split activation only inherits an already tracked family. A
 // pre-existing, cold part may enroll independently on its first exposure.
 if(!exposure&&std::none_of(m.bodies.begin(),m.bodies.end(),[&](const auto& item){return item.second.lineage==id.visual_lineage;}))return;
 Binding b;b.id=o.ID();b.birth=o.PainterBirthCookie();b.world=Level().PainterWorldCookie();b.lineage=id.visual_lineage;
 b.key={id.spawn_cookie,orig!=m.bodies.end()&&orig->second.key.spawn==id.spawn_cookie?orig->second.key.generation+1:1};
 b.fireRig=captureFireRig(*k);b.worker=session;if(piece)b.last=session->capture_damage();
 if(piece && orig!=m.bodies.end() && orig->second.birth==b.birth && orig->second.world==b.world &&
    orig->second.lineage==b.lineage && orig->second.key.spawn==b.key.spawn && orig->second.faces){
     b.exterior=orig->second.exterior;b.faces=b.exterior;
 }
 b.birthPosted=!piece||exposure;
 m.bodies[b.id]=b;
 if(!piece||exposure){auto& w=worker(m);w.post([&w,b]{w.sessions.birth(b.lineage,b.key);});}
 }catch(...){Msg("[painter_burn] receiver bind unavailable");}}
void forget(CObject& o,const std::shared_ptr<gfy_live::Session>& s) noexcept {try{
 if(!active())return;
 auto& m=mainState();std::lock_guard<std::mutex> lk(m.mutex);auto i=m.bodies.find(o.ID());
 if(i==m.bodies.end()||i->second.worker.lock()!=s||i->second.birth!=o.PainterBirthCookie())return;
 auto b=i->second;m.bodies.erase(i);if(b.faces)if(auto* k=smart_cast<IKinematics*>(o.Visual()))k->LL_PainterBurnOwner(0,0,0,{});
 if(m.work){auto& w=*m.work;w.post([&w,b]{w.sessions.retire(b.lineage,b.key);w.handles.erase({b.lineage,b.key.spawn});w.primed.erase({b.lineage,b.key.spawn});auto next=w.handles.lower_bound({b.lineage,0});if(next==w.handles.end()||next->first.first!=b.lineage)w.nextWall.erase(b.lineage);});}
 }catch(...){}}
void split(CObject& stump,const std::vector<CObject*>& children) noexcept {try{
 auto& m=mainState();std::lock_guard<std::mutex> lk(m.mutex);auto i=m.bodies.find(stump.ID());if(i==m.bodies.end()||i->second.key.generation<2)return;
 const auto root=i->second;auto prior=root.key;--prior.generation;
 std::vector<Binding> parts{root};for(auto* o:children){auto j=m.bodies.find(o->ID());if(j!=m.bodies.end()&&j->second.lineage==root.lineage)parts.push_back(j->second);}
 if(root.faces && root.exterior){
     // Both owner lists refer to the same original mesh. Each destination already
     // has its own committed wound geometry. Carry exterior state only.
     for(auto& part:parts){
        CObject* o=part.id==stump.ID()?&stump:Level().Objects.net_Find(part.id);
        if(!o || o->getDestroy() || o->PainterBirthCookie()!=part.birth || part.world!=root.world)continue;
        if(auto* k=smart_cast<IKinematics*>(o->Visual())){
            auto& live=m.bodies.at(part.id);
            live.exterior=root.exterior;live.faces=root.exterior;part.exterior=root.exterior;part.faces=root.exterior;
            k->LL_PainterBurnSplitOwner(root.lineage,part.key.spawn,part.key.generation,prior.spawn,prior.generation,root.exterior);
        }
     }
 }
 auto& w=worker(m);w.post([&w,prior,parts,root]{
   std::vector<OwnerKey> keys;for(const auto& b:parts)keys.push_back(b.key);
   auto owns=[&](OwnerKey own,const painter_burn::ContactCell& c,const std::array<double,3>& p){
     for(const auto& b:parts)if(b.key==own&&b.last)return b.last->owns_burn(c.surface,c.domain,{p[0],p[1],p[2]});return false;};
   auto result=w.sessions.split(root.lineage,prior,keys.data(),keys.size(),owns);
   if(result!=painter_burn::Status::Applied){w.sessions.retire(root.lineage,prior);for(auto k:keys)w.sessions.birth(root.lineage,k);}
   auto previous=w.handles.find({root.lineage,prior.spawn});
   if(previous!=w.handles.end()){
    const auto old=previous->second;for(const auto& b:parts){Handle next;
      if(!old.faces)continue;
      next.faces=wallMap(old.faces->authored);
      if(b.last)for(const auto& source:old.footprints){
        const auto& original=source.second;
        Footprint carried;carried.sourceBody=original.sourceBody;carried.sourceBirth=original.sourceBirth;
        carried.areaTick=original.areaTick;carried.lastTick=original.lastTick;
        carried.sites=original.sites.retainedBy(b.key.spawn,[&](const BurnCandidate& a){
          return a.domain==painter_burn::Exterior&&b.last->owns_burn(a.surface,a.domain,{a.rest[0],a.rest[1],a.rest[2]});
        });
        if(!carried.sites.count())continue;
        for(const auto& h:original.halo){
          const auto& c=h.cell;const auto p=painter_burn::witnessPoint(c);
          if(c.domain==painter_burn::Exterior&&b.last->owns_burn(c.surface,c.domain,{p[0],p[1],p[2]}))carried.halo.push_back(h);
        }
        if(!carried.halo.empty())next.footprints.emplace(source.first,std::move(carried));
      }
      for(auto& wall:old.faces->walls){Point p{wall.rest[0],wall.rest[1],wall.rest[2]};
        if(b.last&&b.last->owns_burn(wall.token,wall.domain,p))next.faces->walls.push_back(wall);}
      w.handles[{b.lineage,b.key.spawn}]=next;
      std::lock_guard<std::mutex> lock(w.mutex);
      w.wallUpdates.push_back({b.id,b.birth,b.world,b.key,b.last?b.last->stamp().revision:0,next.faces});
    }}
 });}catch(...){Msg("[painter_burn] split registration failed");}}
inline double blocker(Point start,Point d,const Triangle& t,double maxRange){
 auto e=sub(t.b,t.a),f=sub(t.c,t.a),h=cross(d,f);double det=dot(e,h);if(std::abs(det)<1.e-12)return maxRange;
 auto s=sub(start,t.a),q=cross(s,e);double u=dot(s,h)/det,v=dot(q,d)/det,z=dot(f,q)/det;
 return u>=0&&v>=0&&u+v<=1&&z>.0001&&z<maxRange?z:maxRange;
}
// Static occluders are copied once on the game thread and indexed only on the heat worker.
class WorldOcclusion {
 struct Node {Point low,high;unsigned first{},end{},left{},right{};};
 const std::vector<Triangle>& triangles;
 std::vector<unsigned> order;
 std::vector<Node> nodes;
 static double coord(Point p,unsigned a){return a==0?p.x:a==1?p.y:p.z;}
 static double centre(const Triangle& t,unsigned a){return coord(t.a,a)+coord(t.b,a)+coord(t.c,a);}
 unsigned build(unsigned first,unsigned end){
   Node n;n.first=first;n.end=end;n.low={1e30,1e30,1e30};n.high={-1e30,-1e30,-1e30};
   for(unsigned j=first;j<end;++j){auto& t=triangles[order[j]];
     for(auto p:{t.a,t.b,t.c}){n.low={std::min(n.low.x,p.x),std::min(n.low.y,p.y),std::min(n.low.z,p.z)};
       n.high={std::max(n.high.x,p.x),std::max(n.high.y,p.y),std::max(n.high.z,p.z)};}}
   const unsigned id=unsigned(nodes.size());nodes.push_back(n);
   if(end-first>12){auto size=sub(n.high,n.low);unsigned a=size.y>size.x?1:0;if(size.z>coord(size,a))a=2;
     unsigned mid=first+(end-first)/2;std::nth_element(order.begin()+first,order.begin()+mid,order.begin()+end,
       [&](unsigned u,unsigned v){return centre(triangles[u],a)<centre(triangles[v],a);});
     unsigned l=build(first,mid),r=build(mid,end);nodes[id].left=l;nodes[id].right=r;}
   return id;
 }
 void visit(unsigned id,Point o,Point d,double& reach)const{
   const auto& n=nodes[id];double entry=0,exit=reach;
   for(unsigned a=0;a<3;++a){auto origin=coord(o,a),dir=coord(d,a),lo=coord(n.low,a),hi=coord(n.high,a);
     if(std::abs(dir)<1e-12){if(origin<lo||origin>hi)return;continue;}
     auto near=(lo-origin)/dir,far=(hi-origin)/dir;if(near>far)std::swap(near,far);
     entry=std::max(entry,near);exit=std::min(exit,far);if(entry>exit)return;}
   if(n.left){visit(n.left,o,d,reach);visit(n.right,o,d,reach);}
   else for(unsigned i=n.first;i<n.end;++i)reach=blocker(o,d,triangles[order[i]],reach);
 }
public:
 explicit WorldOcclusion(const std::vector<Triangle>& in):triangles(in){
   order.reserve(in.size());for(unsigned i=0;i<in.size();++i)order.push_back(i);
   if(!order.empty())build(0,unsigned(order.size()));
 }
 double nearest(Point o,Point d,double reach)const{if(!nodes.empty())visit(0,o,d,reach);return reach;}
};
inline double powerAt(const Event& e,Point pos,double distance){
 if(e.shape==Shape::LocalBone)return e.power;
 if(e.shape==Shape::Segment)return distance<=e.reach?e.power*std::max(0.,1.-distance/e.reach):0.;
 double normalized=distance/std::max(.0001,double(e.radius));
 if(e.shape==Shape::Box){Point q=sub(pos,from(e.centre));normalized=0;
   for(int j=0;j<3;++j)normalized=std::max(normalized,std::abs(dot(q,from(e.axis[j])))/std::max(.0001,double((&e.half.x)[j])));}
 const double falloff=std::max(0.,1.-e.attenuation*normalized*normalized);
 return normalized<=1?e.power*(e.falloffPower==2?falloff*falloff:falloff):0.;
}
inline std::array<float,3> a3(Point p){return {float(p.x),float(p.y),float(p.z)};}
void apply(Worker& w,const std::vector<Exposure>& sources) {
 if(!(PainterBodyMasterStamp()&1u))return;
 using Index=std::pair<u64,u64>;
 using CellIndex=std::tuple<int,int,int,unsigned,unsigned>;
 struct Taken {Binding b;std::map<CellIndex,painter_burn::ContactCell> cells;bool newWalls=false,publishedFront=false;};
 std::map<Index,Taken> hits;
 auto captureFront=[&](const Binding& b,const painter_burn_growth::Sites& sites,bool confirmed=false){
   std::array<painter_burn::RecessionFront,painter_burn::Recession::Limit> fronts{};unsigned count=0;
   for(size_t i=0;i<sites.count();++i)if(sites.current(i)&&(sites.anchor(i).domain==painter_burn::Exterior||confirmed))
     fronts[count++]=painter_burn::Recession::contact(sites.anchor(i).rest,sites.radius(i),sites.activeTicks(i));
   if(!count)return;
   const auto status=w.sessions.commit(b.lineage,b.key,[&](painter_burn::Family& family){return family.expose(b.key,fronts.data(),count);});
   if(status!=painter_burn::Status::Applied)return;
   auto& h=w.handles[{b.lineage,b.key.spawn}];if(!h.faces)h.faces=wallMap(b.exterior);
   auto& hit=hits[{b.lineage,b.key.spawn}];hit.b=b;hit.publishedFront=true;
 };
 for(const auto& src:sources){
   struct Target {const Body* body;std::shared_ptr<const gfy_live::BurnTrace> trace;};
   const Event& e=src.e;std::vector<Target> targets;
   if(e.confirmedBody){
     for(const auto& body:src.bodies){
       const auto& b=body.binding;
       if(!b.last||b.id!=e.targetId||b.birth!=e.witnessBirth)continue;
       const Index idx{b.lineage,b.key.spawn};auto& owner=hits[idx];owner.b=b;
       auto& handle=w.handles[idx];if(!handle.faces)handle.faces=wallMap(b.exterior);
       // One persistent source within each receiver family. Simultaneous fire
       // producers feed the same elapsed time and never start six new seeds.
       const u64 source=~u64(0);
       auto& f=handle.footprints[source];
       f.sourceBody=b.id;f.sourceBirth=b.birth;
       if(f.confirmedRevision!=b.last->stamp().revision || f.confirmedGeneration!=b.key.generation){
         const auto stillOwns=[&](const BurnCandidate& c){
           return b.last->owns_burn(c.surface,c.domain,{c.rest[0],c.rest[1],c.rest[2]});
         };
         f.sites.removeInvalid(stillOwns);
         f.halo.erase(std::remove_if(f.halo.begin(),f.halo.end(),[&](const auto& h){return !stillOwns(h.direction);}),f.halo.end());
         if(f.halo.size()<6){
           f.halo.clear();
           const auto points=b.last->burn_sites(b.key.spawn^0x655552f87366dbdeULL,96);
           for(const auto& p:points){
             unsigned token=p.surface;const auto rest=a3(p.rest),normal=a3(p.normal);
             if(p.domain==painter_burn::Anatomy){
               token=painter_burn::interiorSurfaceToken(*handle.faces,rest,normal,p.child);
               if(!token){
                 auto& next=w.nextWall[b.lineage];if(!next)next=262144;
                 if(next>painter_burn::MaxSurface||handle.faces->walls.size()>=16384)continue;
                 painter_burn::WallWitness wall{rest,normal,p.child,next,p.domain};
                 if(!painter_burn::validWall(wall))continue;
                 if(!handle.faces.unique())handle.faces=std::make_shared<painter_burn::SourceFaces>(*handle.faces);
                 token=next++;handle.faces->walls.push_back(wall);owner.newWalls=true;
               }
             }
             painter_burn::ContactCell cell{};
             if(!painter_burn::makeContact(p.rest.x,p.rest.y,p.rest.z,p.normal.x,p.normal.y,p.normal.z,token,p.domain,1,1,cell))continue;
             BurnCandidate c{};c.rest=rest;c.world=rest;c.normal=normal;c.surface=token;
             c.child=std::uint16_t(p.child);c.domain=std::uint8_t(p.domain);f.halo.push_back({c,cell});
           }
         }
         f.confirmedRevision=b.last->stamp().revision;f.confirmedGeneration=b.key.generation;
       }
       if(f.halo.empty()){traceReceiver(e,b.id,"no_owned_surface");noteBody(1,b.id,1,"none of its surface can be found to burn");continue;}
       std::array<BurnCandidate,96> observed{};const auto count=std::min(observed.size(),f.halo.size());
       for(size_t i=0;i<count;++i)observed[i]=f.halo[i].direction;
       const unsigned period=std::max(1u,(e.areaPeriodMs*16u+999u)/1000u);
       f.sites.advance(source,b.key.spawn,e.tick16,period,observed.data(),count);
       f.areaTick=f.lastTick=e.tick16;
       if(!f.spreadShape||f.spreadRevision!=b.last->stamp().revision||f.spreadGeneration!=b.key.generation){
         f.spreadShape=b.last->burn_spread_shape();f.spreadRevision=b.last->stamp().revision;f.spreadGeneration=b.key.generation;
       }
       if(f.spreadShape&&f.sites.count()){
         std::array<painter_fire::V,6> seeds{};for(unsigned n=0;n<f.sites.count();++n)seeds[n]=f.sites.anchor(n).rest;
         auto result=w.sessions.commit(b.lineage,b.key,[&](painter_burn::Family& family){return family.reach(b.key,*f.spreadShape,body.firePose,seeds.data(),unsigned(f.sites.count()),e.tick16,period,b.key.spawn);});
         owner.publishedFront|=result==painter_burn::Status::Applied;
         // "No change" is no news: no time had passed for the body's fire (it was out of the flames for a moment, or two
         // contacts came close together). No line for it. A real refusal says which one it was.
         if(result==painter_burn::Status::Applied)
           noteBody(1,b.id,f.sites.count()>3?4u:3u,"burning: places found on its surface, places alight",f.halo.size(),f.sites.count());
         else if(result!=painter_burn::Status::NoChange)
           noteBody(1,b.id,10u+unsigned(result),"its burn was refused: status (2 its burn record is gone, 5 no usable surface or seed, 6 no free fire slot), places alight",
             size_t(result),f.sites.count());
       }
       else noteBody(1,b.id,5,"nothing alight on it yet: places found, places alight",f.halo.size(),f.sites.count());
       // A producer-confirmed fire uses only the local arrival grid. When
       // no slot or owned surface is available, retain the saved burn.

     }
     continue;
   }

   if(e.shape==Shape::AttachedSurface){
     for(const auto& body:src.bodies){const auto& b=body.binding;auto h=w.handles.find({b.lineage,b.key.spawn});
       if(h==w.handles.end()||!b.last)continue;
       auto f=h->second.footprints.find(e.sourceSerial);
       if(f==h->second.footprints.end()||f->second.halo.empty()){
         traceReceiver(e,b.id,"no_witnessed_exterior");continue;
       }
       if(b.id!=e.targetId && (f->second.sourceBody!=e.targetId||f->second.sourceBirth!=src.attachedBirth))continue;
       const u32 period=std::max<u32>(1,(e.areaPeriodMs*16u+999u)/1000u);
       if(e.tick16-f->second.areaTick<period||e.tick16==f->second.lastTick)continue;
       f->second.lastTick=e.tick16;
       auto& fstate=f->second;auto& owner=hits[{b.lineage,b.key.spawn}];owner.b=b;
       std::vector<BurnCandidate> visible;visible.reserve(fstate.halo.size());
       for(auto& h:fstate.halo){auto p=painter_burn::witnessPoint(h.cell);
         if(b.last->owns_burn(h.cell.surface,h.cell.domain,{p[0],p[1],p[2]}))visible.push_back(h.direction);}
       fstate.sites.advance(e.sourceSerial,b.key.spawn,e.tick16,period,visible.data(),visible.size());
       captureFront(b,fstate.sites);
       for(auto& h:fstate.halo){auto cell=h.cell;const auto p=painter_burn::witnessPoint(cell);
          const auto weight=fstate.sites.weight(h.direction);
          if(weight<=0.f||!b.last->owns_burn(cell.surface,cell.domain,{p[0],p[1],p[2]}))continue;
          const double energy=e.power*weight;
          cell.dose=static_cast<std::uint16_t>(std::lround(std::min(65535.,energy*65535.)));
          cell.heat=static_cast<std::uint8_t>(std::lround(std::min(255.,energy*4096.)));
          owner.cells[{cell.x,cell.y,cell.z,cell.domain,cell.surface}]=cell;
       }
     }
     continue;
   }
   for(auto& body:src.bodies){
     if(!body.binding.last)continue;
     auto trace=body.binding.last->burn_trace(body.bones);
     if(trace)targets.push_back({&body,trace});else traceReceiver(e,body.binding.id,"pose_unavailable");
   }
   if(targets.empty())continue;
   if(e.witnessOnly){
     bool needed=false;
     for(const auto& target:targets){const auto& b=target.body->binding;
       if(b.id!=e.targetId||b.birth!=e.witnessBirth)continue;
       const auto found=w.handles.find({b.lineage,b.key.spawn});
       if(found==w.handles.end()){needed=true;break;}
       const auto f=found->second.footprints.find(e.sourceSerial);
       needed=f==found->second.footprints.end()||f->second.halo.empty();break;
     }
     if(!needed)continue;
   }
   const WorldOcclusion world(src.world);
   auto trace=[&](Point direction,double reach,const Binding*& winner,gfy_live::BurnIntersection& nearest,double& energy)->bool{
     Point start=from(e.centre);
     reach=world.nearest(start,direction,reach);
     for(const auto& body:src.bodies)if(!body.binding.last)
       reach=painter_burn_capture::blockedAt(start,direction,body.centre,body.radius,reach);
     nearest={};nearest.distance=reach;
     winner=nullptr;
     for(auto& target:targets){
       const auto& b=target.body->binding;
       const double before=nearest.distance;
       auto through=[&](const gfy_live::BurnIntersection& h){
         if(!h.surface||!b.exterior||h.surface>b.exterior->roles.size()||b.exterior->roles[h.surface-1]!=1)return false;
         const auto& p=h.rest;const auto& n=h.normal;const auto prev=w.sessions.inspect(b.lineage,b.key,p.x,p.y,p.z,n.x,n.y,n.z,1,h.surface);
         return painter_burn::clothOpen(painter_burn::MaterialRole::Cloth,painter_burn::burnChart(float(p.x),float(p.y),float(p.z),float(n.x),float(n.y),float(n.z)),prev.dose/65535.f);
       };
       (*target.trace)(start,direction,reach,through,nearest);if(nearest.distance<before)winner=&b;
     }
     if(!winner||!nearest.domain)return false;
     if(e.witnessOnly&&(winner->id!=e.targetId||winner->birth!=e.witnessBirth))return false;
     if(src.limitedWorld){
       const auto p=add(start,scale(direction,nearest.distance));
       if(p.x<src.worldLo.x||p.y<src.worldLo.y||p.z<src.worldLo.z||
          p.x>src.worldHi.x||p.y>src.worldHi.y||p.z>src.worldHi.z)return false;
     }
     energy=powerAt(e,add(start,scale(direction,nearest.distance)),nearest.distance);
     return energy>0.;
   };
   auto candidate=[&](const gfy_live::BurnIntersection& h,Point dir){
      BurnCandidate c{};c.rest=a3(h.rest);c.world=a3(add(from(e.centre),scale(dir,h.distance)));c.normal=a3(h.normal);
      c.surface=h.surface?h.surface:262144u+h.child;c.child=static_cast<std::uint16_t>(h.child);c.domain=static_cast<std::uint8_t>(h.domain);return c;
   };
   auto stamp=[&](const Binding& b,const gfy_live::BurnIntersection& nearest,double local,float weight,const BurnCandidate& c,Footprint* cache){
     if(e.witnessOnly&&(!cache||nearest.domain!=painter_burn::Exterior))return;
     Index idx{b.lineage,b.key.spawn};auto& accepted=hits[idx];accepted.b=b;
     auto& h=w.handles[idx];
     if(!h.faces)h.faces=wallMap(b.exterior);
     unsigned token=nearest.surface;const auto p=a3(nearest.rest),n=a3(nearest.normal);
     if(nearest.domain!=1){
       token=painter_burn::interiorSurfaceToken(*h.faces,p,n,nearest.child);
       if(!token){
         auto& next=w.nextWall[b.lineage];if(!next)next=262144;
         if(next>painter_burn::MaxSurface||h.faces->walls.size()>=16384)return;
         painter_burn::WallWitness wall{p,n,nearest.child,next,nearest.domain};
         if(!painter_burn::validWall(wall))return;
         // Never mutate a table already handed to the render thread.
         if(!h.faces.unique())h.faces=std::make_shared<painter_burn::SourceFaces>(*h.faces);
         h.faces->walls.push_back(wall);token=next++;accepted.newWalls=true;
       }
     }
     double energy=local*e.quanta/16.*weight;painter_burn::ContactCell cell{};
     if(!painter_burn::makeContact(p[0],p[1],p[2],n[0],n[1],n[2],token,nearest.domain,
         (std::uint16_t)std::lround(std::min(65535.,energy*65535)),(std::uint8_t)std::lround(std::min(255.,energy*4096)),cell))return;
     if(cache&&cell.domain==painter_burn::Exterior&&cache->halo.size()<4096)cache->halo.push_back({c,cell});
     if(weight<=0.f||e.witnessOnly)return;
     auto& dst=accepted.cells[{cell.x,cell.y,cell.z,cell.domain,cell.surface}];if(!dst.surface||dst.dose<cell.dose)dst=cell;
   };
   double reach=e.shape==Shape::Segment||e.shape==Shape::LocalBone?e.reach:e.radius;
   if(e.shape==Shape::Box)reach=length(from(e.half));
   if(!(reach>.0001))continue;
   using Visible=std::map<Index,std::vector<BurnCandidate>>;
   Visible observed;
   auto findContact=[&](Point dir){
     const Binding* owner=nullptr;gfy_live::BurnIntersection hit;double energy=0.;
     if(trace(dir,reach,owner,hit,energy))observed[{owner->lineage,owner->key.spawn}].push_back(candidate(hit,dir));
   };
   const bool directional=e.shape==Shape::LocalBone||e.shape==Shape::Segment;
   if(directional){
     const auto d=norm(from(e.direction));const Binding* owner=nullptr;gfy_live::BurnIntersection contact;double heat=0.;
     if(trace(d,reach,owner,contact,heat)){
       const double footprint=std::min(.025,contact.distance*.25);
       painter_burn_sampling::Patch initial(d.x,d.y,d.z,contact.distance,footprint,unsigned(e.sourceSerial));
       while(!initial.slice(128,[&](double x,double y,double z){findContact({x,y,z});})){}
     }
   }else for(const auto& t:targets){
     Point delta=sub(t.body->centre,from(e.centre));double dist=length(delta);if(dist>reach+t.body->radius)continue;
     Point axis=dist>1e-6?scale(delta,1./dist):Point{0,1,0};
     double cap=-1.;if(dist>t.body->radius){double r=t.body->radius/dist;cap=std::sqrt(std::max(0.,1.-r*r));}
     for(unsigned i=0;i<256;++i)painter_burn_sampling::discover(axis.x,axis.y,axis.z,cap,unsigned(e.sourceSerial),i,256,
        [&](double x,double y,double z){findContact({x,y,z});});
   }
   for(auto& pair:observed){
     auto& handle=w.handles[pair.first];auto& fs=handle.footprints;
     for(auto f=fs.begin();f!=fs.end();)if(e.tick16-f->second.lastTick>256u)f=fs.erase(f);else ++f;
     if(!fs.count(e.sourceSerial)&&fs.size()>=8)continue;
     auto& f=fs[e.sourceSerial];
     if(!f.sourceBirth){
       for(const auto& t:targets){const auto& b=t.body->binding;
         if(std::make_pair(b.lineage,b.key.spawn)==pair.first){f.sourceBody=b.id;f.sourceBirth=b.birth;break;}}
     }
     const unsigned period=e.areaPeriodMs?std::max(1u,(e.areaPeriodMs*16u+999u)/1000u):1u;
     f.sites.advance(e.sourceSerial,pair.first.second,e.tick16,period,pair.second.data(),pair.second.size());
     if(!e.witnessOnly)for(const auto& t:targets){
       const auto& b=t.body->binding;
       if(b.lineage==pair.first.first&&b.key.spawn==pair.first.second){captureFront(b,f.sites);break;}
     }
     f.areaTick=f.lastTick=e.tick16;
     // Each kernel samples its own source-facing neighbourhood. Every point must win
     // the real visibility/ownership trace even when a different body moves in front.
     if(e.areaPeriodMs)f.halo.clear();
     for(size_t i=0;i<f.sites.count();++i){
       const auto& anchor=f.sites.anchor(i);const auto& p=anchor.world;
       auto delta=sub({p[0],p[1],p[2]},from(e.centre));double dist=length(delta);if(dist<.0001||dist>reach)continue;
       const auto d=scale(delta,1./dist);double radius=f.sites.radius(i);
       auto visit=[&](double x,double y,double z,bool halo){
         Point dir{x,y,z};const Binding* owner=nullptr;gfy_live::BurnIntersection h;double heat=0.;
         if(!trace(dir,reach,owner,h,heat)||std::make_pair(owner->lineage,owner->key.spawn)!=pair.first)return;
         const auto c=candidate(h,dir);const auto local=sub(h.rest,{anchor.rest[0],anchor.rest[1],anchor.rest[2]});
         const double limit=halo?f.sites.maxRadius(i):radius;
         if(dot(local,local)>limit*limit)return;
         float weight=f.sites.weight(c);if(weight<=0.f&&(!halo||c.domain!=painter_burn::Exterior))return;
         stamp(*owner,h,heat,weight,c,halo?&f:nullptr);
       };
       const auto seed=unsigned(e.sourceSerial^(i*0x9e3779b9u)^pair.first.second);
       painter_burn_sampling::Patch patch(d.x,d.y,d.z,dist,radius,seed);
       const Binding* at=nullptr;gfy_live::BurnIntersection centre;double energy=0.;
       if(trace(d,reach,at,centre,energy)&&at->key.spawn==pair.first.second&&
           length(sub(centre.rest,{anchor.rest[0],anchor.rest[1],anchor.rest[2]}))<=radius){
         auto n=centre.posedNormal;patch.orient(n.x,n.y,n.z);
       }
       patch.contacts(49,[&](double x,double y,double z){visit(x,y,z,false);});
       {std::lock_guard<std::mutex> lk(w.mutex);if(w.stopping)return;}
       if(e.areaPeriodMs){
         // Preserve witnessed exterior neighbours for the addon's short attached fire.
         // Following the moving owner uses original rest points and never exposes deeper tissue.
         const auto x=norm(cross(d,std::abs(d.y)<.9?Point{0,1,0}:Point{1,0,0})),y=cross(d,x);
         for(unsigned j=0;j<48;++j){double a=2.39996322972865*(j+seed%97);double r=f.sites.maxRadius(i)*std::sqrt((j+.5)/48.);
           auto dir=norm(add(delta,add(scale(x,r*std::cos(a)),scale(y,r*std::sin(a)))));visit(dir.x,dir.y,dir.z,true);}
       }
     }
   }
 }
 for(auto& it:hits){
   auto& hit=it.second;auto& b=hit.b;std::vector<painter_burn::ContactCell> cells;cells.reserve(hit.cells.size());
   for(auto& q:hit.cells)cells.push_back(q.second);
   // Source pulses are globally unique, but other emitters have gaps between calls. The worker has an owner-local stream.
   if(cells.empty()&&!hit.publishedFront)continue;
   if(PainterBurnTraceEnabled()){
     const auto source=sources.front().e.confirmedBody?~u64(0):sources.front().e.sourceSerial;
     auto& footprints=w.handles[it.first].footprints;
     const auto f=footprints.find(source);
     if(f!=footprints.end()&&f->second.tracedGeneration!=b.key.generation){
       f->second.tracedGeneration=b.key.generation;
       Msg("[painter_burn_trace] stage=contacts source=%llu id=%u lineage=%llu spawn=%llu generation=%llu geometry_revision=%llu cells=%zu",
           source,unsigned(b.id),b.lineage,b.key.spawn,b.key.generation,b.last->stamp().revision,cells.size());
     }
   }
   bool applied=hit.publishedFront;
   // A full directory can still update marks that already own cells.
   // Only bisect an allocation refusal. Refused transactions never write partial state.
   auto submitRange=[&](auto&& again,size_t off,size_t n)->void{
     auto status=w.sessions.apply(b.lineage,b.key,++w.sequence,sources.front().e.tick16,cells.data()+off,n);
     if(status==painter_burn::Status::Applied){applied=true;return;}
     if(status==painter_burn::Status::Capacity&&n>1){auto half=n/2;again(again,off,half);again(again,off+half,n-half);}
   };
   for(size_t off=0;off<cells.size();off+=256)submitRange(submitRange,off,std::min<size_t>(256,cells.size()-off));
   const auto result=applied?painter_burn::Status::Applied:painter_burn::Status::NoChange;
   if(result==painter_burn::Status::Applied){
     std::lock_guard<std::mutex> lk(w.mutex);
     if(hit.newWalls||!b.faces)w.wallUpdates.push_back({b.id,b.birth,b.world,b.key,b.last->stamp().revision,w.handles[it.first].faces});
     const auto stamp=std::make_pair(u64(b.key.generation),u64(b.last->stamp().revision));
     if(b.last->needs_burn_prime() && (w.primed.find(it.first)==w.primed.end()||w.primed[it.first]!=stamp)){
       w.primed[it.first]=stamp;w.primes.push_back({b.id,b.birth,b.world,b.key,b.last->stamp().revision,{}});
     }
   }
 }
}
void refresh(CObject& o,const std::shared_ptr<gfy_live::Session>& session) noexcept {try{
 if(!active())return;
 auto& m=mainState();std::lock_guard<std::mutex> lk(m.mutex);auto it=m.bodies.find(o.ID());
 if(!m.work||it==m.bodies.end()||it->second.worker.lock()!=session||!it->second.exterior)return;
 auto& b=it->second;const auto before=b.last;auto now=session->capture_damage();
 // Clear the published interior catalogue before this geometry can draw. Original face IDs remain stable.
 if(b.faces&&!b.faces->walls.empty()){b.faces=b.exterior;if(auto* k=smart_cast<IKinematics*>(o.Visual()))k->LL_PainterBurnOwner(b.lineage,b.key.spawn,b.key.generation,b.exterior);}
 b.last=now;auto& w=*m.work;
 w.post([&w,b,before,now]{
   auto it=w.handles.find({b.lineage,b.key.spawn});if(it==w.handles.end()||!it->second.faces)return;
   if(it->second.faces->walls.empty())return;
   auto table=wallMap(it->second.faces->authored);
   for(const auto& wall:it->second.faces->walls){
    if(!before||!now)continue;
    bool retained=true;Point p{wall.rest[0],wall.rest[1],wall.rest[2]};
    const auto domain=wall.domain;
    if(!now->owns_burn(wall.token,domain,p))continue;
    const double h=now->damage_pitch();
    for(int x=-1;x<=1&&retained;++x)for(int y=-1;y<=1&&retained;++y)for(int z=-1;z<=1;++z){
      Point q{p.x+x*h,p.y+y*h,p.z+z*h};if(before->sample_rest(q).removed!=now->sample_rest(q).removed){retained=false;break;}
    }
    if(retained)table->walls.push_back(wall);
   }
   it->second.faces=table;
   if(now&&!table->walls.empty()){std::lock_guard<std::mutex> l(w.mutex);w.wallUpdates.push_back({b.id,b.birth,b.world,b.key,now->stamp().revision,table});}
 });
 }catch(...){}}
// Resolve the original receiver identity, never a replacement object in the same ID slot.
static bool resolve(Main& m,CObject& o,Binding& b,bool pristine){
 auto* k=smart_cast<IKinematics*>(o.Visual());if(!k||b.birth!=o.PainterBirthCookie()||b.world!=Level().PainterWorldCookie())return false;
 bool missing=false;
 {std::lock_guard<std::mutex> lock(m.mutex);auto i=m.bodies.find(o.ID());missing=i==m.bodies.end()||i->second.birth!=b.birth;}
 if(missing){
    auto* shell=smart_cast<CPhysicsShellHolder*>(&o);
    if(!shell||!painter_piece_continuation::bind_burn_receiver(*shell)){
        auto* npc=smart_cast<CAI_Stalker*>(&o);auto* support=npc?npc->character_physics_support():nullptr;
        if(!support||!support->BindExistingBurnReceiver())return false;
    }
 }
 std::lock_guard<std::mutex> lock(m.mutex);auto i=m.bodies.find(o.ID());if(i==m.bodies.end()||i->second.birth!=b.birth)return false;
 auto& live=i->second;auto session=live.worker.lock();if(!session)return false;
 auto snap=session->published_burn_surface();if(!snap||snap->stamp().owner.spawn_cookie!=live.key.spawn||
  snap->stamp().expected_generation!=k->LL_RenderGeometryGeneration()||(pristine&&(snap->stamp().revision||k->LL_RenderGeometryGeneration())))return false;
 if(!live.exterior){
   if(!live.facesPending&&!live.facesFailed){
     auto roles=k->LL_PainterBurnMaterialRoles();
     live.facesPending=true;auto& w=worker(m);
     w.post([&w,session,roles=std::move(roles),request=live]{
       std::shared_ptr<const painter_burn::SourceFaces> faces;
       try{faces=faceTable(roles,session);}catch(const std::exception& ex){Msg("[painter_burn] source table: %s",ex.what());}
       std::lock_guard<std::mutex> done(w.mutex);
       w.faceReady.push_back({request.id,request.birth,request.world,request.key,0,std::move(faces)});
     });
   }
   b.facesPending=live.facesPending;
   return false;
 }
 // A family may have split before its first fire. Its cold branch did
 // not participate in the original heat split. Birth is idempotent for a
 // transferred owner and precedes its first deposit on the same queue.
 if(!live.birthPosted){
    auto& w=worker(m);auto lineage=live.lineage;auto key=live.key;
    w.post([&w,lineage,key]{w.sessions.birth(lineage,key);});live.birthPosted=true;
 }
 b=live;b.last=std::move(snap);return true;
}
static void dispatch(Main& m,std::vector<Exposure> packet){
 if(packet.empty())return;
 std::lock_guard<std::mutex> lk(m.mutex);if(m.stopped)return;auto& w=worker(m);
 {std::lock_guard<std::mutex> lock(w.mutex);if(w.waitingHeat>=24)return;++w.waitingHeat;}
 w.post([&w,p=std::move(packet)] {struct Done{Worker& w;~Done(){std::lock_guard<std::mutex> lk(w.mutex);--w.waitingHeat;}} done{w};apply(w,p);});
}
static void resumeExposures(Main& m){
 std::deque<std::vector<Exposure>> deferred;deferred.swap(m.deferred);
 for(auto& packet:deferred){
   if(packet.empty())continue;
   const bool expired=Device.dwTimeContinual-packet.front().queuedAt>3000u;
   bool ready=true;
   for(auto& e:packet)for(auto& body:e.bodies)if(!body.binding.last){
     auto& b=body.binding;auto* o=Level().Objects.net_Find(b.id);
     if(!o||o->getDestroy()||o->PainterBirthCookie()!=b.birth||b.world!=Level().PainterWorldCookie()){
       traceReceiver(e.e,b.id,"stale_occluder");continue;
     }
     auto* k=smart_cast<IKinematics*>(o->Visual());
     if(!k||k->LL_RenderGeometryGeneration()!=b.capturedGeometry){traceReceiver(e.e,b.id,"geometry_changed");continue;}
     if(!resolve(m,*o,b,false)){if(expired)traceReceiver(e.e,b.id,"expired_occluder");else ready=false;}
   }
   if(ready)dispatch(m,std::move(packet));else m.deferred.push_back(std::move(packet));
 }
}
static void capture(Main& m,const Event& e){
 if(!e.confirmedBody||!e.witnessBirth||e.targetWorld!=Level().PainterWorldCookie())return;
 auto* o=Level().Objects.net_Find(e.targetId);
 if(!o||o->getDestroy()||o->PainterBirthCookie()!=e.witnessBirth)return;
 auto* k=smart_cast<IKinematics*>(o->Visual());if(!k)return;
 Binding b;b.id=o->ID();b.birth=e.witnessBirth;b.world=e.targetWorld;
 if(!resolve(m,*o,b,false)&&!b.facesPending){
    auto* npc=smart_cast<CAI_Stalker*>(o);auto* support=npc?npc->character_physics_support():nullptr;
    if(!support||k->LL_RenderGeometryGeneration()||!support->BindPainterVoxelLive(false)){
      noteBody(0,o->ID(),1,!support?"refused: not a stalker's body":k->LL_RenderGeometryGeneration()?"refused: its wounds are not bound for burning (geometry generation, 0)":"refused: its body data could not be bound (0, 0)",
        size_t(k->LL_RenderGeometryGeneration()));
      return;
    }
    // The bind call also answers "true" when it only waits because the body-data budget is full: then nothing is bound and
    // nothing is asked for, and this contact would wait three seconds for surface data that cannot come.
    if(support->PainterVoxelLiveState()==0){
      noteBody(0,o->ID(),6,"not taken: the body-data budget is full");
      return;
    }
    resolve(m,*o,b,true);
 }
 b.capturedGeometry=k->LL_RenderGeometryGeneration();
 Exposure exp;exp.queuedAt=Device.dwTimeContinual;exp.e=e;exp.attachedBirth=b.birth;
 Body body;body.binding=std::move(b);
 // Read the last published animation/physics pose. Never recalculate bones for fire.
 body.firePose.rig=body.binding.fireRig;const u16 bones=k->LL_BoneCount();body.firePose.boneUp.reserve(bones);
 const auto& object=o->XFORM();body.firePose.up=painter_fire::unit({object.i.y,object.j.y,object.k.y});
 for(u16 bone=0;bone<bones;bone++){
   Fmatrix world;world.mul_43(object,k->LL_GetBoneInstance(bone).mRenderTransform);
   body.firePose.boneUp.push_back(painter_fire::unit({world.i.y,world.j.y,world.k.y}));
 }
 const u16 chest=k->LL_BoneID("bip01_spine2");
 if(chest<bones){Fmatrix bind;bind.invert(k->LL_GetData(chest).m2b_transform);
   body.firePose.torso={bind.c.x,bind.c.y,bind.c.z};body.firePose.up=body.firePose.boneUp[chest];}
 exp.bodies.push_back(std::move(body));
 std::vector<Exposure> packet;packet.push_back(std::move(exp));
 if(!packet.front().bodies.front().binding.last){
   noteBody(0,o->ID(),2,"taken; waiting for its surface data (waiting packets, 0)",m.deferred.size());
   if(m.deferred.size()<24)m.deferred.push_back(std::move(packet));
   return;
 }
 noteBody(0,o->ID(),3,"taken; its fire contact goes to the heat worker");
 dispatch(m,std::move(packet));
}
void releaseWorld(u64 world) noexcept {
 if(!active())return;auto& m=mainState();
 m.deferred.clear();
 std::lock_guard<std::mutex> lk(m.mutex);
 m.events.clear();m.confirmed.clear();
}
void disableMaster() noexcept {try{
    Main* m=activeHost.load(std::memory_order_acquire);if(!m)return;
    std::map<u16,Binding> former;
    std::unique_ptr<Worker> old;
    {
        std::lock_guard<std::mutex> hold(m->mutex);
        former.swap(m->bodies);old=std::move(m->work);
        m->events.clear();m->confirmed.clear();m->ground.clear();m->deferred.clear();m->epoch=0;
    }
    // A finishing job belongs to the old generation. Close admission before detaching its renderer state.
    if(old)old->stop();
    if(g_pGameLevel)for(const auto& item:former){
        const auto& b=item.second;
        if(b.world!=Level().PainterWorldCookie())continue;
        auto* o=Level().Objects.net_Find(item.first);
        if(!o || o->getDestroy() || o->PainterBirthCookie()!=b.birth)continue;
        if(auto* k=smart_cast<IKinematics*>(o->Visual()))k->LL_PainterBurnOwner(0,0,0,{});
    }
}catch(...) {Msg("! [painter_burn] could not retire a body on switch-off");}}
void frame() noexcept {try{
 if(!active())return;
 if(::Render)::Render->PainterBurnClock(painter_burn_source::tick16(Device.dwTimeGlobal));
 auto& m=mainState();std::deque<Event> pending;std::deque<WallUpdate> walls,primes,ready;
 {std::lock_guard<std::mutex> lk(m.mutex);if(m.stopped)return;pending.swap(m.events);
  if(m.work&&::Render){const auto epoch=::Render->PainterBurnEpoch();if(epoch!=m.epoch){m.epoch=epoch;auto& w=*m.work;w.post([&w]{w.sessions.resnapshot();});}}
  if(m.work){std::lock_guard<std::mutex> l(m.work->mutex);walls.swap(m.work->wallUpdates);primes.swap(m.work->primes);ready.swap(m.work->faceReady);}}
 for(const auto& u:ready){
   std::lock_guard<std::mutex> lk(m.mutex);auto i=m.bodies.find(u.id);
   if(i==m.bodies.end()||i->second.birth!=u.birth||i->second.world!=u.world||!(i->second.key==u.key))continue;
   i->second.facesPending=false;i->second.facesFailed=!u.faces;i->second.exterior=u.faces;
 }
 for(const auto& u:walls){
   auto* o=Level().Objects.net_Find(u.id);if(!o||o->PainterBirthCookie()!=u.birth||Level().PainterWorldCookie()!=u.world||o->getDestroy())continue;
   auto* k=smart_cast<IKinematics*>(o->Visual());std::lock_guard<std::mutex> lk(m.mutex);auto i=m.bodies.find(u.id);
   if(!k||i==m.bodies.end()||!(i->second.key==u.key))continue;
   auto s=i->second.worker.lock();auto snap=s?s->published_burn_surface():nullptr;
   if(!snap||!u.faces)continue;
   const auto original=u.faces->authored?u.faces->authored:u.faces;i->second.exterior=original;
   const auto table=snap->stamp().revision==u.revision?u.faces:original;i->second.faces=table;
   k->LL_PainterBurnOwner(i->second.lineage,u.key.spawn,u.key.generation,table);
 }
 for(const auto& u:primes){
   auto* o=Level().Objects.net_Find(u.id);if(!o||o->PainterBirthCookie()!=u.birth||Level().PainterWorldCookie()!=u.world||o->getDestroy())continue;
   if(!PainterPrimeBurnGeometry(o->ID(),u.key.spawn)){std::lock_guard<std::mutex> lk(m.mutex);
     if(m.work&&m.bodies.count(u.id)&&m.bodies[u.id].key==u.key){auto& w=*m.work;auto id=std::make_pair(m.bodies[u.id].lineage,u.key.spawn);
       const auto failed=std::make_pair(u64(u.key.generation),u64(u.revision));
       w.post([&w,id,failed]{auto old=w.primed.find(id);if(old!=w.primed.end()&&old->second==failed)w.primed.erase(old);});}}
 }
 resumeExposures(m);
 unsigned handled=0;while(!pending.empty()&&handled++<4){capture(m,pending.front());pending.pop_front();}
 if(!pending.empty()){std::lock_guard<std::mutex> lk(m.mutex);m.events.insert(m.events.begin(),pending.begin(),pending.end());}
 }catch(...){Msg("[painter_burn] exposure skipped");}}
void quit() noexcept {hostStopped=true;if(!activeHost.load(std::memory_order_acquire))return;auto& m=mainState();Worker* w=nullptr;{std::lock_guard<std::mutex> l(m.mutex);m.stopped=true;w=m.work.get();}if(w)w->stop();}
} // painter_burn_host
extern int painter_burn_enabled; // console_commands.cpp
namespace painter_burn_source {
bool isBody(CObject& target) noexcept {
 if(smart_cast<CAI_Stalker*>(&target))return true;
 auto* shell=smart_cast<CPhysicsShellHolder*>(&target);
 return shell&&painter_piece_continuation::owns(*shell,0);
}
void confirm(CObject& target,Event e) noexcept {try{
 if(!enabled()||!g_pGameLevel||target.getDestroy()||Device.dwPrecacheFrame||!OnServer())return;
 if(!isBody(target))return;
 e.shape=Shape::AttachedSurface;e.confirmedBody=true;e.targetId=target.ID();
 e.witnessBirth=target.PainterBirthCookie();e.targetWorld=Level().PainterWorldCookie();
 e.shapeCount=1;e.shapeIndex=0;submit(e);
 }catch(...){}}
void submit(const Event& source) noexcept {try{if(!source.confirmedBody)return;Event e=source;if(!painter_burn_enabled||painter_burn_host::hostStopped.load())return;auto& m=painter_burn_host::mainState();std::lock_guard<std::mutex> lk(m.mutex);
 const bool accepted=!m.stopped&&m.events.size()<1024&&e.power>0&&e.quanta&&e.sourceSerial&&e.pulse;
 if(PainterBurnTraceEnabled())Msg("[painter_burn_trace] stage=emitted source=%s serial=%llu pulse=%llu emitter=%u body=%u power=%.4f radius=%.3f accepted=%u",causeName(e.cause),e.sourceSerial,e.pulse,unsigned(e.emitterId),unsigned(e.targetId),e.power,e.radius,unsigned(accepted));
 if(accepted){
   auto it=m.confirmed.find(e.targetId);
   if(it!=m.confirmed.end()&&it->second.birth==e.witnessBirth&&it->second.world==e.targetWorld){
     const u32 gap=e.tick16-it->second.tick;
     if(gap<2||gap>=0x80000000u)return;
     e.quanta=std::min(8u,gap);
   }else e.quanta=std::min(16u,std::max(1u,e.quanta));
   m.confirmed[e.targetId]={e.witnessBirth,e.targetWorld,e.tick16};
   e.sourceSerial=(1ull<<63)|(e.witnessBirth&0x7fffffffffffffffull);
   e.areaPeriodMs=std::max<unsigned short>(250,e.areaPeriodMs);
   m.events.push_back(e);
 }
}catch(...){}}
}

#pragma pop_macro("far")
#pragma pop_macro("near")
