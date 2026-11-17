// Exact local voxel-face/body-surface partition and shared-provenance join.
// Included after the isolated kernel types; no engine/runtime dependencies.
#include <limits>
#include <queue>
#include <tuple>
struct CavityProvenanceError : std::runtime_error { using std::runtime_error::runtime_error; };
#include "HarmonicIC0.h"
// Hashes for lookup-only containers. Never iterate a container keyed by these
// where order reaches output or a gate; ordered traversals stay on std::map.
// (This header is included inside a namespace: use only <unordered_map>.)
// Exactly std::llround (round half away from zero) for |x| < 2^52, without
// the library call; anything else (including NaN) defers to std::llround.
inline std::int64_t roundHalfAway(double x){
    if(!(std::abs(x)<4503599627370496.0))return std::llround(x);
    std::int64_t i=std::int64_t(x);const double d=x-double(i);
    if(d>=.5)++i;else if(d<=-.5)--i;return i;
}
// Identical to quant()/edgeKey() of the kernel, using roundHalfAway.
inline Quant quantExact(Vec p){return {roundHalfAway(p.x*1e8),roundHalfAway(p.y*1e8),roundHalfAway(p.z*1e8)};}
inline Edge edgeKeyExact(Vec a,Vec b){Edge key{quantExact(a),quantExact(b)};if(key[1]<key[0])std::swap(key[0],key[1]);return key;}
inline std::uint64_t quantMix(std::uint64_t x){x+=0x9E3779B97F4A7C15ull;x=(x^(x>>30))*0xBF58476D1CE4E5B9ull;x=(x^(x>>27))*0x94D049BB133111EBull;return x^(x>>31);}
struct QuantHash {std::size_t operator()(const Quant&q)const{std::uint64_t h=0;for(auto v:q)h=quantMix(h^std::uint64_t(v));return std::size_t(h);}};
template<std::size_t N>struct QuantArrayHash {std::size_t operator()(const std::array<Quant,N>&key)const{std::uint64_t h=0;for(const auto&q:key)for(auto v:q)h=quantMix(h^std::uint64_t(v));return std::size_t(h);}};
// Open-addressed hash map for the hot lookup tables of the join. Entries live
// densely in FIRST-INSERTION order (try_emplace/operator[] keep the first
// value, exactly like std::unordered_map); iteration visits that order, and
// every caller that iterates is order-free or sorts. References and iterators
// are invalidated by insertion (callers use them only before the next insert).
template<class K,class V,class H>class FlatMap {
public:
    using value_type=std::pair<K,V>;using iterator=value_type*;using const_iterator=const value_type*;
    iterator begin(){return items_.data();}iterator end(){return items_.data()+items_.size();}
    const_iterator begin()const{return items_.data();}const_iterator end()const{return items_.data()+items_.size();}
    std::size_t size()const{return items_.size();}bool empty()const{return items_.empty();}
    void clear(){items_.clear();std::fill(slots_.begin(),slots_.end(),empty_);}
    void reserve(std::size_t n){items_.reserve(n);if(n*2>slots_.size())rehash(n*2);}
    iterator find(const K&k){const auto s=locate(k);return s==npos_?end():items_.data()+s;}
    const_iterator find(const K&k)const{const auto s=locate(k);return s==npos_?end():items_.data()+s;}
    std::size_t count(const K&k)const{return locate(k)!=npos_;}
    template<class...A>std::pair<iterator,bool>try_emplace(const K&k,A&&...a){
        if((items_.size()+1)*2>slots_.size())rehash(std::max<std::size_t>(16,slots_.size()*2));
        std::size_t at=mix(k)&mask_;
        for(;slots_[at]!=empty_;at=(at+1)&mask_)if(items_[slots_[at]].first==k)return {items_.data()+slots_[at],false};
        slots_[at]=std::uint32_t(items_.size());
        items_.emplace_back(std::piecewise_construct,std::forward_as_tuple(k),std::forward_as_tuple(std::forward<A>(a)...));
        return {&items_.back(),true};
    }
    std::pair<iterator,bool>emplace(const K&k,V v){return try_emplace(k,std::move(v));}
    V&operator[](const K&k){return try_emplace(k).first->second;}
    V&at(const K&k){auto it=find(k);if(it==end())throw std::out_of_range("flat_map_at");return it->second;}
    const V&at(const K&k)const{auto it=find(k);if(it==end())throw std::out_of_range("flat_map_at");return it->second;}
private:
    static constexpr std::uint32_t empty_=~std::uint32_t(0);static constexpr std::size_t npos_=~std::size_t(0);
    std::vector<value_type> items_;std::vector<std::uint32_t> slots_;std::size_t mask_{};H hash_{};
    std::size_t mix(const K&k)const{return std::size_t(quantMix(std::uint64_t(hash_(k))));}
    std::size_t locate(const K&k)const{if(slots_.empty())return npos_;
        for(std::size_t at=mix(k)&mask_;;at=(at+1)&mask_){const auto s=slots_[at];if(s==empty_)return npos_;if(items_[s].first==k)return s;}}
    void rehash(std::size_t want){std::size_t n=16;while(n<want)n*=2;slots_.assign(n,empty_);mask_=n-1;
        for(std::uint32_t i=0;i<items_.size();++i){std::size_t at=mix(items_[i].first)&mask_;while(slots_[at]!=empty_)at=(at+1)&mask_;slots_[at]=i;}}
};
// Every thread the kernel starts calls enterPainterThread() before any other
// work: the host's allocation-failure policy (LiveDamageWorker.h) then applies
// to it, so an allocation failure is an exception that the owning job catches.
inline std::atomic<void(*)()> threadEntryHook{nullptr};
inline void enterPainterThread() noexcept{if(auto hook=threadEntryHook.load(std::memory_order_acquire))hook();}
// Runs body(first,last,chunk) over contiguous chunks of [0,n) on up to four
// threads (the caller runs chunk 0). Used only where each item is a pure
// function of shared read-only data, so results are identical to a serial
// loop; the first failing chunk (in chunk order) rethrows.
// Persistent worker pool (sized like the region threads: player floor 4C/8T -> 4, 6C/12T -> 6, 8C/16T+ -> 8, plus two;
// env GFY_REGION_THREADS). Threads are created once, so per-thread caches (the region patch copy, tile caches) survive
// between jobs, and no job pays thread creation. A section started from a pool thread, or from a thread that is itself
// running pool-style work, runs inline: nested sections never spawn (they oversubscribed the CPU before).
inline thread_local bool painterInlineParallel=false;
// done() notifies under the lock, and parallelChunks keeps the latch alive through shared ownership: a waiter that sees
// n==0 returns and releases its reference while the last helper may still be inside done() (the former stack latch
// could be destroyed under a pending notify_all: heap corruption and access violations in multi-body blast runs).
struct Latch {std::mutex m;std::condition_variable cv;std::size_t n;explicit Latch(std::size_t k):n(k){}
    void done(){std::lock_guard<std::mutex> l(m);if(--n==0)cv.notify_all();}
    void release(std::size_t k){std::lock_guard<std::mutex> l(m);n-=k;if(n==0)cv.notify_all();} // R68: tasks that were never queued
    void wait(){std::unique_lock<std::mutex> l(m);cv.wait(l,[&]{return n==0;});}};
#ifdef GFY_VOXEL_WORKER_TESTS
// TEST-ONLY (R68 drain fixture): submitFailAt >= 0 lets that many more submits through, then the next one throws std::bad_alloc
// (once); submitBeforeFail runs first on the submitting thread.
inline std::atomic<long> submitFailAt{-1};inline std::function<void()> submitBeforeFail;
#endif
// quitfix (2026-11-13): orderly end of the kernel's worker threads.
// A service that owns threads for the whole process (the shared pool below, the asset preparation worker, the separation,
// instant-sever, prefit, warm, tile-base and fragment work services, the retirement thread) is made with
// worker_quit::immortal: it is never destroyed, so no destructor joins a thread at process exit, where the objects a job in
// flight needs may already be gone. That was the quit stall of 2026-11-13: this pool's static, made inside the first asset
// build, was destroyed BEFORE the preparation worker's static; the build in flight then queued its next parallel section on
// the dead pool and waited for ever, and the process exit waited for that worker.
// Instead the host calls worker_quit::stopAll() on the main thread before main returns (LiveDamageWorker.h quit_workers):
// job runners first (rank 0), then what their jobs hand work to (rank 1), this pool last (rank 2). Every static still exists
// then, so a job in flight ends normally (with the pool); each service's quit() drops its queued work. Once the runners are
// joined, a parallel section runs on its caller (parallelChunks), so nothing can wait for a pool that has gone.
namespace worker_quit {
struct Entry {void* owner;int rank;void (*stop)(void*);};
struct Registry {std::mutex mutex;std::vector<Entry> entries;};
inline Registry& registry(){static Registry* value=new Registry;return *value;} // never destroyed either
inline std::atomic<bool> stopping{false};
inline bool quitting()noexcept{return stopping.load(std::memory_order_acquire);}
template<class T,class...A>T& immortal(int rank,A&&...a){
    T* made=new T(std::forward<A>(a)...);
    try{auto& r=registry();std::lock_guard<std::mutex> lock(r.mutex);r.entries.push_back(Entry{made,rank,[](void* p){static_cast<T*>(p)->quit();}});}
    catch(...){} // not enlisted: it is not stopped in order, and it never blocks the exit either
    return *made;
}
inline void stopAll()noexcept{
    for(int rank=0;rank<=2;++rank){
        if(rank==2)stopping.store(true,std::memory_order_release); // the jobs in flight kept the pool up to here
        for(;;){
            Entry next{};bool found=false;
            try{auto& r=registry();std::lock_guard<std::mutex> lock(r.mutex);
                for(auto it=r.entries.begin();it!=r.entries.end();++it)if(it->rank==rank){next=*it;r.entries.erase(it);found=true;break;}}
            catch(...){}
            if(!found)break;
            try{next.stop(next.owner);}catch(...){}
        }
    }
}
}
class WorkerPool {
    std::vector<std::thread> threads_;std::mutex m_;std::condition_variable cv_;std::deque<std::function<void()>> q_;bool stop_=false;
    void loop(){painterInlineParallel=true;enterPainterThread();
        for(;;){std::function<void()> f;{std::unique_lock<std::mutex> l(m_);cv_.wait(l,[&]{return stop_||!q_.empty();});if(stop_&&q_.empty())return;f=std::move(q_.front());q_.pop_front();}f();}}
public:
    explicit WorkerPool(std::size_t n){for(std::size_t i=0;i<n;++i)threads_.emplace_back([this]{loop();});}
    // queued chunks still run (the threads leave on an empty queue), then every thread is joined
    void quit()noexcept{{std::lock_guard<std::mutex> l(m_);stop_=true;}cv_.notify_all();for(auto&t:threads_)if(t.joinable())t.join();}
    ~WorkerPool(){quit();}
    std::size_t size()const{return threads_.size();}
    void submit(std::function<void()> f){
#ifdef GFY_VOXEL_WORKER_TESTS
        for(long k=submitFailAt.load();k>=0;)if(submitFailAt.compare_exchange_weak(k,k-1)){if(k==0){if(submitBeforeFail)submitBeforeFail();throw std::bad_alloc();}break;}
#endif
        {std::lock_guard<std::mutex> l(m_);q_.push_back(std::move(f));}cv_.notify_one();}
};
inline std::size_t regionThreadCount(){static const std::size_t n=[]{if(const char* e=std::getenv("GFY_REGION_THREADS"))return std::size_t(std::max(1,std::atoi(e)));
    const unsigned hw=std::thread::hardware_concurrency();return std::size_t(hw>=16?8:hw>=12?6:4);}();return n;}
inline WorkerPool& workerPool(){static WorkerPool& pool=worker_quit::immortal<WorkerPool>(2,regionThreadCount()+2);return pool;} // quitfix: never destroyed
// Width of a parallel section: the caller plus one pool thread per region thread (the pool keeps two spare).
inline std::size_t parallelWidth(){return regionThreadCount()+1;}
template<class F>void parallelChunks(std::size_t n,std::size_t grain,F&&body){
    const std::size_t chunks=n>=2*grain?std::min<std::size_t>(parallelWidth(),n/grain):1;
    if(chunks<=1||painterInlineParallel||worker_quit::quitting()){body(std::size_t(0),n,std::size_t(0));return;} // quitfix: the pool is going
    std::vector<std::exception_ptr> failure(chunks);
    auto run=[&](std::size_t c){try{body(n*c/chunks,n*(c+1)/chunks,c);}catch(...){failure[c]=std::current_exception();}};
    const auto latch=std::make_shared<Latch>(chunks-1);auto& pool=workerPool();
    // R68: a submit that throws leaves the chunks already queued holding this frame: the chunks never queued leave the latch and
    // the queued ones finish before the submit's failure travels on
    std::size_t queued=0;
    try{for(std::size_t c=1;c<chunks;++c){pool.submit([&run,latch,c]{run(c);latch->done();});++queued;}}
    catch(...){latch->release(chunks-1-queued);latch->wait();throw;}
    run(0);latch->wait();
    for(auto&f:failure)if(f)std::rethrow_exception(f);
}
struct Bounds {Vec lo{1e30,1e30,1e30},hi{-1e30,-1e30,-1e30};void add(Vec p){for(int a=0;a<3;++a){lo[a]=std::min(lo[a],p[a]);hi[a]=std::max(hi[a],p[a]);}}};
bool overlapBounds(const Bounds&a,const Bounds&b){for(int k=0;k<3;++k)if(a.hi[k]<b.lo[k]-1e-9||a.lo[k]>b.hi[k]+1e-9)return false;return true;}
Vec closestTriangle(Vec p,Vec a,Vec b,Vec c){Vec ab=b-a,ac=c-a,ap=p-a;double d1=dot(ab,ap),d2=dot(ac,ap);if(d1<=0&&d2<=0)return a;Vec bp=p-b;double d3=dot(ab,bp),d4=dot(ac,bp);if(d3>=0&&d4<=d3)return b;double vc=d1*d4-d3*d2;if(vc<=0&&d1>=0&&d3<=0)return a+ab*(d1/(d1-d3));Vec cp=p-c;double d5=dot(ab,cp),d6=dot(ac,cp);if(d6>=0&&d5<=d6)return c;double vb=d5*d2-d1*d6;if(vb<=0&&d2>=0&&d6<=0)return a+ac*(d2/(d2-d6));double va=d3*d6-d5*d4;if(va<=0&&(d4-d3)>=0&&(d5-d6)>=0)return b+(c-b)*((d4-d3)/((d4-d3)+(d5-d6)));double inv=1/(va+vb+vc);return a+ab*(vb*inv)+ac*(vc*inv);}
// A cap is often a fan of several triangles. A single wound edge can cross
// their shared seams, so no one triangle necessarily contains both endpoints.
// Find its exact parameter interval in each triangle, retaining the existing
// plane-distance tolerance while never filling a gap between separate caps.
bool segmentTriangleInterval(Vec a,Vec b,Vec origin,Vec second,Vec third,double tolerance,std::pair<double,double>&span){
    const Vec u=second-origin,v=third-origin,n=cross(u,v);const double nn=dot(n,n);
    if(nn<1e-28)return false;
    if(std::abs(dot(a-origin,n))>tolerance*std::sqrt(nn)||std::abs(dot(b-origin,n))>tolerance*std::sqrt(nn))return false;
    const double uu=dot(u,u),uv=dot(u,v),vv=dot(v,v),den=uu*vv-uv*uv;if(den<1e-28)return false;
    auto bary=[&](Vec p){Vec q=p-origin;const double qu=dot(q,u),qv=dot(q,v),y=(vv*qu-uv*qv)/den,z=(uu*qv-uv*qu)/den;return std::array<double,3>{1-y-z,y,z};};
    const auto from=bary(a),to=bary(b);double low=0,high=1;
    for(int k=0;k<3;++k){const double d=to[k]-from[k];
        if(std::abs(d)<1e-14){if(from[k]<-1e-10)return false;}
        else if(d>0)low=std::max(low,(-1e-10-from[k])/d);
        else high=std::min(high,(-1e-10-from[k])/d);
    }
    if(low>high)return false;
    // Float skin endpoints can sit just outside an authored opening's rim.
    // Match the single-face closest-point test at the segment endpoints only;
    // keep gaps between separate cap intervals exact.
    if(length(a-closestTriangle(a,origin,second,third))<=tolerance)low=0;
    if(length(b-closestTriangle(b,origin,second,third))<=tolerance)high=1;
    span={low,high};return true;
}
bool coversSegment(std::vector<std::pair<double,double>>spans){
    std::sort(spans.begin(),spans.end());double covered=0;
    for(const auto&span:spans){if(span.first>covered+1e-8)return false;covered=std::max(covered,span.second);if(covered>=1-1e-8)return true;}
    return false;
}
// Patch-independent part of originalTriangleClosureSheets, per envelope and
// skin-vertex fingerprint: for every qualifying fan centre (not an original
// vertex, three incident faces, three rim vertices within tolerance of the rim
// triangle, and a same-key envelope face that cancels the directed rim), the
// keys of its three incident faces, grouped by rim key.
struct ClosureIndex {std::uint64_t key{};std::size_t skinCount{};double tolerance{};
    std::unordered_map<std::array<Quant,3>,std::vector<std::array<std::array<Quant,3>,3>>,QuantArrayHash<3>> rims;};
struct BodyBVH {
    struct Node {Bounds bounds;int left{-1},right{-1};std::size_t first{},count{};};
    const Source&source;std::vector<std::uint32_t>order;std::vector<Node>nodes;
    bool preserveCrossings{};
    // Exact accelerator for inside() on a preserveCrossings envelope: a 2D
    // bucket grid over the triangles projected along the fixed inside()
    // direction. It only culls candidates (padded bounds); every candidate
    // runs the identical halfOpenRay triangle test AND the identical rayBounds
    // test of its leaf and every ancestor, so a crossing counts iff the BVH
    // traversal would have counted it. Built once with the asset envelope.
    struct RayGrid {double sx{},sy{},u0{},v0{},cell{};int nu{},nv{};std::vector<std::uint32_t> first,items;std::vector<int> leafOf,parent;};
    std::shared_ptr<const RayGrid> rayGrid;
    // seedEnvelopeOpenings' rim-weight field over this envelope's vertices. It
    // depends only on the envelope and on the skin vertices (positions and
    // weights), so it is keyed by an exact fingerprint of those and kept in a
    // sparse, bit-exact form (nonzero bone entries per envelope vertex).
    struct SeedField {std::uint64_t key{};std::size_t skinCount{};std::vector<std::uint32_t> first;std::vector<std::pair<std::uint32_t,double>> entries;std::vector<char> valid;};
    std::shared_ptr<std::mutex> seedMutex=std::make_shared<std::mutex>();
    mutable std::shared_ptr<const SeedField> seedField;
    mutable std::shared_ptr<const ClosureIndex> closureIndex;
    static Vec insideRay(){return {.312371,.179319,1};}
    explicit BodyBVH(const Source&s,bool exactCrossings=false):source(s),preserveCrossings(exactCrossings){for(std::uint32_t i=0;i<s.f.size();++i)order.push_back(i);build(0,order.size());if(exactCrossings&&!s.f.empty())buildRayGrid();}
    void buildRayGrid(){
        auto grid=std::make_shared<RayGrid>();auto&r=*grid;const Vec d=insideRay();r.sx=d[0]/d[2];r.sy=d[1]/d[2];
        r.parent.assign(nodes.size(),-1);r.leafOf.assign(source.f.size(),-1);
        for(int n=0;n<int(nodes.size());++n){if(nodes[n].left>=0){r.parent[nodes[n].left]=n;r.parent[nodes[n].right]=n;}
            else for(std::size_t i=nodes[n].first;i<nodes[n].first+nodes[n].count;++i)r.leafOf[order[i]]=n;}
        constexpr double pad=1e-6;double ulo=1e30,uhi=-1e30,vlo=1e30,vhi=-1e30;
        std::vector<std::array<double,4>> box(source.f.size());
        for(std::size_t f=0;f<source.f.size();++f){auto&b=box[f];b={1e30,-1e30,1e30,-1e30};
            for(auto id:source.f[f]){const Vec q=source.v[id].p;const double u=q[0]-r.sx*q[2],v=q[1]-r.sy*q[2];b[0]=std::min(b[0],u);b[1]=std::max(b[1],u);b[2]=std::min(b[2],v);b[3]=std::max(b[3],v);}
            b[0]-=pad;b[1]+=pad;b[2]-=pad;b[3]+=pad;ulo=std::min(ulo,b[0]);uhi=std::max(uhi,b[1]);vlo=std::min(vlo,b[2]);vhi=std::max(vhi,b[3]);}
        r.cell=std::max({.005,(uhi-ulo)/1024,(vhi-vlo)/1024});r.u0=ulo;r.v0=vlo;
        r.nu=int((uhi-ulo)/r.cell)+1;r.nv=int((vhi-vlo)/r.cell)+1;
        auto range=[&](double lo,double hi,double o,int n,int&a,int&b){a=std::clamp(int(std::floor((lo-o)/r.cell)),0,n-1);b=std::clamp(int(std::floor((hi-o)/r.cell)),0,n-1);};
        r.first.assign(std::size_t(r.nu)*r.nv+1,0);
        for(int pass=0;pass<2;++pass){std::vector<std::uint32_t> cursor;if(pass){for(std::size_t c=1;c<r.first.size();++c)r.first[c]+=r.first[c-1];r.items.resize(r.first.back());cursor.assign(r.first.begin(),r.first.end()-1);}
            for(std::uint32_t f=0;f<source.f.size();++f){int ua,ub,va,vb;range(box[f][0],box[f][1],r.u0,r.nu,ua,ub);range(box[f][2],box[f][3],r.v0,r.nv,va,vb);
                for(int u=ua;u<=ub;++u)for(int v=va;v<=vb;++v){const std::size_t c=std::size_t(u)*r.nv+v;if(pass)r.items[cursor[c]++]=f;else ++r.first[c+1];}}}
        rayGrid=std::move(grid);
    }
    // The exact per-triangle crossing test of halfOpenRay.
    bool halfOpenHit(Vec p,Vec d,std::uint32_t face,double&t)const{
        int z=0;for(int k=1;k<3;++k)if(std::abs(d[k])>std::abs(d[z]))z=k;
        const int x=(z+1)%3,y=(z+2)%3;const double sx=d[x]/d[z],sy=d[y]/d[z];
        auto project=[&](Vec q){q=q-p;return Vec{q[x]-sx*q[z],q[y]-sy*q[z],q[z]/d[z]};};
        auto edge=[](Vec a,Vec b){return a.x*b.y-a.y*b.x;};
        auto inclusive=[](Vec a,Vec b){return b.y>a.y || (b.y==a.y&&b.x<a.x);};
        auto&f=source.f[face];
        std::array<Vec,3>v{project(source.v[f[0]].p),project(source.v[f[1]].p),project(source.v[f[2]].p)};
        double area2=edge(v[1]-v[0],v[2]-v[0]);if(std::abs(area2)<1e-18)return false;
        if(area2<0){std::swap(v[1],v[2]);area2=-area2;}
        std::array<double,3>e;bool inside=true;
        for(int k=0;k<3;++k){e[k]=edge(v[k],v[(k+1)%3]);if(e[k]<0 || (e[k]==0&&!inclusive(v[k],v[(k+1)%3])))inside=false;}
        if(!inside)return false;
        t=(e[1]*v[0].z+e[2]*v[1].z+e[0]*v[2].z)/area2;return t>1e-9;
    }
    bool gridInside(Vec p)const{
        const auto&r=*rayGrid;const Vec d=insideRay();
        const double u=p[0]-r.sx*p[2],v=p[1]-r.sy*p[2];
        const double fu=std::floor((u-r.u0)/r.cell),fv=std::floor((v-r.v0)/r.cell);
        if(!(fu>=0&&fv>=0&&fu<r.nu&&fv<r.nv))return false; // no padded triangle bound reaches p
        const std::size_t c=std::size_t(fu)*r.nv+std::size_t(fv);std::size_t crossings=0;
        for(std::size_t i=r.first[c];i<r.first[c+1];++i){const auto face=r.items[i];double t;if(!halfOpenHit(p,d,face,t))continue;
            bool visited=true;for(int n=r.leafOf[face];n>=0&&visited;n=r.parent[n])visited=rayBounds(nodes[n].bounds,p,d);
            crossings+=visited;}
        return crossings%2!=0;
    }
    Bounds faceBounds(std::uint32_t f)const{Bounds b;for(auto id:source.f[f])b.add(source.v[id].p);return b;}
    int build(std::size_t first,std::size_t count){Node n;n.first=first;n.count=count;for(std::size_t i=first;i<first+count;++i)for(auto id:source.f[order[i]])n.bounds.add(source.v[id].p);int index=int(nodes.size());nodes.push_back(n);if(count<=8)return index;int axis=0;for(int a=1;a<3;++a)if(n.bounds.hi[a]-n.bounds.lo[a]>n.bounds.hi[axis]-n.bounds.lo[axis])axis=a;auto center=[&](std::uint32_t f){double v=0;for(auto id:source.f[f])v+=source.v[id].p[axis];return v;};auto middle=first+count/2;std::nth_element(order.begin()+std::ptrdiff_t(first),order.begin()+std::ptrdiff_t(middle),order.begin()+std::ptrdiff_t(first+count),[&](auto a,auto b){return center(a)<center(b);});int left=build(first,count/2),right=build(middle,count-count/2);nodes[index].left=left;nodes[index].right=right;return index;}
    void query(int n,const Bounds&bounds,std::vector<std::uint32_t>&out)const{auto&node=nodes[n];if(!overlapBounds(node.bounds,bounds))return;if(node.left<0){for(std::size_t i=node.first;i<node.first+node.count;++i)if(overlapBounds(faceBounds(order[i]),bounds))out.push_back(order[i]);}else{query(node.left,bounds,out);query(node.right,bounds,out);}}
    bool rayBounds(const Bounds&bounds,Vec p,Vec d)const{double low=0,high=1e9;for(int a=0;a<3;++a){if(std::abs(d[a])<1e-14){if(p[a]<bounds.lo[a]||p[a]>bounds.hi[a])return false;continue;}double t0=(bounds.lo[a]-p[a])/d[a],t1=(bounds.hi[a]-p[a])/d[a];if(t0>t1)std::swap(t0,t1);low=std::max(low,t0);high=std::min(high,t1);if(high<low)return false;}return true;}
    void ray(int n,Vec p,Vec d,std::vector<double>&hits)const{auto&node=nodes[n];if(!rayBounds(node.bounds,p,d))return;if(node.left>=0){ray(node.left,p,d,hits);ray(node.right,p,d,hits);return;}for(std::size_t i=node.first;i<node.first+node.count;++i){auto&f=source.f[order[i]];Vec a=source.v[f[0]].p,e1=source.v[f[1]].p-a,e2=source.v[f[2]].p-a,h=cross(d,e2);double det=dot(e1,h);if(std::abs(det)<1e-14)continue;double inv=1/det;Vec s=p-a;double u=dot(s,h)*inv;if(u<0||u>1)continue;Vec q=cross(s,e1);double v=dot(d,q)*inv;if(v<0||u+v>1)continue;double t=dot(e2,q)*inv;if(t>1e-9)hits.push_back(t);}}
    void halfOpenRay(int n,Vec p,Vec d,std::vector<double>&hits)const{
        auto&node=nodes[n];if(!rayBounds(node.bounds,p,d))return;
        if(node.left>=0){halfOpenRay(node.left,p,d,hits);halfOpenRay(node.right,p,d,hits);return;}
        int z=0;for(int k=1;k<3;++k)if(std::abs(d[k])>std::abs(d[z]))z=k;
        const int x=(z+1)%3,y=(z+2)%3;const double sx=d[x]/d[z],sy=d[y]/d[z];
        auto project=[&](Vec q){q=q-p;return Vec{q[x]-sx*q[z],q[y]-sy*q[z],q[z]/d[z]};};
        auto edge=[](Vec a,Vec b){return a.x*b.y-a.y*b.x;};
        auto inclusive=[](Vec a,Vec b){return b.y>a.y || (b.y==a.y&&b.x<a.x);};
        for(std::size_t i=node.first;i<node.first+node.count;++i){auto&f=source.f[order[i]];
            std::array<Vec,3>v{project(source.v[f[0]].p),project(source.v[f[1]].p),project(source.v[f[2]].p)};
            double area2=edge(v[1]-v[0],v[2]-v[0]);if(std::abs(area2)<1e-18)continue;
            if(area2<0){std::swap(v[1],v[2]);area2=-area2;}
            std::array<double,3>e;bool inside=true;
            for(int k=0;k<3;++k){e[k]=edge(v[k],v[(k+1)%3]);if(e[k]<0 || (e[k]==0&&!inclusive(v[k],v[(k+1)%3])))inside=false;}
            if(!inside)continue;
            double t=(e[1]*v[0].z+e[2]*v[1].z+e[0]*v[2].z)/area2;if(t>1e-9)hits.push_back(t);
        }
    }
    bool insideDirection(Vec p,Vec d)const{
        // Half-open projected edges assign each shared edge to one triangle.
        // Distinct coincident sheets retain BOTH crossings; distance-based
        // deduplication loses multiplicity and changes parity with direction.
        if(preserveCrossings){
            if(rayGrid){const Vec fixed=insideRay();if(d.x==fixed.x&&d.y==fixed.y&&d.z==fixed.z){
#ifdef GFY_CS_VERIFY
                std::vector<double>check;halfOpenRay(0,p,d,check);if((check.size()%2!=0)!=gridInside(p))throw std::runtime_error("verify_grid_inside_mismatch");
#endif
                return gridInside(p);}}
            std::vector<double>hits;halfOpenRay(0,p,d,hits);return hits.size()%2!=0;}
        std::vector<double>hits;ray(0,p,d,hits);std::sort(hits.begin(),hits.end());std::size_t unique=0;double previous=-1;for(double t:hits)if(t-previous>1e-8){++unique;previous=t;}return unique%2!=0;
    }
    bool inside(Vec p)const{return insideDirection(p,{.312371,.179319,1});}
    std::vector<double> crossingDistances(Vec p,Vec d)const{
        std::vector<double>hits;
        if(preserveCrossings)halfOpenRay(0,p,d,hits);else ray(0,p,d,hits);
        std::sort(hits.begin(),hits.end());std::vector<double>out;
        for(std::size_t i=0;i<hits.size();){std::size_t end=i+1;while(end<hits.size()&&hits[end]-hits[i]<1e-8)++end;
            if(!preserveCrossings || (end-i)%2)out.push_back(hits[i]);i=end;
        }return out;
    }
    double lowerDistance(const Bounds&b,Vec p)const{double d=0;for(int a=0;a<3;++a){double x=std::max({b.lo[a]-p[a],p[a]-b.hi[a],0.});d+=x*x;}return d;}
    void nearest(int n,Vec p,double&best,std::uint32_t&face,Vec&point)const{auto&node=nodes[n];if(lowerDistance(node.bounds,p)>best)return;if(node.left>=0){int a=node.left,b=node.right;if(lowerDistance(nodes[a].bounds,p)>lowerDistance(nodes[b].bounds,p))std::swap(a,b);nearest(a,p,best,face,point);nearest(b,p,best,face,point);return;}for(std::size_t i=node.first;i<node.first+node.count;++i){auto id=order[i];auto&f=source.f[id];Vec q=closestTriangle(p,source.v[f[0]].p,source.v[f[1]].p,source.v[f[2]].p);double dd=dot(p-q,p-q);if(dd<best){best=dd;face=id;point=q;}}}
};
Poly planeClip(const Poly&p,Vec normal,double bound,double sign){Poly out;if(p.empty())return out;auto append=[&](V v){if(out.empty()||quantExact(out.back().p)!=quantExact(v.p))out.push_back(v);};for(std::size_t i=0;i<p.size();++i){auto a=p[i],b=p[(i+1)%p.size()];double da=(dot(a.p,normal)-bound)*sign,db=(dot(b.p,normal)-bound)*sign;bool ia=da>=-1e-12,ib=db>=-1e-12;if(ia)append(a);if(ia!=ib){double t=da/(da-db);V v;v.material=a.material;v.p=a.p+(b.p-a.p)*t;for(int k=0;k<3;++k)v.b[k]=a.b[k]+(b.b[k]-a.b[k])*t;append(v);}}if(out.size()>1&&quantExact(out.front().p)==quantExact(out.back().p))out.pop_back();if(out.size()<3||area(out)<1e-16)out.clear();return out;}
// Snapped plane clipping can produce A-B-A backtracks. They bound no
// area and have two oppositely directed copies of the same edge. Cancel only
// these exact lattice identities; never use a distance/area relaxation here.
void cancelCavityBacktracks(Poly& poly){
    bool changed=true;
    while(changed&&poly.size()>=3){
        changed=false;
        for(std::size_t i=0;i<poly.size();++i){
            const auto previous=(i+poly.size()-1)%poly.size();
            const auto next=(i+1)%poly.size();
            if(quantExact(poly[previous].p)==quantExact(poly[i].p)||
               quantExact(poly[previous].p)==quantExact(poly[next].p)){
                poly.erase(poly.begin()+std::ptrdiff_t(i));changed=true;break;
            }
        }
    }
}
// pitch>0 (tile-local joins): a quad whose envelope query box, grown one cell into its removed cells, meets
// no envelope face lies wholly inside the body (its removed cells were body cells): kept without clipping
// or the inside() ray test.
std::vector<Poly>cavityPolygons(const BodyBVH&body,const std::vector<Quad>&quads,double pitch=0){
    // Each quad's pieces are a pure function of the quad and the envelope;
    // chunks of quads run concurrently and are concatenated in quad order.
    std::vector<std::vector<Poly>> parts(parallelWidth());
    parallelChunks(quads.size(),1024,[&](std::size_t first,std::size_t last,std::size_t chunk){auto&output=parts[chunk];
    for(std::size_t qi=first;qi<last;++qi){const auto&q=quads[qi];Bounds qb;for(auto p:q.p)qb.add(p);std::vector<std::uint32_t>faces;body.query(0,qb,faces);Poly base;for(auto p:q.p)base.push_back({p,{},q.material});if(pitch>0&&faces.empty()){int ax=0;for(int a=1;a<3;++a)if(qb.hi[a]-qb.lo[a]<qb.hi[ax]-qb.lo[ax])ax=a;const Vec qn=cross(q.p[1]-q.p[0],q.p[2]-q.p[0]);Bounds deep=qb;if(qn[ax]>0)deep.lo[ax]-=pitch;else deep.hi[ax]+=pitch;std::vector<std::uint32_t>near;body.query(0,deep,near);if(near.empty()){Poly poly=base;std::reverse(poly.begin(),poly.end());for(auto&v:poly)v.p=snap(v.p);cancelCavityBacktracks(poly);if(poly.size()>=3)output.push_back(std::move(poly));continue;}}std::vector<Poly>pieces{base};int axis=0;for(int a=1;a<3;++a)if(qb.hi[a]-qb.lo[a]<qb.hi[axis]-qb.lo[axis])axis=a;double plane=qb.lo[axis];
        for(auto face:faces){auto&f=body.source.f[face];Vec points[6];int count=0;for(int k=0;k<3;++k){Vec a=body.source.v[f[k]].p,b=body.source.v[f[(k+1)%3]].p;double da=a[axis]-plane,db=b[axis]-plane;if(std::abs(da)<1e-12)points[count++]=a;if((da>1e-12&&db<-1e-12)||(da<-1e-12&&db>1e-12))points[count++]=a+(b-a)*(da/(da-db));}if(count<2)continue;Bounds line;for(int k=0;k<count;++k)line.add(points[k]);if(!overlapBounds(line,qb))continue;Vec a=body.source.v[f[0]].p,b=body.source.v[f[1]].p,c=body.source.v[f[2]].p,n=cross(b-a,c-a);double norm=length(n);if(norm<1e-14)continue;n=n*(1/norm);double bound=dot(a,n);std::vector<Poly>next;
            for(auto&poly:pieces){
                double low=1e30,high=-1e30;for(const auto&v:poly){double distance=dot(v.p,n)-bound;low=std::min(low,distance);high=std::max(high,distance);}
                // A coincident/tangent plane does not partition a polygon.
                // Clipping both tolerant half-spaces in that case duplicates
                // the whole polygon and can leave mismatched later splits.
                if(low>=-1e-12 || high<=1e-12){next.push_back(std::move(poly));continue;}
                auto positive=planeClip(poly,n,bound,1),negative=planeClip(poly,n,bound,-1);if(!positive.empty())next.push_back(std::move(positive));if(!negative.empty())next.push_back(std::move(negative));}pieces=std::move(next);}
        for(auto&poly:pieces){Vec center;for(auto&v:poly)center=center+v.p;center=center*(1./double(poly.size()));if(!body.inside(center))continue;std::reverse(poly.begin(),poly.end());for(auto&v:poly)v.p=snap(v.p);cancelCavityBacktracks(poly);if(poly.size()>=3)output.push_back(std::move(poly));}}});
    std::vector<Poly>output;std::size_t total=0;for(auto&part:parts)total+=part.size();output.reserve(total);
    for(auto&part:parts)for(auto&poly:part)output.push_back(std::move(poly));
    return output;
}
struct JoinedVertex {Vec p;Vec normal,tangent,binormal;std::uint32_t sourceFace{~std::uint32_t(0)};std::array<double,3>sourceBary{};std::array<double,2>uv{};std::array<double,64>w{};bool hardware_corner{};std::array<std::uint16_t,4>hardware_ids{};std::array<float,4>hardware_w{};};
struct JoinedPolygon {unsigned material{};std::vector<JoinedVertex>v;};
struct JoinedTriangle {unsigned material{};std::array<JoinedVertex,3>v;};
inline const std::array<double,64>& skinRow(const std::array<double,64>& w){return w;}
inline const std::array<double,64>& skinRow(const std::array<double,64>* w){return *w;}
template<class Edges,class Skin>bool nativeEdgeCovered(const Edge&,const Edges&,const Skin&,double);
// Plane partitions can round a collinear cavity sliver into a tiny triangle.
// Remove only dangling spur components attached to an already covered edge.
// A polygon fan may make several slivers sharing internal edges. Validate the
// component's complete perimeter; never delete a thin gap-closing surface.
using EdgeCountMap=FlatMap<Edge,unsigned,QuantArrayHash<2>>;
// A triangle whose corners point into a stable vertex store (fan triangles of
// one polygon share its vertices instead of copying 656-byte records).
struct IndexedTriangle {unsigned material{};std::array<JoinedVertex*,3> v{};};
inline const JoinedVertex& vx(const JoinedTriangle&t,int k){return t.v[k];}
inline const JoinedVertex& vx(const IndexedTriangle&t,int k){return *t.v[k];}
// sharedCounts (optional) receives the edge counts of the RETAINED triangles
// (removed triangles are decremented; zero entries may remain).
template<class Triangle>std::size_t removeFloatCavitySpurs(std::vector<Triangle>& triangles,double tolerance,
                                   bool allowNumericalFallback,EdgeCountMap* sharedCounts=nullptr){
    // counts/thinEdges are lookup-only (the one full scan below feeds an
    // ordered std::map), so hashing cannot change any decision order.
    EdgeCountMap localCounts;auto& counts=sharedCounts?*sharedCounts:localCounts;counts.clear();counts.reserve(triangles.size()*2);
    for(const auto&t:triangles)for(int k=0;k<3;++k)++counts[edgeKeyExact(vx(t,k).p,vx(t,(k+1)%3).p)];
    std::vector<bool> remove(triangles.size()),candidate(triangles.size()),visited(triangles.size());std::size_t removed=0;
    FlatMap<Edge,std::vector<std::size_t>,QuantArrayHash<2>> thinEdges;
    // Triangles are not modified until the final compaction, so the first-wins
    // skin map is identical for every fallback component: build it once.
    std::map<Quant,std::array<double,64>> skin;bool skinReady=false;
    for(std::size_t i=0;i<triangles.size();++i){const auto&t=triangles[i];if(!t.material)continue;
        std::array<Edge,3> edges;std::array<double,3> lengths;
        for(int k=0;k<3;++k){edges[k]=edgeKeyExact(vx(t,k).p,vx(t,(k+1)%3).p);lengths[k]=length(vx(t,k).p-vx(t,(k+1)%3).p);}
        const int longest=int(std::max_element(lengths.begin(),lengths.end())-lengths.begin());
        if(lengths[longest]<=0)continue;
        const double altitude=length(cross(vx(t,1).p-vx(t,0).p,vx(t,2).p-vx(t,0).p))/lengths[longest];
        if(altitude>tolerance)continue;
        candidate[i]=true;for(const auto&edge:edges)thinEdges[edge].push_back(i);
    }
    for(std::size_t root=0;root<triangles.size();++root){if(!candidate[root]||visited[root])continue;
        std::vector<std::size_t> component{root};visited[root]=true;std::map<Edge,unsigned> removedIncidence;
        for(std::size_t at=0;at<component.size();++at){const auto&t=triangles[component[at]];
            for(int k=0;k<3;++k){const auto edge=edgeKeyExact(vx(t,k).p,vx(t,(k+1)%3).p);++removedIncidence[edge];
                for(auto next:thinEdges.at(edge))if(!visited[next]){visited[next]=true;component.push_back(next);}}}
        bool crowded=false;for(const auto&item:removedIncidence)crowded=crowded||counts[item.first]>2;
        if(!crowded||component.size()>128)continue;
        // Some thin fan triangles really close a seam while adjacent ones are
        // redundant. Solve their edge incidence rather than deleting the whole
        // component. Non-thin faces are fixed and no new singly-used edge may
        // be introduced. The bounded search prefers retaining geometry.
        struct Constraint {unsigned fixed;std::vector<unsigned> variables;};
        std::map<std::size_t,unsigned> local;for(unsigned i=0;i<component.size();++i)local[component[i]]=i;
        std::vector<Constraint> constraints;
        for(const auto&item:removedIncidence){Constraint c;c.fixed=counts[item.first]-item.second;
            for(auto global:thinEdges.at(item.first))c.variables.push_back(local.at(global));constraints.push_back(std::move(c));}
        unsigned visits=0;std::vector<int> solution;
        auto solve=[&](auto&&self,std::vector<int> state)->bool{
            if(++visits>4096)return false;
            bool changed=true;
            while(changed){changed=false;for(const auto&c:constraints){unsigned on=0,free=0;for(auto id:c.variables){on+=state[id]==1;free+=state[id]<0;}
                unsigned minimum=~0u,maximum=0;
                for(unsigned total:{0u,1u,2u}){bool allowed=c.fixed>=2?total==0:c.fixed==1?total==1:(total==0||total==2);
                    if(allowed&&total>=on&&total<=on+free){minimum=std::min(minimum,total);maximum=std::max(maximum,total);}}
                if(minimum==~0u)return false;
                if(free&&(maximum==on||minimum==on+free)){int value=minimum==on+free?1:0;for(auto id:c.variables)if(state[id]<0){state[id]=value;changed=true;}}
            }}
            auto unknown=std::find(state.begin(),state.end(),-1);if(unknown==state.end()){solution=std::move(state);return true;}
            auto index=std::size_t(unknown-state.begin());state[index]=1;if(self(self,state))return true;state[index]=0;return self(self,std::move(state));
        };
        if(!solve(solve,std::vector<int>(component.size(),-1))){
            // Near-collinear float edges can be covered by several shorter
            // segments without having identical quantized keys. The exact
            // incidence solver cannot express that T-junction. Search only a
            // tiny sliver component, retaining as much geometry as possible;
            // every changed singly-used edge must have full skin-compatible
            // coverage. Real gaps and substantial surfaces still refuse.
            if(allowNumericalFallback&&component.size()<=12){
                Bounds localBounds;for(auto id:component)for(int k=0;k<3;++k)localBounds.add(vx(triangles[id],k).p);
                const Vec pad{1e-6,1e-6,1e-6};localBounds.lo=localBounds.lo-pad;localBounds.hi=localBounds.hi+pad;
                auto inside=[&](Vec p){for(int a=0;a<3;++a)if(p[a]<localBounds.lo[a]||p[a]>localBounds.hi[a])return false;return true;};
                auto position=[](Quant q){return Vec{q[0]*1e-8,q[1]*1e-8,q[2]*1e-8};};
                std::map<Edge,unsigned> localCounts;
                for(const auto&edge:counts)if(inside(position(edge.first[0]))&&inside(position(edge.first[1])))
                    localCounts.emplace(edge.first,edge.second);
                if(!skinReady){for(const auto&t:triangles)for(int corner=0;corner<3;++corner)if(const auto&v=vx(t,corner);true)
                    skin.try_emplace(quantExact(v.p),v.w);skinReady=true;}
                const unsigned combinations=1u<<component.size();unsigned bestKept=0;
#ifdef GFY_CUT_DIAGNOSTICS
                unsigned refusedArea=0,refusedCounts=0,refusedCoverage=0;
#endif
                for(unsigned mask=0;mask<combinations;++mask){
                    unsigned kept=0;for(unsigned bits=mask;bits;bits>>=1)kept+=bits&1u;
                    if(kept<bestKept||kept==component.size())continue;
                    auto proposed=localCounts;double areaRemoved=0;bool valid=true;
                    for(unsigned i=0;i<component.size();++i)if(!(mask&(1u<<i))){
                        const auto&t=triangles[component[i]];
                        areaRemoved+=.5*length(cross(vx(t,1).p-vx(t,0).p,vx(t,2).p-vx(t,0).p));
                        for(int k=0;k<3;++k){const auto edge=edgeKeyExact(vx(t,k).p,vx(t,(k+1)%3).p);
                            auto found=proposed.find(edge);if(found==proposed.end()||!found->second){valid=false;break;}
                            if(!--found->second)proposed.erase(found);}
                    }
                    if(!valid||areaRemoved>1e-7){
#ifdef GFY_CUT_DIAGNOSTICS
                        ++refusedArea;
#endif
                        continue;}
                    for(const auto&edge:removedIncidence){const auto found=proposed.find(edge.first);
                        const unsigned remaining=found==proposed.end()?0:found->second;
                        if(remaining>2){
#ifdef GFY_CUT_DIAGNOSTICS
                            ++refusedCounts;
#endif
                            valid=false;break;}
                        if(remaining==1&&counts.at(edge.first)!=1&&
                              !nativeEdgeCovered(edge.first,proposed,skin,1e-6)){
#ifdef GFY_CUT_DIAGNOSTICS
                            ++refusedCoverage;
#endif
                            valid=false;break;}}
                    if(valid&&kept>=bestKept){bestKept=kept;solution.assign(component.size(),0);
                        for(unsigned i=0;i<component.size();++i)solution[i]=bool(mask&(1u<<i));}
                }
#ifdef GFY_CUT_DIAGNOSTICS
                if(solution.empty()&&localBounds.lo.x<-.1869&&localBounds.hi.x>-.1871)
                    std::cout<<"thin_search_refusals "<<component.size()<<' '<<refusedArea<<' '
                             <<refusedCounts<<' '<<refusedCoverage<<'\n';
#endif
            }
#ifdef GFY_CUT_DIAGNOSTICS
            if(!solution.empty())std::cout<<"thin_component_repaired "<<component.size()<<' '
                                          <<std::count(solution.begin(),solution.end(),0)<<'\n';
#endif
            if(solution.empty()){
#ifdef GFY_CUT_DIAGNOSTICS
            Bounds b;for(auto id:component)for(int k=0;k<3;++k)b.add(vx(triangles[id],k).p);
            if(b.lo.x<-.1869&&b.hi.x>-.1871&&b.lo.y<.82&&b.hi.y>.815)
                std::cout<<"thin_component_refused "<<component.size()<<' '<<constraints.size()<<' '<<visits
                         <<' '<<b.lo.x<<' '<<b.lo.y<<' '<<b.lo.z<<' '<<b.hi.x<<' '<<b.hi.y<<' '<<b.hi.z<<'\n';
#endif
                continue;}}
        for(unsigned i=0;i<component.size();++i)if(!solution[i]){auto id=component[i];remove[id]=true;++removed;
            const auto&t=triangles[id];for(int k=0;k<3;++k)--counts[edgeKeyExact(vx(t,k).p,vx(t,(k+1)%3).p)];}
    }
    std::size_t write=0;for(std::size_t i=0;i<triangles.size();++i)if(!remove[i])triangles[write++]=std::move(triangles[i]);
    triangles.resize(write);return removed;
}
// Edge multiplicity alone cannot recognize a float T junction: one long
// edge may meet several shorter edges without any physical opening. Require
// full segment coverage and matching skin interpolation before accepting it.
// Order of `edges` is irrelevant: spans are sorted before the coverage walk.
template<class Edges,class Skin>bool nativeEdgeCovered(const Edge&target,const Edges&edges,const Skin&skin,double tolerance){
    auto position=[](Quant q){return Vec{q[0]*1e-8,q[1]*1e-8,q[2]*1e-8};};
    Vec a=position(target[0]),d=position(target[1])-a;double dd=dot(d,d);if(dd<=1e-20)return false;
    // Sub-tolerance edges cannot be proven closed by a permissive interval
    // margin: a tiny partial overlap could otherwise stand for the whole edge.
    const double margin=std::min(.01,tolerance/std::sqrt(dd));std::vector<std::pair<double,double>>spans;
    for(const auto&item:edges){if(item.first==target)continue;
        Vec p=position(item.first[0]),q=position(item.first[1]);double u=dot(p-a,d)/dd,v=dot(q-a,d)/dd;
        if(u>v){std::swap(u,v);std::swap(p,q);}if(v<0||u>1||v-u<=1e-12)continue;
        if(length(p-(a+d*u))>tolerance||length(q-(a+d*v))>tolerance)continue;
        auto kp=quantExact(p),kq=quantExact(q);double lo=std::max(0.,u),hi=std::min(1.,v);bool matches=true;
        for(double t:{lo,hi}){double fraction=(t-u)/(v-u);
            const auto&s0=skinRow(skin.at(target[0]));const auto&s1=skinRow(skin.at(target[1]));const auto&sp=skinRow(skin.at(kp));const auto&sq=skinRow(skin.at(kq));
            for(unsigned bone=0;bone<64;++bone){double own=s0[bone]*(1-t)+s1[bone]*t;
                double other=sp[bone]*(1-fraction)+sq[bone]*fraction;
                // Match homogeneous LBS contributions, not only weights.
                // A rest-collinear subdivision can separate when bent if
                // weight*position interpolation differs across the seam.
                Vec ownPosition=a*(s0[bone]*(1-t))+(a+d)*(s1[bone]*t);
                Vec otherPosition=p*(sp[bone]*(1-fraction))+q*(sq[bone]*fraction);
                if(std::abs(own-other)>1e-6||length(ownPosition-otherPosition)>tolerance){matches=false;break;}}}
        if(matches&&hi>lo)spans.emplace_back(lo,hi);
    }
    std::sort(spans.begin(),spans.end());double end=0;
    for(const auto&span:spans){if(span.first>end+margin)return false;end=std::max(end,span.second);if(end>=1-margin)return true;}
    return false;
}
// sharedCounts (optional): already the exact edge counts of `triangles`.
template<class Triangle>std::size_t removeSubmicronSpurs(std::vector<Triangle>&triangles,double tolerance,EdgeCountMap* sharedCounts=nullptr){
    EdgeCountMap localCounts;if(!sharedCounts){localCounts.reserve(triangles.size()*2);for(const auto&t:triangles)for(int k=0;k<3;++k)++localCounts[edgeKeyExact(vx(t,k).p,vx(t,(k+1)%3).p)];}
    auto& counts=sharedCounts?*sharedCounts:localCounts;
    // Only edges used more than twice can make a triangle crowded. Mirror
    // their counts in a small table (kept in step with every decrement) so the
    // per-triangle test never touches the full edge table.
    EdgeCountMap crowdedCounts;for(const auto&e:counts)if(e.second>2)crowdedCounts.try_emplace(e.first,e.second);
    auto crowdedEdge=[&](const Edge&e){auto it=crowdedCounts.find(e);return it!=crowdedCounts.end()&&it->second>2;};
    // Compact in place: triangle i is read before any slot >= i is written.
    std::size_t write=0,removed=0;
    for(std::size_t index=0;index<triangles.size();++index){const auto&t=triangles[index];bool redundant=false;if(t.material){
        int longest=0;double lengths[3];for(int k=0;k<3;++k){lengths[k]=length(vx(t,k).p-vx(t,(k+1)%3).p);if(lengths[k]>lengths[longest])longest=k;}
        bool crowded=false;if(lengths[longest]>0&&lengths[longest]<=.005&&!crowdedCounts.empty())for(int k=0;k<3;++k)crowded=crowded||crowdedEdge(edgeKeyExact(vx(t,k).p,vx(t,(k+1)%3).p));
        if(lengths[longest]>0&&lengths[longest]<=.005&&crowded){
            double altitude=length(cross(vx(t,1).p-vx(t,0).p,vx(t,2).p-vx(t,0).p))/lengths[longest];redundant=altitude<=tolerance;
            // A rest-pose sliver can carry a real skinning transition. Removing
            // it replaces two edges with the longest edge: require matching
            // homogeneous LBS coefficients, not just near-collinear positions.
            if(redundant){
                const auto&a=vx(t,longest);const auto&b=vx(t,(longest+1)%3);const auto&c=vx(t,(longest+2)%3);
                const Vec d=b.p-a.p;const double f=dot(c.p-a.p,d)/dot(d,d);
                redundant=f>=0&&f<=1;
                for(std::size_t bone=0;redundant&&bone<a.w.size();++bone){
                    const double expectedWeight=a.w[bone]*(1-f)+b.w[bone]*f;
                    const Vec expectedPosition=a.p*(a.w[bone]*(1-f))+b.p*(b.w[bone]*f);
                    redundant=std::abs(c.w[bone]-expectedWeight)<=1e-6&&length(c.p*c.w[bone]-expectedPosition)<=tolerance;
                }
            }
        }}
        if(redundant){++removed;for(int k=0;k<3;++k){const auto e=edgeKeyExact(vx(t,k).p,vx(t,(k+1)%3).p);--counts[e];auto small=crowdedCounts.find(e);if(small!=crowdedCounts.end())--small->second;}}
        else{if(write!=index)triangles[write]=std::move(triangles[index]);++write;}
    }
    // A subsequent geometric coverage gate must still close every new edge.
    triangles.resize(write);return removed;
}
// Authored clothing is a collection of sheets, not necessarily the boundary
// of a single solid. A sheet wholly inside the query solid may meet a sound
// cavity at a T junction. Require exactly one original sheet and two coplanar
// cavity faces on opposite sides, and prove BOTH sides of the original sheet
// are already interior. This is not an exemption for a new overlapping face.
bool inheritedInteriorSheet(const Edge&edge,const std::vector<const JoinedTriangle*>&faces,const BodyBVH&body,double pitch,bool certifiedClosure=false){
    if(faces.size()!=3)return false;
    const JoinedTriangle*sheet=nullptr;std::vector<const JoinedTriangle*>cavity;
    for(auto*t:faces){if(t->material)cavity.push_back(t);else{if(sheet)return false;sheet=t;}}
    if(!sheet||cavity.size()!=2||sheet->v[0].sourceFace==~std::uint32_t(0))return false;
    for(int k=1;k<3;++k)if(sheet->v[k].sourceFace!=sheet->v[0].sourceFace)return false;
    // Use the actual float edge for plane tests. Reconstructing an endpoint
    // from its rounded hash key tilts the normal of a very thin triangle.
    Vec a,b;bool foundA=false,foundB=false;
    for(const auto&v:cavity[0]->v){if(quantExact(v.p)==edge[0]){a=v.p;foundA=true;}if(quantExact(v.p)==edge[1]){b=v.p;foundB=true;}}
    if(!foundA||!foundB)return false;const Vec d=b-a;
    if(length(d)<4e-6)return false;
    Vec sides[2];
    for(int n=0;n<2;++n){bool found=false;
        for(const auto&v:cavity[n]->v)if(quantExact(v.p)!=edge[0]&&quantExact(v.p)!=edge[1]){sides[n]=cross(d,v.p-a);found=true;break;}
        if(!found||length(sides[n])<1e-14)return false;
        sides[n]=sides[n]*(1/length(sides[n]));
    }
    if(dot(sides[0],sides[1])>-.999999)return false;
    Vec normal=cross(sheet->v[1].p-sheet->v[0].p,sheet->v[2].p-sheet->v[0].p);
    if(length(normal)<1e-14)return false;normal=normal*(1/length(normal));
    // A separately certified original sheet and its opposite query cap enclose
    // no tissue. Its intentional cut rim may meet both sides of the cavity.
    // Keep the same strict one-source/two-opposing-cavity incidence above.
    if(certifiedClosure)return true;
    // Do not step farther than the local edge when two authored clothing
    // sheets lie very close together; that samples a different compartment.
    const Vec center=(a+b)*.5;const double offset=std::max(1e-6,std::min({pitch*.1,5e-5,length(d)*.1}));
    for(double sign:{-1.,1.})for(Vec ray:std::vector<Vec>{{.312371,.179319,1},{1,.37,.53},{-.73,1,.39}})
        if(!body.insideDirection(center+normal*(offset*sign),ray))return false;
    return true;
}
using SurfaceFaceKey=std::array<Quant,3>;
SurfaceFaceKey surfaceFaceKey(const Source&s,const std::array<std::uint32_t,3>&face){
    SurfaceFaceKey key{quantExact(s.v[face[0]].p),quantExact(s.v[face[1]].p),quantExact(s.v[face[2]].p)};std::sort(key.begin(),key.end());return key;
}
std::set<SurfaceFaceKey> originalTriangleClosureSheets(const Source&skin,const Source&envelope,double tolerance){
    std::set<Quant>originalVertices;for(const auto&v:skin.v)originalVertices.insert(quantExact(v.p));
    std::set<SurfaceFaceKey>originalFaces;for(const auto&f:skin.f)originalFaces.insert(surfaceFaceKey(skin,f));
    std::map<SurfaceFaceKey,std::vector<unsigned>>sourceFaces;
    std::vector<std::vector<unsigned>>incident(envelope.v.size());
    for(unsigned i=0;i<envelope.f.size();++i){for(auto v:envelope.f[i])incident[v].push_back(i);auto key=surfaceFaceKey(envelope,envelope.f[i]);if(originalFaces.count(key))sourceFaces[key].push_back(i);}
    std::set<SurfaceFaceKey>result;
    for(unsigned center=0;center<envelope.v.size();++center){
        if(originalVertices.count(quantExact(envelope.v[center].p))||incident[center].size()!=3)continue;
        std::set<unsigned>rim;bool newFaces=true;
        for(auto face:incident[center]){newFaces=newFaces&&!originalFaces.count(surfaceFaceKey(envelope,envelope.f[face]));for(auto v:envelope.f[face])if(v!=center)rim.insert(v);}
        if(!newFaces||rim.size()!=3)continue;
        std::array<std::uint32_t,3>triangle{};unsigned at=0;for(auto v:rim)triangle[at++]=v;
        const auto key=surfaceFaceKey(envelope,triangle);auto found=sourceFaces.find(key);if(found==sourceFaces.end())continue;
        const Vec p=envelope.v[center].p;
        if(length(p-closestTriangle(p,envelope.v[triangle[0]].p,envelope.v[triangle[1]].p,envelope.v[triangle[2]].p))>tolerance)continue;
        // The fan must cancel the exact directed rim of an original triangle.
        // Coincident positions alone cannot certify unrelated query geometry.
        std::map<Edge,int>boundary;
        auto add=[&](std::map<Edge,int>&edges,unsigned a,unsigned b){const auto qa=quantExact(envelope.v[a].p),qb=quantExact(envelope.v[b].p);if(qa<qb)++edges[{qa,qb}];else --edges[{qb,qa}];};
        for(auto face:incident[center])for(unsigned k=0;k<3;++k){const auto&f=envelope.f[face];if(f[k]!=center&&f[(k+1)%3]!=center)add(boundary,f[k],f[(k+1)%3]);}
        for(auto face:found->second){auto proof=boundary;const auto&f=envelope.f[face];for(unsigned k=0;k<3;++k)add(proof,f[k],f[(k+1)%3]);bool cancels=true;for(const auto&e:proof)cancels=cancels&&e.second==0;if(cancels){result.insert(key);break;}}
    }
    return result;
}
inline std::uint64_t skinFingerprint(const Source&skin){
    std::uint64_t key=skin.v.size();
    for(const auto&v:skin.v){std::uint64_t words[4];std::memcpy(words,&v.p,24);std::memcpy(&words[3],v.ids.data(),8);for(auto w:words)key=quantMix(key^w);
        std::uint64_t w0,w1;std::memcpy(&w0,v.w.data(),8);std::memcpy(&w1,v.w.data()+2,8);key=quantMix(key^w0);key=quantMix(key^w1);}
    return key;
}
inline std::shared_ptr<const ClosureIndex> envelopeClosureIndex(const Source&skin,const BodyBVH&body,double tolerance){
    const auto key=skinFingerprint(skin);const auto&envelope=body.source;
    std::lock_guard<std::mutex> lock(*body.seedMutex);
    if(body.closureIndex&&body.closureIndex->key==key&&body.closureIndex->skinCount==skin.v.size()&&body.closureIndex->tolerance==tolerance)return body.closureIndex;
    auto out=std::make_shared<ClosureIndex>();out->key=key;out->skinCount=skin.v.size();out->tolerance=tolerance;
    std::unordered_map<Quant,char,QuantHash>originalVertices;for(const auto&v:skin.v)originalVertices.emplace(quantExact(v.p),0);
    std::unordered_map<SurfaceFaceKey,std::vector<unsigned>,QuantArrayHash<3>>facesByKey;
    std::vector<std::vector<unsigned>>incident(envelope.v.size());
    for(unsigned i=0;i<envelope.f.size();++i){for(auto v:envelope.f[i])incident[v].push_back(i);facesByKey[surfaceFaceKey(envelope,envelope.f[i])].push_back(i);}
    for(unsigned center=0;center<envelope.v.size();++center){
        if(originalVertices.count(quantExact(envelope.v[center].p))||incident[center].size()!=3)continue;
        std::set<unsigned>rim;std::array<SurfaceFaceKey,3> incidentKeys;unsigned n=0;
        for(auto face:incident[center]){incidentKeys[n++]=surfaceFaceKey(envelope,envelope.f[face]);for(auto v:envelope.f[face])if(v!=center)rim.insert(v);}
        if(rim.size()!=3)continue;
        std::array<std::uint32_t,3>triangle{};unsigned at=0;for(auto v:rim)triangle[at++]=v;
        const auto rimKey=surfaceFaceKey(envelope,triangle);auto found=facesByKey.find(rimKey);if(found==facesByKey.end())continue;
        const Vec p=envelope.v[center].p;
        if(length(p-closestTriangle(p,envelope.v[triangle[0]].p,envelope.v[triangle[1]].p,envelope.v[triangle[2]].p))>tolerance)continue;
        std::map<Edge,int>boundary;
        auto add=[&](std::map<Edge,int>&edges,unsigned a,unsigned b){const auto qa=quantExact(envelope.v[a].p),qb=quantExact(envelope.v[b].p);if(qa<qb)++edges[{qa,qb}];else --edges[{qb,qa}];};
        for(auto face:incident[center])for(unsigned k=0;k<3;++k){const auto&f=envelope.f[face];if(f[k]!=center&&f[(k+1)%3]!=center)add(boundary,f[k],f[(k+1)%3]);}
        bool cancelled=false;
        for(auto face:found->second){auto proof=boundary;const auto&f=envelope.f[face];for(unsigned k=0;k<3;++k)add(proof,f[k],f[(k+1)%3]);bool cancels=true;for(const auto&e:proof)cancels=cancels&&e.second==0;if(cancels){cancelled=true;break;}}
        if(cancelled)out->rims[rimKey].push_back(incidentKeys);
    }
    body.closureIndex=out;return out;
}
// Exactly originalTriangleClosureSheets(skin,envelope,tolerance).count(face)
// for a face key of the skin patch; patchKeys is the set of patch face keys.
template<class PatchKeys>bool closureSheetMember(const ClosureIndex&index,const PatchKeys&patchKeys,const SurfaceFaceKey&face){
    if(!patchKeys.count(face))return false;auto found=index.rims.find(face);if(found==index.rims.end())return false;
    for(const auto&keys:found->second){bool fresh=true;for(const auto&k:keys)fresh=fresh&&!patchKeys.count(k);if(fresh)return true;}
    return false;
}
struct JoinResult {std::size_t rimTJunctionsRepaired{};std::vector<JoinedTriangle>triangles;std::size_t originalOpen{},newOpen{},nonmanifold{},cavityTriangles{},cavityPolygons{},numericalSeams{},numericalSpurs{},numericalOpenEdges{},interiorSheetJunctions{};double cutPerimeter{},maxJoinWeightConflict{},numericalOpenLength{};std::size_t earClippedFragments{},nonmanifoldCavityDoubled{};};
template<class Skin>bool recordNumericalOpening(const Edge&edge,const Skin&skin,unsigned bones,JoinResult&result){
    if(result.numericalOpenEdges>=16)return false;
    auto position=[](Quant q){return Vec{q[0]*1e-8,q[1]*1e-8,q[2]*1e-8};};
    const double distance=length(position(edge[1])-position(edge[0]));
    if(distance>1e-6||result.numericalOpenLength+distance>1e-5)return false;
    double difference=0;for(unsigned bone=0;bone<bones;++bone)difference+=std::abs(skinRow(skin.at(edge[0]))[bone]-skinRow(skin.at(edge[1]))[bone]);
    if(difference>1e-6)return false;
    ++result.numericalOpenEdges;result.numericalOpenLength+=distance;return true;
}
JoinedVertex interpolateVertex(const Source&s,std::uint32_t face,const V&v){JoinedVertex out;out.p=v.p;out.w=weights(s,face,v);out.sourceFace=face;out.sourceBary=v.b;for(unsigned k=0;k<3;++k)if(v.b[k]==1&&v.b[(k+1)%3]==0&&v.b[(k+2)%3]==0){const auto&src=s.v[s.f[face][k]];if(src.authored_hardware){out.hardware_corner=true;out.hardware_ids=src.ids;out.hardware_w=src.w;}}for(int k=0;k<3;++k){const auto&src=s.v[s.f[face][k]];out.normal=out.normal+src.normal*v.b[k];out.tangent=out.tangent+src.tangent*v.b[k];out.binormal=out.binormal+src.binormal*v.b[k];}for(int k=0;k<3;++k)for(int a=0;a<2;++a)out.uv[a]+=s.v[s.f[face][k]].uv[a]*v.b[k];return out;}
// The cavity vertex graph is a pure function of the polygons. Ids follow first
// appearance in polygon order (stored point = first occurrence); each vertex's
// edge list follows polygon/edge order. The polygons must outlive the graph.
struct CavityGraph {
    FlatMap<Quant,int,QuantHash> index;
    std::vector<std::pair<Quant,int>> sorted; // index in ascending key order
    std::vector<Vec> points;
    std::vector<std::vector<std::pair<int,double>>> edges;
    const std::vector<Poly>*polygons{};
};
CavityGraph buildCavityGraph(const std::vector<Poly>&cavity){
    CavityGraph graph;graph.polygons=&cavity;
    std::size_t corners=0;for(const auto&poly:cavity)corners+=poly.size();
    graph.points.reserve(corners);graph.edges.reserve(corners);
    std::vector<int> ids;
    for(const auto&poly:cavity){ids.clear();
        // Resolve each corner once; re-resolving an inserted key returns the same id.
        for(const auto&v:poly){auto q=quantExact(v.p);auto [it,fresh]=graph.index.try_emplace(q,int(graph.points.size()));
            if(fresh){graph.points.push_back(v.p);graph.edges.emplace_back();}
            ids.push_back(it->second);}
        for(std::size_t i=0;i<poly.size();++i){int a=ids[i],b=ids[(i+1)%poly.size()];
            if(a==b)continue;double lengthAB=length(graph.points[a]-graph.points[b]);
            graph.edges[a].emplace_back(b,lengthAB);graph.edges[b].emplace_back(a,lengthAB);}}
    graph.sorted.assign(graph.index.begin(),graph.index.end());
    std::sort(graph.sorted.begin(),graph.sorted.end(),[](const auto&a,const auto&b){return a.first<b.first;});
    return graph;
}
// Canonical skin points and cavity skins: first-insertion-wins hashed maps.
// Every ordered use (conform buckets) sorts by key explicitly.
using CanonicalMap=FlatMap<Quant,JoinedVertex,QuantHash>;
CanonicalMap assignCavitySkin(const CavityGraph&graph,const CanonicalMap&canonical){
    // Cavity skins follow the cut's connected boundary, not the globally
    // nearest triangle. The latter can come from an adjacent limb/sheet that
    // is spatially closer but disconnected from this wound. A source boundary
    // seed is mandatory for each component; an enclosed unsupported pocket
    // rejects before a renderer packet can be published.
    const auto&cavityIndex=graph.index;
    const auto&cavityPoints=graph.points;
    const auto&cavityEdges=graph.edges;
    const int vertexCount=int(cavityPoints.size());
    // Resolve canonical seeds once, in key order. canonical is not modified
    // here, so node pointers stay valid. quantExact(cavityPoints[i]) is i's key.
    std::vector<const JoinedVertex*> seedVertex(cavityPoints.size(),nullptr);
    for(auto&item:cavityIndex){auto found=canonical.find(item.first);if(found!=canonical.end())seedVertex[item.second]=&found->second;}
#ifndef GFY_CUT_DIAGNOSTICS
    // Every finite edge relaxes an infinite distance, so Dijkstra labels a
    // subset of the seed-reachable vertices. If plain reachability already
    // leaves a vertex unlabelled, the labelling loop below must throw this
    // same error first; reject before the heap propagation. (Diagnostics
    // builds keep the full path for their report.)
    {std::vector<char> reached(cavityPoints.size(),0);std::vector<int> pending;pending.reserve(cavityPoints.size());
        for(int i=0;i<vertexCount;++i)if(seedVertex[i]){reached[i]=1;pending.push_back(i);}
        for(std::size_t k=0;k<pending.size();++k)for(const auto&edge:cavityEdges[pending[k]])
            if(!reached[edge.first]){reached[edge.first]=1;pending.push_back(edge.first);}
        if(pending.size()!=cavityPoints.size()){
            // Enclosed pocket (rough/forensic cuts leave them): borrow the skin of the nearest seeded vertex, one seed per
            // unreached component, deterministic (first unreached vertex id, nearest seed with the lowest id on ties).
            std::vector<int> seeds;for(int i=0;i<vertexCount;++i)if(seedVertex[i])seeds.push_back(i);
            if(seeds.empty())throw CavityProvenanceError("cavity_component_without_exterior_skin_seed");
            for(int r=0;r<vertexCount;++r){if(reached[r])continue;
                int best=seeds.front();double bestD=1e300;for(int s:seeds){const Vec d=cavityPoints[r]-cavityPoints[s];const double q=dot(d,d);if(q<bestD){bestD=q;best=s;}}
                seedVertex[r]=seedVertex[best];reached[r]=1;std::vector<int> stack{r};
                while(!stack.empty()){const int v=stack.back();stack.pop_back();for(const auto&edge:cavityEdges[v])if(!reached[edge.first]){reached[edge.first]=1;stack.push_back(edge.first);}}}
        }}
#endif
    using Visit=std::tuple<double,int,int>; // path length, seed id, vertex id
    std::priority_queue<Visit,std::vector<Visit>,std::greater<Visit>> queue;
    std::vector<double> distance(cavityPoints.size(),std::numeric_limits<double>::infinity());
    std::vector<int> assignedSeed(cavityPoints.size(),-1);
    for(auto&item:cavityIndex)if(seedVertex[item.second]){
        distance[item.second]=0;assignedSeed[item.second]=item.second;queue.emplace(0.,item.second,item.second);}
    while(!queue.empty()){auto [d,seed,at]=queue.top();queue.pop();
        if(d>distance[at]+1e-12 || (std::abs(d-distance[at])<=1e-12 && seed!=assignedSeed[at]))continue;
        for(auto [next,edge]:cavityEdges[at]){double candidate=d+edge;
            if(candidate+1e-12<distance[next] || (std::abs(candidate-distance[next])<=1e-12 && seed<assignedSeed[next])){
                distance[next]=candidate;assignedSeed[next]=seed;queue.emplace(candidate,seed,next);}}}
    // A pure predicate: stop at the first difference (same result).
    bool needsSmooth=false;
    for(std::size_t i=0;i<cavityEdges.size()&&!needsSmooth;++i)for(auto [j,edge]:cavityEdges[i]){
        if(assignedSeed[i]<0 || assignedSeed[j]<0)continue;
        const auto& a=seedVertex[assignedSeed[i]]->w;
        const auto& b=seedVertex[assignedSeed[j]]->w;
        for(std::size_t bone=0;bone<a.size()&&!needsSmooth;++bone)
            needsSmooth=std::abs(a[bone]-b[bone])>.10;
        if(needsSmooth)break;
    }
    CanonicalMap cavitySkin;cavitySkin.reserve(std::size_t(vertexCount));
    // Pointers into cavitySkin by vertex id; reserved for every vertex, so no
    // insertion below moves an entry.
    std::vector<JoinedVertex*> skinVertex(cavityPoints.size(),nullptr);
#ifdef GFY_CUT_DIAGNOSTICS
    const auto&cavity=*graph.polygons;
#endif
    for(auto&item:graph.sorted){int seed=assignedSeed[item.second];if(seed<0){
#ifdef GFY_CUT_DIAGNOSTICS
        Bounds missing;std::size_t vertices=0;double nearest=1e30;Vec closest,unseeded;
        for(const auto&p:cavityIndex)if(assignedSeed[p.second]<0){auto position=cavityPoints[p.second];missing.add(position);++vertices;
            for(const auto&v:canonical){double distance=length(position-v.second.p);if(distance<nearest){nearest=distance;closest=v.second.p;unseeded=position;}}}
        std::map<Edge,unsigned> unsupportedEdges;
        for(const auto&poly:cavity)if(assignedSeed[cavityIndex.at(quantExact(poly.front().p))]<0)for(std::size_t n=0;n<poly.size();++n)++unsupportedEdges[edgeKeyExact(poly[n].p,poly[(n+1)%poly.size()].p)];
        unsigned openings=0,crowded=0;for(const auto&e:unsupportedEdges){openings+=e.second==1;crowded+=e.second>2;}
        std::cout<<"unseeded count="<<vertices<<" open="<<openings<<" nonmanifold="<<crowded<<" lo="<<missing.lo.x<<','<<missing.lo.y<<','<<missing.lo.z<<" hi="<<missing.hi.x<<','<<missing.hi.y<<','<<missing.hi.z<<" nearest="<<nearest<<" at="<<unseeded.x<<','<<unseeded.y<<','<<unseeded.z<<" seed="<<closest.x<<','<<closest.y<<','<<closest.z<<'\n';
#endif
        throw CavityProvenanceError("cavity_component_without_exterior_skin_seed");}
        const JoinedVertex*source=seedVertex[seed];
        if(!source)throw CavityProvenanceError("cavity_seed_lost");
        // Keys arrive in ascending order: append at the end.
        skinVertex[item.second]=&cavitySkin.try_emplace(item.first,*source).first->second;}
    StageProfile skinStages("skin");skinStages.mark("labels");
    if(needsSmooth){
        // Harmonic continuation of the actual skin weights, independently on
        // each connected cavity. Exact boundary weights are Dirichlet data.
        // Positive graph weights keep the solution inside their convex range.
        // No unrelated nearby surface contributes and no influence is dropped.
        const int count=vertexCount;
        std::vector<std::vector<int>> adjacency(count);
        std::vector<bool> fixed(count),visited(count);
        for(int i=0;i<count;++i){
            fixed[i]=seedVertex[i]!=nullptr;
            for(auto [j,edge]:cavityEdges[i])adjacency[i].push_back(j);
            auto& row=adjacency[i];std::sort(row.begin(),row.end());row.erase(std::unique(row.begin(),row.end()),row.end());
        }
        std::vector<int> unknownIndex(count,-1);
        for(int root=0;root<count;++root){if(visited[root])continue;
            std::vector<int> component{root};visited[root]=true;
            std::set<unsigned> bones;bool signedComponent=false;
            for(std::size_t k=0;k<component.size();++k){int i=component[k];
                if(fixed[i]){const auto&w=seedVertex[i]->w;
                    for(unsigned b=0;b<w.size();++b){signedComponent|=w[b]<0;if(w[b]>1e-12||w[b]<0)bones.insert(b);}}
                for(int j:adjacency[i])if(!visited[j]){visited[j]=true;component.push_back(j);}
            }
            if(bones.empty())throw CavityProvenanceError("cavity_component_without_exterior_skin_seed");
            // Solve every boundary influence. A cavity can span many bones
            // even though each original vertex fits the native four-weight
            // format. Reduction belongs to final packing, per vertex, rather
            // than rejecting the whole connected wound here.
            std::vector<int> unknown;
            for(int i:component)if(!fixed[i]){unknownIndex[i]=int(unknown.size());unknown.push_back(i);}
            const auto n=unknown.size();if(!n)continue;
            std::vector<std::array<double,64>> solved(n);
            // Bone-invariant operator data, built once per component. Each
            // unknown's free and fixed neighbours keep adjacency order, so
            // every sum below runs in the original order. degree is the exact
            // size_t->double conversion the mixed expressions performed.
            std::vector<double> degree(n);
            std::vector<std::size_t> freeFirst(n+1),fixedFirst(n+1);
            std::vector<std::size_t> freeColumn;std::vector<const std::array<double,64>*> fixedWeights,initial(n);
            for(std::size_t k=0;k<n;++k){int i=unknown[k];degree[k]=double(adjacency[i].size());initial[k]=&skinVertex[i]->w;
                freeFirst[k]=freeColumn.size();fixedFirst[k]=fixedWeights.size();
                for(int j:adjacency[i]){if(fixed[j])fixedWeights.push_back(&seedVertex[j]->w);else freeColumn.push_back(std::size_t(unknownIndex[j]));}}
            freeFirst[n]=freeColumn.size();fixedFirst[n]=fixedWeights.size();
            auto multiply=[&](const std::vector<double>&x,std::vector<double>&y){
                for(std::size_t k=0;k<n;++k){y[k]=degree[k]*x[k];
                    for(std::size_t e=freeFirst[k];e<freeFirst[k+1];++e)y[k]-=x[freeColumn[e]];}
            };
            auto product=[](const std::vector<double>&a,const std::vector<double>&b){double sum=0;for(std::size_t k=0;k<a.size();++k)sum+=a[k]*b[k];return sum;};
            // Bones are independent solves sharing one optional IC(0) factor.
            // The operator/RHS, initial weights and all validation tolerances
            // are unchanged. Jacobi remains the factor/solve-failure fallback.
            // Large components retain the existing four-bone concurrency;
            // outcomes are checked in ascending bone order.
            HarmonicIC0 factor;
            const bool factorReady=n>=64&&factor.build(degree,freeFirst,freeColumn);
            const std::vector<unsigned> order(bones.begin(),bones.end());
            enum Outcome:char{solvedOk,notConverged,outOfRange};
            std::vector<Outcome> outcome(order.size(),solvedOk);
            auto solveAttempt=[&](std::size_t slot,bool ic){const unsigned bone=order[slot];
                outcome[slot]=solvedOk;
                std::vector<double>x(n),rhs(n,0.),r(n),z(n),p(n),ap(n);
                double skinLow=0,skinHigh=1;if(signedComponent){skinHigh=0;for(const auto*w:fixedWeights){skinLow=std::min(skinLow,(*w)[bone]);skinHigh=std::max(skinHigh,(*w)[bone]);}}
                for(std::size_t k=0;k<n;++k){x[k]=(*initial[k])[bone];
                    for(std::size_t e=fixedFirst[k];e<fixedFirst[k+1];++e)rhs[k]+=(*fixedWeights[e])[bone];}
                multiply(x,ap);for(std::size_t k=0;k<n;++k){r[k]=rhs[k]-ap[k];z[k]=r[k]/degree[k];p[k]=z[k];}
                if(ic){factor.apply(r,z);p=z;}
                double rz=product(r,z);bool converged=false;
                // The Jacobi branch retains its former operation order.
                // IC(0) changes only the SPD preconditioning operation; residual
                // maxima, iteration cap and true-residual checks are unchanged.
                double maximum=0;for(std::size_t k=0;k<n;++k)maximum=std::max(maximum,std::abs(r[k])/degree[k]);
                for(unsigned iteration=0;iteration<2048;++iteration){
                    if(maximum<1e-10){converged=true;break;}
                    double denom=0;
                    for(std::size_t k=0;k<n;++k){double y=degree[k]*p[k];for(std::size_t e=freeFirst[k];e<freeFirst[k+1];++e)y-=p[freeColumn[e]];ap[k]=y;denom+=p[k]*y;}
                    if(!(denom>0)||!std::isfinite(denom))break;
                    double alpha=rz/denom,next=0;maximum=0;
                    if(ic){
                        for(std::size_t k=0;k<n;++k){x[k]+=alpha*p[k];r[k]-=alpha*ap[k];maximum=std::max(maximum,std::abs(r[k])/degree[k]);}
                        factor.apply(r,z);next=product(r,z);
                    }else{
                    for(std::size_t k=0;k<n;++k){x[k]+=alpha*p[k];r[k]-=alpha*ap[k];z[k]=r[k]/degree[k];next+=r[k]*z[k];maximum=std::max(maximum,std::abs(r[k])/degree[k]);}
                    }
                    double beta=next/rz;rz=next;
                    for(std::size_t k=0;k<n;++k)p[k]=z[k]+beta*p[k];
                }
                // Recompute the true residual rather than trusting accumulated
                // CG rounding. Failed solves never reach the renderer.
                multiply(x,ap);double error=0;
                for(std::size_t k=0;k<n;++k)error=std::max(error,std::abs(rhs[k]-ap[k])/degree[k]);
                if(!converged||error>1e-8){outcome[slot]=notConverged;return;}
                for(std::size_t k=0;k<n;++k)if(!std::isfinite(x[k])||x[k]<skinLow-1e-7||x[k]>skinHigh+1e-7){outcome[slot]=outOfRange;return;}
                for(std::size_t k=0;k<n;++k)solved[k][bone]=std::clamp(x[k],skinLow,skinHigh);
            };
            auto solveBone=[&](std::size_t slot){
                solveAttempt(slot,factorReady);
                if(factorReady&&outcome[slot]!=solvedOk)solveAttempt(slot,false);
            };
            const std::size_t workers=n>=4096?std::min<std::size_t>(order.size(),4):1;
            if(workers<=1)for(std::size_t slot=0;slot<order.size();++slot)solveBone(slot);
            else{std::atomic<std::size_t> nextSlot{0};
                // A helper's failure (e.g. std::bad_alloc) is rethrown here after
                // every helper joined; it never escapes a std::thread.
                std::vector<std::exception_ptr> helperFailure(workers);
                auto run=[&](std::size_t t){try{for(std::size_t slot;(slot=nextSlot++)<order.size();)solveBone(slot);}catch(...){helperFailure[t]=std::current_exception();}};
                std::vector<std::thread> pool;pool.reserve(workers-1);
                try{for(std::size_t t=1;t<workers;++t)pool.emplace_back([&run,t]{enterPainterThread();run(t);});}catch(...){}
                run(0);for(auto&t:pool)t.join();
                for(auto&f:helperFailure)if(f)std::rethrow_exception(f);}
            for(auto result:outcome){if(result==notConverged)throw CavityProvenanceError("cavity_skin_solver_did_not_converge");
                if(result==outOfRange)throw CavityProvenanceError("cavity_skin_solver_out_of_range");}
            // A preconditioned solve must also satisfy the existing joint
            // partition test before any skin vertex is changed. Retry the
            // original solver from the original initial weights if needed.
            bool partitionRetry=false;
            if(factorReady)for(std::size_t k=0;k<n&&!partitionRetry;++k){double sum=0;for(double w:solved[k])sum+=w;partitionRetry=std::abs(sum-1)>1e-6;}
            if(partitionRetry){for(std::size_t slot=0;slot<order.size();++slot)solveAttempt(slot,false);
                for(auto result:outcome){if(result==notConverged)throw CavityProvenanceError("cavity_skin_solver_did_not_converge");
                    if(result==outOfRange)throw CavityProvenanceError("cavity_skin_solver_out_of_range");}}
            for(std::size_t k=0;k<n;++k){double sum=0;for(double w:solved[k])sum+=w;
                if(std::abs(sum-1)>1e-6)throw CavityProvenanceError("cavity_skin_solver_partition_error");
                for(auto&w:solved[k])w/=sum;
                skinVertex[unknown[k]]->w=solved[k];skinVertex[unknown[k]]->hardware_corner=false;
            }
        }
    }
    skinStages.mark("smooth");
    return cavitySkin;
}
CanonicalMap assignCavitySkin(const std::vector<Poly>&cavity,const CanonicalMap&canonical){
    return assignCavitySkin(buildCavityGraph(cavity),canonical);
}
void conformCavityBoundary(std::vector<Poly>&cavity,const CanonicalMap&canonical,const Grid&g){
    // A skin seed or neighbouring cavity vertex can lie in the middle of an
    // edge. Insert that exact existing point before computing connectivity;
    // do not join a nearby but disconnected surface or invent skin weights.
    // First position per key (cavity corners, then canonical points), visited
    // in key order exactly as the former std::map<Quant,Vec>.
    FlatMap<Quant,Vec,QuantHash> first;first.reserve(canonical.size()*2);
    for(auto&poly:cavity)for(auto&v:poly)first.try_emplace(quantExact(v.p),v.p);
    for(auto&item:canonical)first.try_emplace(item.first,item.second.p);
    std::vector<const std::pair<Quant,Vec>*> points;points.reserve(first.size());for(const auto&item:first)points.push_back(&item);
    std::sort(points.begin(),points.end(),[](auto a,auto b){return a->first<b->first;});
    auto bucket=[&](Vec p){return Cell{int(std::floor((p.x-g.origin.x)/g.pitch)),int(std::floor((p.y-g.origin.y)/g.pitch)),int(std::floor((p.z-g.origin.z)/g.pitch))};};
    FlatMap<Cell,std::vector<Vec>,CellHash>buckets;
    for(auto item:points)buckets[bucket(item->second)].push_back(item->second);
    // Polygons are independent given the read-only buckets; in place, chunked.
    parallelChunks(cavity.size(),1024,[&](std::size_t firstPoly,std::size_t lastPoly,std::size_t){
    std::vector<std::pair<double,Vec>> inside;
    for(std::size_t polyIndex=firstPoly;polyIndex<lastPoly;++polyIndex){auto&poly=cavity[polyIndex];Poly out;out.reserve(poly.size()+4);
        for(std::size_t i=0;i<poly.size();++i){auto a=poly[i],b=poly[(i+1)%poly.size()];out.push_back(a);
            Vec d=b.p-a.p;double dd=dot(d,d);if(dd<1e-20)continue;
            Bounds bounds;bounds.add(a.p);bounds.add(b.p);
            Cell lo=bucket(bounds.lo-Vec{3e-8,3e-8,3e-8}),hi=bucket(bounds.hi+Vec{3e-8,3e-8,3e-8});
            // First candidate per exact t wins, then ascending t (std::map::emplace semantics).
            inside.clear();
            for(int x=lo.x;x<=hi.x;++x)for(int y=lo.y;y<=hi.y;++y)for(int z=lo.z;z<=hi.z;++z){
                auto it=buckets.find({x,y,z});if(it==buckets.end())continue;
                for(auto p:it->second){if(quantExact(p)==quantExact(a.p)||quantExact(p)==quantExact(b.p))continue;
                    double t=dot(p-a.p,d)/dd;if(t<=1e-8||t>=1-1e-8||length(p-(a.p+d*t))>3e-8)continue;
                    bool seen=false;for(const auto&h:inside)if(h.first==t){seen=true;break;}if(!seen)inside.emplace_back(t,p);
                }
            }
            std::sort(inside.begin(),inside.end(),[](const auto&x,const auto&y){return x.first<y.first;});
            for(auto&item:inside){V v;v.p=item.second;v.material=a.material;for(int k=0;k<3;++k)v.b[k]=a.b[k]+(b.b[k]-a.b[k])*item.first;out.push_back(v);}
        }
        poly=std::move(out);
    }});
}
// The former per-call field of seedEnvelopeOpenings, cached per envelope.
inline std::shared_ptr<const BodyBVH::SeedField> envelopeSeedField(const Source&skin,const BodyBVH&body){
    using Weights=std::array<double,64>;
    std::uint64_t key=skin.v.size();
    for(const auto&v:skin.v){std::uint64_t words[5];std::memcpy(words,&v.p,24);std::memcpy(&words[3],v.ids.data(),8);key=quantMix(key^words[0]);key=quantMix(key^words[1]);key=quantMix(key^words[2]);key=quantMix(key^words[3]);
        std::uint64_t w0,w1;std::memcpy(&w0,v.w.data(),8);std::memcpy(&w1,v.w.data()+2,8);key=quantMix(key^w0);key=quantMix(key^w1);}
    std::lock_guard<std::mutex> lock(*body.seedMutex);
    if(body.seedField&&body.seedField->key==key&&body.seedField->skinCount==skin.v.size())return body.seedField;
    // Lookup-only, first skin vertex at a key wins (as with map::emplace).
    std::unordered_map<Quant,Weights,QuantHash> original;original.reserve(skin.v.size());
    for(const auto&v:skin.v){Weights w{};for(unsigned k=0;k<4;++k)w[v.ids[k]]+=v.w[k];original.try_emplace(quantExact(v.p),w);}
    const auto&envelope=body.source;std::vector<Weights> field(envelope.v.size());std::vector<char> valid(envelope.v.size());
    // Sorted, deduplicated rows: the same ascending sequence a std::set gives.
    std::vector<std::vector<unsigned>> neighbors(envelope.v.size());
    for(const auto&f:envelope.f)for(unsigned k=0;k<3;++k){neighbors[f[k]].push_back(f[(k+1)%3]);neighbors[f[(k+1)%3]].push_back(f[k]);}
    for(auto&row:neighbors){std::sort(row.begin(),row.end());row.erase(std::unique(row.begin(),row.end()),row.end());}
    for(std::size_t i=0;i<envelope.v.size();++i){auto found=original.find(quantExact(envelope.v[i].p));if(found!=original.end()){field[i]=found->second;valid[i]=true;}}
    // The cap baker uses center fans. Only reconstruct a new center when ALL
    // its neighbors have original provenance; no nearest-surface fallback.
    for(std::size_t i=0;i<envelope.v.size();++i)if(!valid[i]&&!neighbors[i].empty()){
        bool anchored=true;for(auto n:neighbors[i])anchored=anchored&&valid[n];if(!anchored)continue;
        for(auto n:neighbors[i])for(unsigned b=0;b<64;++b)field[i][b]+=field[n][b]/neighbors[i].size();valid[i]=true;
    }
    auto out=std::make_shared<BodyBVH::SeedField>();out->key=key;out->skinCount=skin.v.size();out->valid=std::move(valid);
    out->first.resize(envelope.v.size()+1);
    for(std::size_t i=0;i<envelope.v.size();++i){out->first[i]=std::uint32_t(out->entries.size());
        for(unsigned b=0;b<64;++b){std::uint64_t bits;std::memcpy(&bits,&field[i][b],8);if(bits)out->entries.emplace_back(b,field[i][b]);}}
    out->first[envelope.v.size()]=std::uint32_t(out->entries.size());
    body.seedField=out;return out;
}
std::size_t seedEnvelopeOpenings(const std::vector<Poly>&cavity,CanonicalMap&canonical,const Source&skin,const BodyBVH&body){
    // A query-only cap closes authored coat hems, eye sockets and similar
    // openings. A wound reaching that cap has no visible skin vertex to seed
    // from. Reconstruct the cap's skin from its ORIGINAL rim vertices instead
    // of selecting an unrelated nearby limb or rejecting the whole impact.
    // This adds weights only; the cap never becomes visible geometry.
    using Weights=std::array<double,64>;
    // Reachability is a set property and every seed below is keyed by its own
    // distinct position, so hashed containers give the identical result.
    FlatMap<Quant,std::vector<Quant>,QuantHash> adjacency;
    for(const auto&poly:cavity)for(std::size_t i=0;i<poly.size();++i){auto a=quantExact(poly[i].p),b=quantExact(poly[(i+1)%poly.size()].p);adjacency[a].push_back(b);adjacency[b].push_back(a);}
    FlatMap<Quant,char,QuantHash> reached;reached.reserve(adjacency.size());std::vector<Quant> queue;
    for(const auto&row:adjacency)if(canonical.count(row.first)){reached.emplace(row.first,0);queue.push_back(row.first);}
    for(std::size_t i=0;i<queue.size();++i)for(auto next:adjacency.at(queue[i]))if(reached.emplace(next,0).second)queue.push_back(next);
    const auto&envelope=body.source;const auto cache=envelopeSeedField(skin,body);
    auto fieldRow=[&](std::size_t i){Weights w{};for(auto e=cache->first[i];e<cache->first[i+1];++e)w[cache->entries[e].first]=cache->entries[e].second;return w;};
    const auto&valid=cache->valid;
    // Each unreached vertex's seed is a pure function of its position and the
    // envelope: computed concurrently, inserted afterwards (distinct keys).
    std::vector<Quant> unreached;for(const auto&item:adjacency)if(!reached.count(item.first))unreached.push_back(item.first);
    std::vector<JoinedVertex> seeds(unreached.size());std::vector<char> seeded(unreached.size(),0);
    parallelChunks(unreached.size(),256,[&](std::size_t first,std::size_t last,std::size_t){for(std::size_t n=first;n<last;++n){const Quant key=unreached[n];
        Vec p{key[0]*1e-8,key[1]*1e-8,key[2]*1e-8},q;double squared=9e-14;std::uint32_t face=~std::uint32_t(0);
        body.nearest(0,p,squared,face,q);if(face>=envelope.f.size())continue;
        const auto&ids=envelope.f[face];if(!valid[ids[0]]||!valid[ids[1]]||!valid[ids[2]])continue;
        Vec a=envelope.v[ids[0]].p,u=envelope.v[ids[1]].p-a,v=envelope.v[ids[2]].p-a,d=q-a;
        double uu=dot(u,u),uv=dot(u,v),vv=dot(v,v),du=dot(d,u),dv=dot(d,v),den=uu*vv-uv*uv;if(std::abs(den)<1e-24)continue;
        double y=(vv*du-uv*dv)/den,z=(uu*dv-uv*du)/den;std::array<double,3>bary{1-y-z,y,z};
        bool inside=true;for(auto b:bary)inside=inside&&b>=-1e-8&&b<=1+1e-8;if(!inside)continue;
        JoinedVertex seed;seed.p=p;double sum=0;
        for(unsigned k=0;k<3;++k){const auto field=fieldRow(ids[k]);for(unsigned b=0;b<64;++b)seed.w[b]+=std::clamp(bary[k],0.,1.)*field[b];}
        for(auto w:seed.w)sum+=w;if(!(sum>0))continue;for(auto&w:seed.w)w/=sum;
        seeds[n]=seed;seeded[n]=1;}});
    std::size_t added=0;for(std::size_t n=0;n<unreached.size();++n)if(seeded[n]){canonical.emplace(unreached[n],seeds[n]);++added;}
    return added;
}
using JoinBuckets=FlatMap<Cell,std::vector<const JoinedVertex*>,CellHash>;
Cell joinBucket(Vec origin,double pitch,Vec p){return Cell{int(std::floor((p.x-origin.x)/pitch)),int(std::floor((p.y-origin.y)/pitch)),int(std::floor((p.z-origin.z)/pitch))};}
// Buckets of the endpoint box [lo,hi] that may hold a point within the 3e-8 m
// point-on-edge tolerance of segment ab, visited in strictly ascending Cell::<
// order (a subsequence of the former x/y/z box scan). Nested slab clipping in
// the segment parameter: x slab -> y column -> z run, so work follows the
// corridor length instead of the box volume and needs no allocation or sort.
// The cube radius r is the tolerance in cells plus 1e-6 cells, far above the
// rounding of cell coordinates below 1e6; otherwise the full box is scanned.
template<class Visit>void edgeCorridorCells(Vec origin,double pitch,Vec a,Vec b,Cell lo,Cell hi,Visit&&visit){
    if(lo.x>hi.x||lo.y>hi.y||lo.z>hi.z)return;
    auto box=[&]{for(int x=lo.x;x<=hi.x;++x)for(int y=lo.y;y<=hi.y;++y)for(int z=lo.z;z<=hi.z;++z)visit(Cell{x,y,z});};
    if(double(hi.x-lo.x+1)*double(hi.y-lo.y+1)*double(hi.z-lo.z+1)<=8){box();return;}
    const Vec u0{(a.x-origin.x)/pitch,(a.y-origin.y)/pitch,(a.z-origin.z)/pitch},u1{(b.x-origin.x)/pitch,(b.y-origin.y)/pitch,(b.z-origin.z)/pitch},du=u1-u0;
    const double r=3e-8/pitch+1e-6;bool usable=pitch>0&&std::isfinite(r);
    for(int k=0;k<3;++k)usable=usable&&std::abs(u0[k])<=1e6&&std::abs(u1[k])<=1e6&&std::abs(a[k])<=1e4&&std::abs(b[k])<=1e4;
    if(!usable){box();return;}
    // Narrow [t0,t1] to parameters whose cube meets [low,high] on one axis.
    auto clip=[&](int k,double low,double high,double&t0,double&t1){
        if(du[k]==0){if(u0[k]<low||u0[k]>high)t1=-1;return;}
        double ta=(low-u0[k])/du[k],tb=(high-u0[k])/du[k];if(du[k]<0)std::swap(ta,tb);
        t0=std::max(t0,ta);t1=std::min(t1,tb);};
    auto span=[&](int k,double t0,double t1,int first,int last,int&from,int&to){
        double p=u0[k]+t0*du[k],q=u0[k]+t1*du[k];if(p>q)std::swap(p,q);
        from=std::max(first,int(std::floor(p-r)));to=std::min(last,int(std::floor(q+r)));};
    int xa,xb;span(0,0,1,lo.x,hi.x,xa,xb);
    for(int x=xa;x<=xb;++x){double t0=0,t1=1;clip(0,x-r,x+1+r,t0,t1);if(t0>t1)continue;
        int ya,yb;span(1,t0,t1,lo.y,hi.y,ya,yb);
        for(int y=ya;y<=yb;++y){double s0=t0,s1=t1;clip(1,y-r,y+1+r,s0,s1);if(s0>s1)continue;
            int za,zb;span(2,s0,s1,lo.z,hi.z,za,zb);
            for(int z=za;z<=zb;++z)visit(Cell{x,y,z});}}
}
// Canonical points strictly inside edge ab, as (t, point) with the first
// bucket-order candidate winning a duplicate t, sorted by t: the same entries
// std::map<double,JoinedVertex>::emplace kept, without a node per edge.
void conformEdgeHits(const JoinBuckets&buckets,Vec origin,double pitch,const JoinedVertex&a,const JoinedVertex&b,std::vector<std::pair<double,const JoinedVertex*>>&hits){
    hits.clear();Vec d=b.p-a.p;double dd=dot(d,d);Bounds bounds;bounds.add(a.p);bounds.add(b.p);Cell lo=joinBucket(origin,pitch,bounds.lo-Vec{3e-8,3e-8,3e-8}),hi=joinBucket(origin,pitch,bounds.hi+Vec{3e-8,3e-8,3e-8});
    const Quant qa=quantExact(a.p),qb=quantExact(b.p);
    auto visit=[&](Cell c){auto it=buckets.find(c);if(it==buckets.end())return;for(const JoinedVertex*entry:it->second){const auto&point=*entry;const Quant qp=quantExact(point.p);if(qp==qa||qp==qb)continue;double t=dot(point.p-a.p,d)/dd;if(t<=1e-8||t>=1-1e-8||length(point.p-(a.p+d*t))>3e-8)continue;
        bool seen=false;for(const auto&h:hits)if(h.first==t){seen=true;break;}if(!seen)hits.emplace_back(t,entry);}};
    edgeCorridorCells(origin,pitch,a.p,b.p,lo,hi,visit);
    std::sort(hits.begin(),hits.end(),[](const auto&x,const auto&y){return x.first<y.first;});
}
// The point inserted at parameter t of edge ab (former conformEdgePoints body).
JoinedVertex conformedPoint(const JoinedVertex&point,const JoinedVertex&a,const JoinedVertex&b,double t){
    JoinedVertex v=point;for(int uv=0;uv<2;++uv)v.uv[uv]=a.uv[uv]+(b.uv[uv]-a.uv[uv])*t;
    v.normal=a.normal+(b.normal-a.normal)*t;v.tangent=a.tangent+(b.tangent-a.tangent)*t;v.binormal=a.binormal+(b.binormal-a.binormal)*t;
    v.sourceFace=a.sourceFace;for(int k=0;k<3;++k)v.sourceBary[k]=a.sourceBary[k]+(b.sourceBary[k]-a.sourceBary[k])*t;return v;
}
// Canonical points strictly inside edge ab, keyed by t (first bucket-order
// candidate wins a duplicate t). fullBox restores the former box enumeration.
void conformEdgePoints(const JoinBuckets&buckets,Vec origin,double pitch,const JoinedVertex&a,const JoinedVertex&b,std::map<double,JoinedVertex>&inside,bool fullBox=false){
    Vec d=b.p-a.p;double dd=dot(d,d);Bounds bounds;bounds.add(a.p);bounds.add(b.p);Cell lo=joinBucket(origin,pitch,bounds.lo-Vec{3e-8,3e-8,3e-8}),hi=joinBucket(origin,pitch,bounds.hi+Vec{3e-8,3e-8,3e-8});
    auto visit=[&](Cell c){auto it=buckets.find(c);if(it==buckets.end())return;for(const JoinedVertex*entry:it->second){const auto&point=*entry;if(quantExact(point.p)==quantExact(a.p)||quantExact(point.p)==quantExact(b.p))continue;double t=dot(point.p-a.p,d)/dd;if(t<=1e-8||t>=1-1e-8||length(point.p-(a.p+d*t))>3e-8)continue;JoinedVertex v=point;for(int uv=0;uv<2;++uv)v.uv[uv]=a.uv[uv]+(b.uv[uv]-a.uv[uv])*t;
        v.normal=a.normal+(b.normal-a.normal)*t;v.tangent=a.tangent+(b.tangent-a.tangent)*t;v.binormal=a.binormal+(b.binormal-a.binormal)*t;
        v.sourceFace=a.sourceFace;for(int k=0;k<3;++k)v.sourceBary[k]=a.sourceBary[k]+(b.sourceBary[k]-a.sourceBary[k])*t;
        inside.emplace(t,v);}};
    if(fullBox){for(int x=lo.x;x<=hi.x;++x)for(int y=lo.y;y<=hi.y;++y)for(int z=lo.z;z<=hi.z;++z)visit(Cell{x,y,z});}
    else edgeCorridorCells(origin,pitch,a.p,b.p,lo,hi,visit);
}
// Exact replacement for the weld search over the 3x3x3 neighbouring weld cells:
// points live in 4x4x4-cell super buckets; the candidates of those 27 cells are
// visited in (cell x,y,z) order and, inside one cell, in insertion order -- the
// order of the former nested loop -- so the first strict minimum is identical.
struct WeldIndex {
    struct Entry {Cell cell;Vec p;};
    FlatMap<Cell,std::vector<Entry>,CellHash> super;std::vector<const Entry*> scratch;
    static int group(int c){return c>=0?c/4:-((-c+3)/4);}
    void insert(Cell c,Vec p){super[Cell{group(c.x),group(c.y),group(c.z)}].push_back({c,p});}
    bool nearest(Cell c,Vec p,double&best,Vec&result){
        scratch.clear();
        for(int x=group(c.x-1);x<=group(c.x+1);++x)for(int y=group(c.y-1);y<=group(c.y+1);++y)for(int z=group(c.z-1);z<=group(c.z+1);++z){
            auto it=super.find({x,y,z});if(it==super.end())continue;
            for(const auto&e:it->second)if(std::abs(e.cell.x-c.x)<=1&&std::abs(e.cell.y-c.y)<=1&&std::abs(e.cell.z-c.z)<=1)scratch.push_back(&e);}
        if(scratch.empty())return false;
        std::stable_sort(scratch.begin(),scratch.end(),[](const Entry*a,const Entry*b){return a->cell<b->cell;});
        bool found=false;for(auto e:scratch){double d=dot(p-e->p,p-e->p);if(d<best){best=d;result=e->p;found=true;}}
        return found;
    }
    // Same result as nearest() on one index holding base's entries followed by
    // this one's (base entries precede in every cell, as they were inserted first).
    bool nearest(const WeldIndex& base,Cell c,Vec p,double&best,Vec&result){
        scratch.clear();
        for(int x=group(c.x-1);x<=group(c.x+1);++x)for(int y=group(c.y-1);y<=group(c.y+1);++y)for(int z=group(c.z-1);z<=group(c.z+1);++z){
            for(const auto* index:{&base,static_cast<const WeldIndex*>(this)}){auto it=index->super.find({x,y,z});if(it==index->super.end())continue;
                for(const auto&e:it->second)if(std::abs(e.cell.x-c.x)<=1&&std::abs(e.cell.y-c.y)<=1&&std::abs(e.cell.z-c.z)<=1)scratch.push_back(&e);}}
        if(scratch.empty())return false;
        std::stable_sort(scratch.begin(),scratch.end(),[](const Entry*a,const Entry*b){return a->cell<b->cell;});
        bool found=false;for(auto e:scratch){double d=dot(p-e->p,p-e->p);if(d<best){best=d;result=e->p;found=true;}}
        return found;
    }
};
JoinResult join(const Source&s,const BodyBVH&sourceTree,const BodyBVH&body,const Grid&g,const ClipResult&exteriorMesh,const std::vector<Quad>&quads){
    StageProfile stages("join");JoinResult result;auto cavity=cavityPolygons(body,quads);stages.mark("cavity_polygons");result.cavityPolygons=cavity.size();std::vector<JoinedPolygon>polygons;polygons.reserve(exteriorMesh.polygons.size()+cavity.size());CanonicalMap canonical;
    {std::size_t corners=0;for(const auto&f:exteriorMesh.polygons)corners+=f.polygon.size();for(const auto&poly:cavity)corners+=poly.size();canonical.reserve(corners);}
    for(auto&fragment:exteriorMesh.polygons){JoinedPolygon out;out.v.reserve(fragment.polygon.size());for(auto&v:fragment.polygon){auto j=interpolateVertex(s,fragment.sourceFace,v);canonical.try_emplace(quantExact(j.p),j);out.v.push_back(j);}polygons.push_back(std::move(out));}
    CanonicalMap cavitySkin;
    try {cavitySkin=assignCavitySkin(cavity,canonical);}
    catch(const CavityProvenanceError&e){
        stages.mark("cs_attempt1");
        if(std::string(e.what())!="cavity_component_without_exterior_skin_seed")throw;
        conformCavityBoundary(cavity,canonical,g);stages.mark("cs_conform");
        // Attempts 2 and 3 see identical polygons (seedEnvelopeOpenings only
        // adds canonical seeds), so they share one graph of this cavity.
        const auto graph=buildCavityGraph(cavity);stages.mark("cs_graph");
        try {cavitySkin=assignCavitySkin(graph,canonical);}
        catch(const CavityProvenanceError&again){
            stages.mark("cs_attempt2");
            if(!body.preserveCrossings||std::string(again.what())!="cavity_component_without_exterior_skin_seed"||!seedEnvelopeOpenings(cavity,canonical,s,body))throw;
            stages.mark("cs_seed");
            cavitySkin=assignCavitySkin(graph,canonical);
        }
    }
    stages.mark("cavity_skin");
    auto nearest=[&](Vec p){JoinedVertex out;auto found=cavitySkin.find(quantExact(p));
        if(found==cavitySkin.end())throw CavityProvenanceError("cavity_vertex_not_in_component");
        out=found->second;out.p=p;out.uv={p.x/.03,p.y/.03};
        canonical.try_emplace(quantExact(p),out);return out;};
    for(auto&poly:cavity){cancelCavityBacktracks(poly);if(poly.size()<3)continue;JoinedPolygon out;out.material=poly.front().material;out.v.reserve(poly.size());
        // Plane intersections are snapped to a 10 nm lattice. A line can
        // therefore return as a narrow polygon and its fan creates spurious
        // overlapping triangles. Cull only polygons indistinguishable from
        // a line at that precision; the final boundary gate still requires
        // the remaining skin and cavity edges to close exactly.
        Vec lineA=poly.front().p,lineB=lineA;double longest=0;
        for(const auto&v:poly){double d=length(v.p-lineA);if(d>longest){longest=d;lineB=v.p;}}
        if(longest>0){double width=0;for(const auto&v:poly)width=std::max(width,length(cross(v.p-lineA,lineB-lineA))/longest);
            if(width<=3e-8)continue;}
        Vec normal;for(std::size_t k=1;k+1<poly.size();++k)normal=normal+cross(poly[k].p-poly[0].p,poly[k+1].p-poly[0].p);
        double norm=length(normal);if(norm<1e-16)throw CavityProvenanceError("invalid_cavity_normal");normal=normal*(1/norm);
        Vec tangent=std::abs(normal.x)>.9?Vec{0,0,1}:Vec{1,0,0};Vec binormal=cross(normal,tangent);
        for(auto&v:poly){auto vertex=nearest(v.p);vertex.normal=normal;vertex.tangent=tangent;vertex.binormal=binormal;vertex.sourceFace=~std::uint32_t(0);vertex.sourceBary={};vertex.uv={dot(vertex.p,tangent)/.03,dot(vertex.p,binormal)/.03};out.v.push_back(vertex);}polygons.push_back(std::move(out));}
    // Canonical endpoints carry one skin solution across material/UV seams.
    // A spatial bucket visits only points near an edge, avoiding a global O(VE)
    // search while conforming partial shared voxel and exterior edge segments.
    stages.mark("cavity_vertices");
    JoinBuckets buckets;
    {std::vector<const CanonicalMap::value_type*> ordered;ordered.reserve(canonical.size());for(const auto&item:canonical)ordered.push_back(&item);
     std::sort(ordered.begin(),ordered.end(),[](auto a,auto b){return a->first<b->first;});
     buckets.reserve(ordered.size());for(auto item:ordered)buckets[joinBucket(g.origin,g.pitch,item->second.p)].push_back(&item->second);}
    // Each polygon's conformed edges depend only on the (now read-only)
    // canonical buckets: chunks of polygons run concurrently, in place.
    parallelChunks(polygons.size(),1024,[&](std::size_t firstPoly,std::size_t lastPoly,std::size_t){
    std::vector<std::pair<double,const JoinedVertex*>> hits;
    for(std::size_t polyIndex=firstPoly;polyIndex<lastPoly;++polyIndex){auto&poly=polygons[polyIndex];
        // Most polygons gain no point: only rebuild those that do.
        std::vector<JoinedVertex>out;bool grown=false;
        for(std::size_t i=0;i<poly.v.size();++i){const auto&a=poly.v[i];const auto&b=poly.v[(i+1)%poly.v.size()];
            Vec d=b.p-a.p;double dd=dot(d,d);if(dd<1e-20){if(grown)out.push_back(a);continue;}
            conformEdgeHits(buckets,g.origin,g.pitch,a,b,hits);
            if(!hits.empty()&&!grown){grown=true;out.reserve(poly.v.size()+hits.size()+4);out.insert(out.end(),poly.v.begin(),poly.v.begin()+std::ptrdiff_t(i));}
            if(grown){out.push_back(a);for(const auto&h:hits)out.push_back(conformedPoint(*h.second,a,b,h.first));}}
        if(grown)poly.v=std::move(out);}});
    stages.mark("conform_polygons");
    {std::size_t expected=0;for(const auto&poly:polygons)expected+=poly.v.size()==3?1:poly.v.size();result.triangles.reserve(expected);}
    for(auto&poly:polygons){if(poly.v.size()==3){result.triangles.push_back({poly.material,{poly.v[0],poly.v[1],poly.v[2]}});continue;}JoinedVertex center;for(auto&v:poly.v){center.p=center.p+v.p;center.normal=center.normal+v.normal;center.tangent=center.tangent+v.tangent;center.binormal=center.binormal+v.binormal;center.sourceFace=poly.v[0].sourceFace;for(int k=0;k<3;++k)center.sourceBary[k]+=v.sourceBary[k];for(int uv=0;uv<2;++uv)center.uv[uv]+=v.uv[uv];for(std::uint32_t bone=0;bone<s.bones;++bone)center.w[bone]+=v.w[bone];}double inv=1./double(poly.v.size());center.p=center.p*inv;center.normal=center.normal*inv;center.tangent=center.tangent*inv;center.binormal=center.binormal*inv;for(auto&b:center.sourceBary)b*=inv;for(auto&uv:center.uv)uv*=inv;for(auto&w:center.w)w*=inv;
        for(std::size_t i=0;i<poly.v.size();++i){const auto&a=poly.v[i];const auto&b=poly.v[(i+1)%poly.v.size()];if(length(cross(a.p-center.p,b.p-center.p))<1e-16)continue;result.triangles.push_back({poly.material,{center,a,b}});}}
    // Joined polygons, buckets and the vertex maps are dead from here on; release
    // them before the native stages rather than at return.
    std::vector<JoinedPolygon>().swap(polygons);buckets=JoinBuckets();
    canonical=CanonicalMap();cavitySkin=CanonicalMap();
    // The renderer stores float positions. Validate that actual representation,
    // removing triangles collapsed by conversion rather than treating their
    // nanometre sliver edges as a four-face surface. Never bridge a visible gap.
    stages.mark("triangulate");
    // Lookup-only key maps (never iterated): hashed, same first-wins semantics.
    FlatMap<Quant,const std::array<double,64>*,QuantHash> nativeWeights;
    FlatMap<Quant,Quant,QuantHash> nativeOriginalPosition;
    FlatMap<Quant,std::uint32_t,QuantHash> nativeFirstSourceFace;
    FlatMap<std::array<Quant,3>,char,QuantArrayHash<3>> nativeFaces;nativeFaces.reserve(result.triangles.size());
    constexpr double weld=3e-7; // <0.001 of even the High voxel edge.
    auto weldBucket=[weld](Vec p){return Cell{int(std::floor(p.x/weld)),int(std::floor(p.y/weld)),int(std::floor(p.z/weld))};};
    WeldIndex welded;
    // Original model vertices have priority and retain their exact positions.
    FlatMap<Quant,char,QuantHash> originalPositions;originalPositions.reserve(s.v.size());
    for(auto&v:s.v){if(originalPositions.try_emplace(quantExact(v.p),0).second)welded.insert(weldBucket(v.p),v.p);}
    stages.mark("np_original_positions");
    // A query that returned itself (an original position, or a new weld
    // representative) returns itself again: nothing inserted later can be
    // closer than distance 0 or precede it with an equal distance.
    struct BitsHash{std::size_t operator()(const std::array<std::uint64_t,3>&k)const{return std::size_t(quantMix(k[0]^quantMix(k[1]^quantMix(k[2]))));}};
    FlatMap<std::array<std::uint64_t,3>,char,BitsHash> selfCanonical;selfCanonical.reserve(result.triangles.size());
    auto bits=[](Vec p){std::array<std::uint64_t,3>k;std::memcpy(k.data(),&p,24);return k;};
    auto canonicalPositionSlow=[&](Vec p){
        if(originalPositions.count(quantExact(p)))return p;
        Cell c=weldBucket(p);double best=weld*weld;Vec result=p;const bool found=welded.nearest(c,p,best,result);
        if(!found)welded.insert(c,p);return result;
    };
    auto canonicalPosition=[&](Vec p){auto key=bits(p);if(selfCanonical.count(key))return p;
        Vec out=canonicalPositionSlow(p);if(!std::memcmp(&out,&p,sizeof(Vec)))selfCanonical.emplace(key,0);return out;};
    // A clipped vertex on an authored source edge must use that edge's source
    // weights. Independent triangle clipping can leave a tiny third-vertex
    // barycentric term, even though both source faces share the same edge.
    // This only applies to an exact source-face edge within 1 nm of the
    // renderer's already canonicalized position; it cannot invent a seam.
    auto authoredEdgeWeights=[&](const JoinedVertex&v,std::array<double,64>&out){
        if(v.sourceFace>=s.f.size())return false;
        const auto&face=s.f[v.sourceFace];double nearest=1e-18;int choice=-1;double fraction=0;
        for(int edge=0;edge<3;++edge){auto a=s.v[face[edge]].p,b=s.v[face[(edge+1)%3]].p;
            Vec d=b-a;double dd=dot(d,d);if(dd<=1e-20)continue;
            double t=dot(v.p-a,d)/dd;if(t<0||t>1)continue;
            double distance2=dot(v.p-(a+d*t),v.p-(a+d*t));
            if(distance2<nearest){nearest=distance2;choice=edge;fraction=t;}}
        if(choice<0)return false;
        out={};
        const auto&from=s.v[face[choice]],&to=s.v[face[(choice+1)%3]];
        for(int k=0;k<4;++k){out[from.ids[k]]+=(1-fraction)*from.w[k];out[to.ids[k]]+=fraction*to.w[k];}
        return true;
    };
    // Prepared triangles are compacted in place (slot index <= source index,
    // same order); their unwelded keys run in a parallel vector.
    std::vector<std::array<Quant,3>>preparedUnwelded;preparedUnwelded.reserve(result.triangles.size());
    std::size_t preparedCount=0;
    for(std::size_t index=0;index<result.triangles.size();++index){auto&triangle=result.triangles[index];std::array<Quant,3> keys,unwelded;
        for(int k=0;k<3;++k){auto&v=triangle.v[k];unwelded[k]=quantExact(v.p);v.p={double(float(v.p.x)),double(float(v.p.y)),double(float(v.p.z))};v.p=canonicalPosition(v.p);keys[k]=quantExact(v.p);}
        if(keys[0]==keys[1]||keys[1]==keys[2]||keys[2]==keys[0])continue;
        if(length(cross(triangle.v[1].p-triangle.v[0].p,triangle.v[2].p-triangle.v[0].p))<1e-16)continue;
        auto face=keys;std::sort(face.begin(),face.end());if(!nativeFaces.try_emplace(face,0).second)continue;
        if(preparedCount!=index)result.triangles[preparedCount]=std::move(triangle);
        ++preparedCount;preparedUnwelded.push_back(unwelded);
    }
    result.triangles.resize(preparedCount);
    std::vector<JoinedTriangle>prepared=std::move(result.triangles);result.triangles.clear();
    stages.mark("native_prepare");
    // Collect the original edge solution before visiting cavity polygons: a
    // cavity vertex may be emitted earlier than the source face at that key.
    std::unordered_map<Quant,std::array<double,64>,QuantHash>edgeSkin;
    for(const auto&triangle:prepared)for(const auto&v:triangle.v){
        std::array<double,64>value;if(!authoredEdgeWeights(v,value))continue;
        auto[it,fresh]=edgeSkin.try_emplace(quantExact(v.p),value);
        if(!fresh)for(unsigned bone=0;bone<s.bones;++bone)
            if(std::abs(value[bone]-it->second[bone])>1e-6)
                throw CavityProvenanceError("authored_edge_skin_conflict");
    }
    // Float conversion can displace the same authored edge by more than the
    // 1 nm single-face test above. Admit a slightly wider correction ONLY when
    // two source faces at this exact rendered key share both original vertex
    // IDs. This is source topology evidence, not a nearby-bone projection.
    // (key, face) -> first corner's weights/unwelded key; first position per
    // key. A stable sort by (key, face) over corners in scan order reproduces
    // the former nested std::map (first emplace wins, ascending faces).
    struct FaceCorner {Quant key;std::uint32_t face;const std::array<double,64>*weights;Quant unwelded;};
    std::vector<FaceCorner> corners;FlatMap<Quant,Vec,QuantHash> positionAtKey;
    for(std::size_t index=0;index<prepared.size();++index)for(int corner=0;corner<3;++corner){const auto&v=prepared[index].v[corner];
        if(v.sourceFace<s.f.size()){const auto key=quantExact(v.p);
            corners.push_back({key,v.sourceFace,&v.w,preparedUnwelded[index][corner]});
            positionAtKey.try_emplace(key,v.p);}}
    std::stable_sort(corners.begin(),corners.end(),[](const FaceCorner&a,const FaceCorner&b){return a.key<b.key||(a.key==b.key&&a.face<b.face);});
    corners.erase(std::unique(corners.begin(),corners.end(),[](const FaceCorner&a,const FaceCorner&b){return a.key==b.key&&a.face==b.face;}),corners.end());
    for(std::size_t groupBegin=0;groupBegin<corners.size();){std::size_t groupEnd=groupBegin+1;while(groupEnd<corners.size()&&corners[groupEnd].key==corners[groupBegin].key)++groupEnd;
        const std::size_t groupSize=groupEnd-groupBegin;const std::size_t firstIndex=groupBegin;groupBegin=groupEnd;
        if(groupSize<2||groupSize>8)continue;
        const Quant itemKey=corners[firstIndex].key;const auto p=positionAtKey.at(itemKey);
        for(std::size_t i=firstIndex;i<firstIndex+groupSize;++i)for(std::size_t j=i+1;j<firstIndex+groupSize;++j){
            const auto&first=corners[i];const auto&second=corners[j];
            if(first.unwelded!=second.unwelded)continue;
            double disagreement=0;
            for(unsigned bone=0;bone<s.bones;++bone)disagreement=std::max(disagreement,
                std::abs((*first.weights)[bone]-(*second.weights)[bone]));
            if(disagreement<=1e-6||disagreement>1e-5)continue;
            std::array<std::uint32_t,2>shared{};unsigned count=0;
            for(auto a:s.f[first.face])for(auto b:s.f[second.face])if(a==b&&count<2)shared[count++]=a;
            if(count!=2)continue;
            const auto&from=s.v[shared[0]],&to=s.v[shared[1]];Vec d=to.p-from.p;
            const double dd=dot(d,d);if(dd<=1e-20)continue;
            const double t=dot(p-from.p,d)/dd;if(t<0||t>1)continue;
            // At yâ‰ˆ1.36 m, half an IEEE float ULP is 59.6 nm. Include that
            // unavoidable renderer rounding, with exact shared source IDs.
            if(dot(p-(from.p+d*t),p-(from.p+d*t))>4.9e-15)continue; // 70 nm.
            std::array<double,64>value{};
            for(int k=0;k<4;++k){value[from.ids[k]]+=(1-t)*from.w[k];value[to.ids[k]]+=t*to.w[k];}
            auto[it,fresh]=edgeSkin.emplace(itemKey,value);
            if(!fresh)for(unsigned bone=0;bone<s.bones;++bone)
                if(std::abs(value[bone]-it->second[bone])>1e-6)
                    throw CavityProvenanceError("authored_shared_edge_skin_conflict");
        }
    }
    stages.mark("edge_skin");
    nativeWeights.reserve(prepared.size());nativeOriginalPosition.reserve(prepared.size());nativeFirstSourceFace.reserve(prepared.size());
    for(std::size_t index=0;index<prepared.size();++index){auto&triangle=prepared[index];const auto&unwelded=preparedUnwelded[index];
        for(int k=0;k<3;++k){auto&v=triangle.v[k];auto key=quantExact(v.p);
            auto sourceSkin=edgeSkin.find(key);if(sourceSkin!=edgeSkin.end()){
                // An authored edge keeps the source skin; a small disagreement from a rough cut edge no longer
                // cancels the surface (it refused an in-game sever, 2026-09-26): the source skin is adopted as before.
                if(!v.hardware_corner)v.w=sourceSkin->second;
            }
            auto [it,fresh]=nativeWeights.try_emplace(key,&v.w);
            if(fresh){nativeOriginalPosition.emplace(key,unwelded[k]);
                nativeFirstSourceFace.emplace(key,v.sourceFace);}
            else{
                // Collapsing a generated edge merges its skin interpolation
                // too. Exact coincident vertices must still agree beforehand;
                // only a bounded, nonzero spatial weld may unify two solutions.
                if(nativeOriginalPosition.at(key)==unwelded[k])for(unsigned bone=0;bone<s.bones;++bone)
                    if(std::abs(v.w[bone]-(*it->second)[bone])>1e-6){
#ifdef GFY_CUT_DIAGNOSTICS
                        std::cout<<"weight_conflict "<<std::setprecision(14)<<v.p.x<<' '<<v.p.y<<' '<<v.p.z
                                 <<' '<<unwelded[k][0]*1e-8<<' '<<unwelded[k][1]*1e-8<<' '<<unwelded[k][2]*1e-8
                                 <<' '<<triangle.material<<' '<<v.sourceFace<<' '<<nativeFirstSourceFace.at(key)<<' '<<bone
                                 <<' '<<v.w[bone]<<' '<<(*it->second)[bone]<<'\n';
                        for(auto faceId:{v.sourceFace,nativeFirstSourceFace.at(key)}){
                            if(faceId>=s.f.size())continue;
                            std::cout<<"conflict_source_face "<<faceId;
                            for(auto index:s.f[faceId]){
                                const auto&sv=s.v[index];
                                std::cout<<' '<<index<<' '<<sv.p.x<<' '<<sv.p.y<<' '<<sv.p.z;
                                for(int j=0;j<4;++j)std::cout<<' '<<sv.ids[j]<<':'<<sv.w[j];
                            }
                            std::cout<<'\n';
                        }
#endif
                        throw CavityProvenanceError("native_float_seam_weight_conflict");}
                if(!v.hardware_corner)v.w=*it->second;}}
    }
    // Every prepared triangle is kept (or the loop threw): take the storage.
    std::vector<JoinedTriangle> nativeTriangles=std::move(prepared);
    stages.mark("native_weights");
    EdgeCountMap retainedCounts;
    result.numericalSpurs=removeFloatCavitySpurs(nativeTriangles,weld,body.preserveCrossings,&retainedCounts);stages.mark("spurs_float");
    if(body.preserveCrossings)result.numericalSpurs+=removeSubmicronSpurs(nativeTriangles,1e-6,&retainedCounts);stages.mark("spurs_submicron");
    // Exactly the edge counts of the final triangles once emptied keys go.
    {EdgeCountMap retained;retained.reserve(retainedCounts.size());for(const auto&e:retainedCounts)if(e.second)retained.try_emplace(e.first,e.second);retainedCounts=std::move(retained);}
    stages.mark("spurs");
    result.triangles=std::move(nativeTriangles);
    // Lookup-only hashed maps. The one order-sensitive walk (the gate below,
    // whose numerical-opening budget is consumed in key order) visits the
    // non-manifold/open edges sorted exactly as the former std::map did.
    // result.triangles is final from here on, so skin can point into it.
    using EdgeCounts=EdgeCountMap;
    EdgeCounts edgeCounts=std::move(retainedCounts),original;original.reserve(s.f.size()*2);
    FlatMap<Quant,const std::array<double,64>*,QuantHash>skin;skin.reserve(result.triangles.size());
    for(auto&f:s.f)for(int k=0;k<3;++k)++original[edgeKeyExact(s.v[f[k]].p,s.v[f[(k+1)%3]].p)];
    for(auto&t:result.triangles){if(t.material)++result.cavityTriangles;for(int k=0;k<3;++k){auto&v=t.v[k];auto[it,fresh]=skin.try_emplace(quantExact(v.p),&v.w);if(!fresh)for(std::uint32_t bone=0;bone<s.bones;++bone)result.maxJoinWeightConflict=std::max(result.maxJoinWeightConflict,std::abs((*it->second)[bone]-v.w[bone]));}}
    stages.mark("edge_maps");
    // The occupancy envelope closes the source's pre-existing eye/mouth/neck
    // openings with temporary triangles. Those triangles are not visible skin.
    // Cutting across an opening subdivides its old rim and makes a new cavity
    // edge on the temporary closure. Both remain part of the ORIGINAL opening,
    // not a crack in the joined wound. Check exact geometry, not a larger
    // distance tolerance or a blanket exemption for nearby edges.
    using FaceKey=std::array<Quant,3>;
    auto faceKey=[](const Source& src,const std::array<std::uint32_t,3>& f){FaceKey key{quantExact(src.v[f[0]].p),quantExact(src.v[f[1]].p),quantExact(src.v[f[2]].p)};std::sort(key.begin(),key.end());return key;};
    // Membership tests only: hashed faces and a per-face flag.
    std::unordered_map<FaceKey,char,QuantArrayHash<3>>sourceFaces;sourceFaces.reserve(s.f.size());for(auto&f:s.f)sourceFaces.try_emplace(faceKey(s,f),0);
    // Evaluated lazily for the few envelope faces an open edge actually queries.
    std::vector<signed char>closureState(body.source.f.size(),-1);
    auto closure=[&](std::uint32_t f){auto&state=closureState[f];if(state<0)state=sourceFaces.count(faceKey(body.source,body.source.f[f]))?0:1;return state!=0;};
    auto position=[](Quant q){return Vec{q[0]*1e-8,q[1]*1e-8,q[2]*1e-8};};
    std::vector<Edge>originalBoundary;for(const auto&entry:original)if(entry.second==1)originalBoundary.push_back(entry.first);
    // Any-match search over the original boundary segments. A segment can only
    // accept point a if a lies within weld of it, so a cell index of the
    // segments' weld-padded bounds is an exact candidate filter.
    constexpr double boundaryCell=.01;
    auto boundaryKey=[&](Vec p){return Cell{int(std::floor(p.x/boundaryCell)),int(std::floor(p.y/boundaryCell)),int(std::floor(p.z/boundaryCell))};};
    FlatMap<Cell,std::vector<std::uint32_t>,CellHash> boundaryCells;
    for(std::uint32_t i=0;i<originalBoundary.size();++i){Vec p=position(originalBoundary[i][0]),q=position(originalBoundary[i][1]);Bounds bb;bb.add(p);bb.add(q);
        const Cell lo=boundaryKey(bb.lo-Vec{2*weld,2*weld,2*weld}),hi=boundaryKey(bb.hi+Vec{2*weld,2*weld,2*weld});
        if(double(hi.x-lo.x+1)*double(hi.y-lo.y+1)*double(hi.z-lo.z+1)>4096){boundaryCells[Cell{INT32_MIN,0,0}].push_back(i);continue;} // huge: always a candidate
        for(int x=lo.x;x<=hi.x;++x)for(int y=lo.y;y<=hi.y;++y)for(int z=lo.z;z<=hi.z;++z)boundaryCells[Cell{x,y,z}].push_back(i);}
    auto onOriginalOpening=[&](const Edge&edge){
        Vec a=position(edge[0]),b=position(edge[1]);
        auto tryEntry=[&](const Edge&entry){Vec p=position(entry[0]),d=position(entry[1])-p;double dd=dot(d,d);if(dd<1e-20)return false;
            auto onSegment=[&](Vec v){double t=dot(v-p,d)/dd;return t>=-1e-8&&t<=1+1e-8&&length(v-(p+d*std::clamp(t,0.,1.)))<=weld;};
            return onSegment(a)&&onSegment(b);};
        {auto it=boundaryCells.find(boundaryKey(a));if(it!=boundaryCells.end())for(auto i:it->second)if(tryEntry(originalBoundary[i]))return true;
         auto huge=boundaryCells.find(Cell{INT32_MIN,0,0});if(huge!=boundaryCells.end())for(auto i:huge->second)if(tryEntry(originalBoundary[i]))return true;}
        Bounds bounds;bounds.add(a);bounds.add(b);bounds.lo=bounds.lo-Vec{weld,weld,weld};bounds.hi=bounds.hi+Vec{weld,weld,weld};
        std::vector<std::uint32_t>nearby;body.query(0,bounds,nearby);
        std::vector<std::pair<double,double>>spans;
        for(auto f:nearby)if(closure(f)){auto&ids=body.source.f[f];
            auto onFace=[&](Vec p){return length(p-closestTriangle(p,body.source.v[ids[0]].p,body.source.v[ids[1]].p,body.source.v[ids[2]].p))<=weld;};
            if(onFace(a)&&onFace(b))return true;
            std::pair<double,double>span;
            if(segmentTriangleInterval(a,b,body.source.v[ids[0]].p,body.source.v[ids[1]].p,body.source.v[ids[2]].p,weld,span))spans.push_back(span);
        }
        return coversSegment(std::move(spans));
    };
    // Authored clothing sheets can already share a nonmanifold edge. A cut
    // may subdivide it; preserve only its exact original face incidence. New
    // cavity faces and repeated faces never qualify for this exception.
    std::map<Edge,std::set<std::uint32_t>> originalIncidence;
    for(std::uint32_t f=0;f<s.f.size();++f)for(int k=0;k<3;++k){auto edge=edgeKeyExact(s.v[s.f[f][k]].p,s.v[s.f[f][(k+1)%3]].p);if(original[edge]>2)originalIncidence[edge].insert(f);}
    std::unordered_map<Edge,std::vector<const JoinedTriangle*>,QuantArrayHash<2>> crowded;
    // Only edges used more than twice, in the former map's key order.
    std::vector<const EdgeCounts::value_type*> irregular;
    for(const auto&item:edgeCounts)if(item.second!=2)irregular.push_back(&item);
    std::sort(irregular.begin(),irregular.end(),[](auto a,auto b){return a->first<b->first;});
    bool anyCrowded=false;for(auto e:irregular)anyCrowded=anyCrowded||e->second>2;
    if(anyCrowded){EdgeCountMap crowdedEdges;for(auto e:irregular)if(e->second>2)crowdedEdges.try_emplace(e->first,e->second);
        for(const auto&t:result.triangles)for(int k=0;k<3;++k){auto edge=edgeKeyExact(t.v[k].p,t.v[(k+1)%3].p);if(crowdedEdges.count(edge))crowded[edge].push_back(&t);}}
    const auto closureIndex=crowded.empty()?std::shared_ptr<const ClosureIndex>{}:envelopeClosureIndex(s,body,weld);
    auto inheritedClosureSheet=[&](const Edge&edge){
        const JoinedTriangle*sheet=nullptr;for(auto*t:crowded.at(edge))if(!t->material){if(sheet)return false;sheet=t;}
        if(!sheet||sheet->v[0].sourceFace>=s.f.size())return false;
        return closureSheetMember(*closureIndex,sourceFaces,surfaceFaceKey(s,s.f[sheet->v[0].sourceFace]))&&onOriginalOpening(edge);
    };
    auto inheritedJunction=[&](const Edge&edge){
        std::set<std::uint32_t> faces;
        for(const auto*t:crowded.at(edge)){
            if(t->material || t->v[0].sourceFace!=t->v[1].sourceFace || t->v[0].sourceFace!=t->v[2].sourceFace || !faces.insert(t->v[0].sourceFace).second)return false;
        }
        for(const auto&candidate:originalIncidence){
            if(!std::includes(candidate.second.begin(),candidate.second.end(),faces.begin(),faces.end()))continue;
            Vec p=position(candidate.first[0]),d=position(candidate.first[1])-p;double dd=dot(d,d);if(dd<1e-20)continue;
            bool on=true;for(auto q:edge){Vec v=position(q);double t=dot(v-p,d)/dd;on=on&&t>=-1e-8&&t<=1+1e-8&&length(v-(p+d*std::clamp(t,0.,1.)))<=weld;}
            if(on)return true;
        }return false;
    };
    // Render vertices are float32. A generated edge shorter than one micron
    // can survive round-off beside a collapsed triangle. Do not move source
    // vertices or reject the entire shot for that render-only residual. Keep
    // a strict length/count budget and verify identical endpoint skin so it
    // cannot expand into a visible crack when bent. This is NOT a declaration
    // of watertight topology: the residual is separately counted and measured.
    stages.mark("gate_setup");
    for(auto entry:irregular){const auto&item=*entry;bool rejected=false;if(item.second>2){if(!inheritedJunction(item.first)){
        if(body.preserveCrossings&&inheritedInteriorSheet(item.first,crowded.at(item.first),body,g.pitch,inheritedClosureSheet(item.first)))++result.interiorSheetJunctions;
        else{++result.nonmanifold;rejected=true;}}}else if(item.second==1){if(onOriginalOpening(item.first))++result.originalOpen;
        else if(body.preserveCrossings&&nativeEdgeCovered(item.first,edgeCounts,skin,1e-6))++result.numericalSeams;
        else if(body.preserveCrossings&&recordNumericalOpening(item.first,skin,s.bones,result)){}
        else{++result.newOpen;rejected=true;}}
#ifdef GFY_CUT_DIAGNOSTICS
        if(rejected){std::cout<<"rejected_edge "<<item.second;for(auto q:item.first)for(auto value:q)std::cout<<' '<<std::setprecision(14)<<value*1e-8;std::cout<<'\n';
            if(item.second>2)for(const auto*t:crowded.at(item.first)){
                std::cout<<"rejected_incident "<<t->material;
                for(const auto&v:t->v)std::cout<<' '<<v.sourceFace;
                for(const auto&v:t->v)std::cout<<' '<<std::setprecision(14)<<v.p.x<<' '<<v.p.y<<' '<<v.p.z;
                std::cout<<'\n';}}
#endif
    }
    stages.mark("gate_validate");
    // Perimeter comes from the intentional pre-join cut boundary only.
    EdgeCounts outer;outer.reserve(exteriorMesh.polygons.size()*8);std::unordered_map<Quant,Vec,QuantHash>coords;
    for(auto&f:exteriorMesh.polygons)for(std::size_t i=0;i<f.polygon.size();++i){Vec a=f.polygon[i].p,b=f.polygon[(i+1)%f.polygon.size()].p;++outer[edgeKeyExact(a,b)];coords[quantExact(a)]=a;coords[quantExact(b)]=b;}
    // Sum in the former std::map key order: identical floating result.
    std::vector<const Edge*>single;for(auto&item:outer)if(item.second==1)single.push_back(&item.first);
    std::sort(single.begin(),single.end(),[](auto a,auto b){return *a<*b;});
    for(auto e:single){auto found=original.find(*e);if(!(found!=original.end()&&found->second==1))result.cutPerimeter+=length(coords[(*e)[1]]-coords[(*e)[0]]);}
    stages.mark("perimeter");
    return result;
}
// Output of joinIndexed: the SAME triangles, vertices and statistics as
// join(), with corners referencing a vertex store (stats.triangles is empty).
// Extra joined vertices: address-stable blocks (push_back/back/size only). std::deque held one 656-byte vertex per heap
// block on MSVC: an allocation per wall vertex.
struct JoinedVertexStore {
    static constexpr std::size_t B=256;std::vector<std::unique_ptr<JoinedVertex[]>> blocks;std::size_t n=0;
    void push_back(const JoinedVertex& v){if(n%B==0)blocks.emplace_back(new JoinedVertex[B]);blocks.back()[n%B]=v;++n;}
    JoinedVertex& back(){return blocks.back()[(n-1)%B];}std::size_t size()const{return n;}bool empty()const{return n==0;}
    JoinedVertex& operator[](std::size_t i){return blocks[i/B][i%B];}const JoinedVertex& operator[](std::size_t i)const{return blocks[i/B][i%B];}
};
struct IndexedJoin {std::vector<JoinedVertex> vertices;JoinedVertexStore extra;std::vector<IndexedTriangle> triangles;JoinResult stats;};
// Region joins of tile-local exterior pieces (FastWound_impl.h): canonical
// lattice vertices keep their positions and skin, and edges shared with pieces
// emitted outside this join are legitimate openings of this join.
// The render weld's original-position set and index of a source's vertices:
// identical for every join of an asset, so built once (asset preparation).
struct WeldBase {FlatMap<Quant,char,QuantHash> positions;WeldIndex welded;};
inline std::shared_ptr<const WeldBase> buildWeldBase(const Source& s){constexpr double weld=3e-7;
    auto base=std::make_shared<WeldBase>();base->positions.reserve(s.v.size());
    for(auto&v:s.v)if(base->positions.try_emplace(quantExact(v.p),0).second)base->welded.insert(Cell{int(std::floor(v.p.x/weld)),int(std::floor(v.p.y/weld)),int(std::floor(v.p.z/weld))},v.p);
    return base;}
struct JoinExtras {const WeldBase* weldBase{}; // the source's weld set/index, prebuilt (else built per join)
    const std::vector<Vec>*fixedPositions{};const std::unordered_map<Quant,std::array<double,64>,QuantHash>*fixedSkin{};std::function<bool(Vec,Vec)> extraOpening;
    // Chunked regions: true for a point on a chunk seam plane, with its position-only skin weights (seam seeds).
    std::function<bool(Vec,std::array<double,64>&)> seamSkin;
    // Chunked regions: position-only skin weights of any point (seeds of wall pockets without skin or seam).
    std::function<bool(Vec,std::array<double,64>&)> anySkin;};
IndexedJoin joinIndexed(const Source&s,const BodyBVH&body,const Grid&g,const ClipResult&exteriorMesh,const std::vector<Quad>&quads,const JoinExtras*extras=nullptr){
    StageProfile stages("join");IndexedJoin indexed;JoinResult&result=indexed.stats;auto cavity=cavityPolygons(body,quads);stages.mark("cavity_polygons");result.cavityPolygons=cavity.size();std::vector<JoinedPolygon>polygons;polygons.reserve(exteriorMesh.polygons.size()+cavity.size());CanonicalMap canonical;
    {std::size_t corners=0;for(const auto&f:exteriorMesh.polygons)corners+=f.polygon.size();for(const auto&poly:cavity)corners+=poly.size();canonical.reserve(corners);}
    for(auto&fragment:exteriorMesh.polygons){JoinedPolygon out;out.v.reserve(fragment.polygon.size());for(auto&v:fragment.polygon){auto j=interpolateVertex(s,fragment.sourceFace,v);canonical.try_emplace(quantExact(j.p),j);out.v.push_back(j);}polygons.push_back(std::move(out));}
    // Tile-local: a cavity rim point computed from the quads can differ from the
    // exterior cut point it meets by a snap quantum (1e-8 m) when the exact value
    // sits on a rounding boundary; move it onto that exterior point (within 3e-8 m,
    // the conform tolerance) so both share one key, one skin seed and one vertex.
    if(extras)stages.mark("exterior_vertices");
    if(extras){constexpr double tol=3e-8,cell=1e-7;
        auto key=[&](Vec p){return Cell{int(std::floor(p.x/cell)),int(std::floor(p.y/cell)),int(std::floor(p.z/cell))};};
        // Exterior points by 1e-7 m cell: head index per cell, chained (no per-cell vectors).
        std::vector<Vec> pts;pts.reserve(canonical.size());std::vector<std::uint32_t> chain;chain.reserve(canonical.size());
        FlatMap<Cell,std::uint32_t,CellHash> head;head.reserve(canonical.size());
        for(const auto&item:canonical){const auto id=std::uint32_t(pts.size());pts.push_back(item.second.p);auto [it,fresh]=head.try_emplace(key(item.second.p),id);chain.push_back(fresh?~0u:it->second);if(!fresh)it->second=id;}
        // Only points off the cell lattice can be rim points (quad corners have all
        // three coordinates on cell planes): a cheap arithmetic filter first.
        auto onLattice=[&](Vec p){for(int a=0;a<3;++a){const double u=(p[a]-g.origin[a])/g.pitch;if(std::abs(u-std::round(u))>1e-6)return false;}return true;};
        for(auto&poly:cavity)for(auto&v:poly){if(onLattice(v.p)||canonical.count(quantExact(v.p)))continue;const Cell c=key(v.p);double best=tol;const Vec*hit=nullptr;
            for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)for(int dz=-1;dz<=1;++dz){auto it=head.find(Cell{c.x+dx,c.y+dy,c.z+dz});if(it==head.end())continue;
                for(auto id=it->second;id!=~0u;id=chain[id]){const double d=length(pts[id]-v.p);if(d<=best){best=d;hit=&pts[id];}}}
            if(hit)v.p=*hit;}
        stages.mark("cs_rim_snap");}
    // Chunked regions: every cavity vertex on a chunk seam plane seeds the cavity skin with position-only
    // weights, identical in both chunks joining that seam (no crack when posed, no unseeded component).
    FlatMap<Quant,char,QuantHash> seamSeeds; // position-only seeds (seams, pockets): may sit on an authored edge
    if(extras&&extras->seamSkin){for(auto&poly:cavity)for(auto&v:poly){const auto q=quantExact(v.p);if(canonical.count(q))continue;
            std::array<double,64> w{};if(!extras->seamSkin(v.p,w))continue;JoinedVertex j;j.p=v.p;j.w=w;j.sourceFace=~std::uint32_t(0);canonical.try_emplace(q,j);seamSeeds.try_emplace(q,0);}
        stages.mark("cs_seam_seeds");}
    CanonicalMap cavitySkin;
    try {cavitySkin=assignCavitySkin(cavity,canonical);}
    catch(const CavityProvenanceError&e){
        stages.mark("cs_attempt1");
        if(std::string(e.what())!="cavity_component_without_exterior_skin_seed")throw;
        conformCavityBoundary(cavity,canonical,g);stages.mark("cs_conform");
        // Attempts 2 and 3 see identical polygons (seedEnvelopeOpenings only
        // adds canonical seeds), so they share one graph of this cavity.
        const auto graph=buildCavityGraph(cavity);stages.mark("cs_graph");
        try {cavitySkin=assignCavitySkin(graph,canonical);}
        catch(const CavityProvenanceError&again){
            stages.mark("cs_attempt2");
            const bool seedError=std::string(again.what())=="cavity_component_without_exterior_skin_seed";if(!seedError)throw;
            bool solved=false;
            if(body.preserveCrossings&&seedEnvelopeOpenings(cavity,canonical,s,body)){stages.mark("cs_seed");
                try{cavitySkin=assignCavitySkin(graph,canonical);solved=true;}
                catch(const CavityProvenanceError&third){if(!(extras&&extras->anySkin)||std::string(third.what())!="cavity_component_without_exterior_skin_seed")throw;}}
            if(!solved){
                // Chunked region: a wall pocket reaching neither skin nor a seam inside this chunk takes
                // position-only skin weights at every vertex (the rule the seams use), instead of rejecting.
                if(!(extras&&extras->anySkin))throw;
                std::vector<char> reached(graph.points.size(),0);std::vector<int> pending;
                for(const auto&item:graph.index)if(canonical.count(item.first)){reached[item.second]=1;pending.push_back(item.second);}
                for(std::size_t k=0;k<pending.size();++k)for(const auto&e:graph.edges[pending[k]])if(!reached[e.first]){reached[e.first]=1;pending.push_back(e.first);}
                for(const auto&item:graph.sorted)if(!reached[item.second]){std::array<double,64> w{};if(!extras->anySkin(graph.points[item.second],w))continue;
                    JoinedVertex jv;jv.p=graph.points[item.second];jv.w=w;jv.sourceFace=~std::uint32_t(0);canonical.try_emplace(item.first,jv);seamSeeds.try_emplace(item.first,0);}
                stages.mark("cs_pocket_seeds");cavitySkin=assignCavitySkin(graph,canonical);}
        }
    }
    stages.mark("cavity_skin");
    auto nearest=[&](Vec p){JoinedVertex out;auto found=cavitySkin.find(quantExact(p));
        if(found==cavitySkin.end())throw CavityProvenanceError("cavity_vertex_not_in_component");
        out=found->second;out.p=p;out.uv={p.x/.03,p.y/.03};
        canonical.try_emplace(quantExact(p),out);return out;};
    for(auto&poly:cavity){cancelCavityBacktracks(poly);if(poly.size()<3)continue;JoinedPolygon out;out.material=poly.front().material;out.v.reserve(poly.size());
        // Plane intersections are snapped to a 10 nm lattice. A line can
        // therefore return as a narrow polygon and its fan creates spurious
        // overlapping triangles. Cull only polygons indistinguishable from
        // a line at that precision; the final boundary gate still requires
        // the remaining skin and cavity edges to close exactly.
        Vec lineA=poly.front().p,lineB=lineA;double longest=0;
        for(const auto&v:poly){double d=length(v.p-lineA);if(d>longest){longest=d;lineB=v.p;}}
        if(longest>0){double width=0;for(const auto&v:poly)width=std::max(width,length(cross(v.p-lineA,lineB-lineA))/longest);
            if(width<=3e-8)continue;}
        Vec normal;for(std::size_t k=1;k+1<poly.size();++k)normal=normal+cross(poly[k].p-poly[0].p,poly[k+1].p-poly[0].p);
        double norm=length(normal);if(norm<1e-16)throw CavityProvenanceError("invalid_cavity_normal");normal=normal*(1/norm);
        Vec tangent=std::abs(normal.x)>.9?Vec{0,0,1}:Vec{1,0,0};Vec binormal=cross(normal,tangent);
        for(auto&v:poly){auto vertex=nearest(v.p);vertex.normal=normal;vertex.tangent=tangent;vertex.binormal=binormal;vertex.sourceFace=~std::uint32_t(0);vertex.sourceBary={};vertex.uv={dot(vertex.p,tangent)/.03,dot(vertex.p,binormal)/.03};out.v.push_back(vertex);}polygons.push_back(std::move(out));}
    // Canonical endpoints carry one skin solution across material/UV seams.
    // A spatial bucket visits only points near an edge, avoiding a global O(VE)
    // search while conforming partial shared voxel and exterior edge segments.
    stages.mark("cavity_vertices");
    JoinBuckets buckets;
    {std::vector<const CanonicalMap::value_type*> ordered;ordered.reserve(canonical.size());for(const auto&item:canonical)ordered.push_back(&item);
     std::sort(ordered.begin(),ordered.end(),[](auto a,auto b){return a->first<b->first;});
     buckets.reserve(ordered.size());for(auto item:ordered)buckets[joinBucket(g.origin,g.pitch,item->second.p)].push_back(&item->second);}
    // Each polygon's conformed edges depend only on the (now read-only)
    // canonical buckets: chunks of polygons run concurrently, in place.
    parallelChunks(polygons.size(),1024,[&](std::size_t firstPoly,std::size_t lastPoly,std::size_t){
    std::vector<std::pair<double,const JoinedVertex*>> hits;
    for(std::size_t polyIndex=firstPoly;polyIndex<lastPoly;++polyIndex){auto&poly=polygons[polyIndex];
        // Most polygons gain no point: only rebuild those that do.
        std::vector<JoinedVertex>out;bool grown=false;
        for(std::size_t i=0;i<poly.v.size();++i){const auto&a=poly.v[i];const auto&b=poly.v[(i+1)%poly.v.size()];
            Vec d=b.p-a.p;double dd=dot(d,d);if(dd<1e-20){if(grown)out.push_back(a);continue;}
            conformEdgeHits(buckets,g.origin,g.pitch,a,b,hits);
            if(!hits.empty()&&!grown){grown=true;out.reserve(poly.v.size()+hits.size()+4);out.insert(out.end(),poly.v.begin(),poly.v.begin()+std::ptrdiff_t(i));}
            if(grown){out.push_back(a);for(const auto&h:hits)out.push_back(conformedPoint(*h.second,a,b,h.first));}}
        if(grown)poly.v=std::move(out);}});
    stages.mark("conform_polygons");
    // Same fan triangulation; corners point into one vertex store instead of
    // copying three 656-byte vertices per triangle.
    auto&store=indexed.vertices;std::vector<IndexedTriangle> triangles;
    {std::size_t vertexCount=0,expected=0;for(const auto&poly:polygons){vertexCount+=poly.v.size()+(poly.v.size()==3?0:1);expected+=poly.v.size()==3?1:poly.v.size();}store.reserve(vertexCount);triangles.reserve(expected);}
    for(auto&poly:polygons){if(poly.v.size()==3){JoinedVertex*a=&store.emplace_back(poly.v[0]);JoinedVertex*b=&store.emplace_back(poly.v[1]);JoinedVertex*c=&store.emplace_back(poly.v[2]);triangles.push_back({poly.material,{a,b,c}});continue;}
        JoinedVertex center;for(auto&v:poly.v){center.p=center.p+v.p;center.normal=center.normal+v.normal;center.tangent=center.tangent+v.tangent;center.binormal=center.binormal+v.binormal;center.sourceFace=poly.v[0].sourceFace;for(int k=0;k<3;++k)center.sourceBary[k]+=v.sourceBary[k];for(int uv=0;uv<2;++uv)center.uv[uv]+=v.uv[uv];for(std::uint32_t bone=0;bone<s.bones;++bone)center.w[bone]+=v.w[bone];}double inv=1./double(poly.v.size());center.p=center.p*inv;center.normal=center.normal*inv;center.tangent=center.tangent*inv;center.binormal=center.binormal*inv;for(auto&b:center.sourceBary)b*=inv;for(auto&uv:center.uv)uv*=inv;for(auto&w:center.w)w*=inv;
        const std::size_t n=poly.v.size();
#ifdef GFY_CS_CORNER_FANS
        // PROTOTYPE (off by default; changes cavity triangulation, not exact):
        // Cavity polygons (convex voxel-face pieces with collinear T points)
        // fan from a corner when every fan triangle is well shaped (altitude
        // >= 1 um, same orientation): n-2 triangles and no averaged centre
        // vertex. Otherwise, and always for exterior skin, the centre fan.
        if(poly.material&&n>=4){
            Vec normal;for(std::size_t k=1;k+1<n;++k)normal=normal+cross(poly.v[k].p-poly.v[0].p,poly.v[k+1].p-poly.v[0].p);
            std::size_t root=n;
            for(std::size_t r=0;r<n&&root==n;++r){bool ok=true;const Vec o=poly.v[r].p;
                for(std::size_t i=1;ok&&i+1<n;++i){const Vec a=poly.v[(r+i)%n].p,b=poly.v[(r+i+1)%n].p;const Vec c=cross(a-o,b-o);
                    const double longest=std::max({length(a-o),length(b-o),length(b-a)});
                    ok=dot(c,normal)>0&&longest>0&&length(c)/longest>=1e-6;}
                if(ok)root=r;}
            if(root<n){const std::size_t first=store.size();for(auto&v:poly.v)store.emplace_back(v);
                for(std::size_t i=1;i+1<n;++i)triangles.push_back({poly.material,{&store[first+root],&store[first+(root+i)%n],&store[first+(root+i+1)%n]}});
                continue;}
        }
#endif
        // Tile-local exterior only: a fragment whose centre fan would fold (a
        // fan triangle against the polygon's orientation: micrometre slivers
        // beside a lattice line) is ear-clipped instead, from its own vertices.
        if(extras&&n>=4){
            Vec normal;for(std::size_t k=0;k<n;++k){const Vec a=poly.v[k].p,b=poly.v[(k+1)%n].p;normal=normal+Vec{(a.y-b.y)*(a.z+b.z),(a.z-b.z)*(a.x+b.x),(a.x-b.x)*(a.y+b.y)};}
            bool folds=false;for(std::size_t i=0;i<n&&!folds;++i){const Vec a=poly.v[i].p,b=poly.v[(i+1)%n].p;const Vec t=cross(a-center.p,b-center.p);if(length(t)>=1e-16&&dot(t,normal)<=0)folds=true;}
            if(folds){int ax=0;for(int a=1;a<3;++a)if(std::abs(normal[a])>std::abs(normal[ax]))ax=a;int u=(ax+1)%3,w=(ax+2)%3;if(normal[ax]<0)std::swap(u,w);
                std::vector<::polytri::P2> pts;std::vector<std::size_t> vertexOf;FlatMap<Quant,std::uint32_t,QuantHash> ids;std::vector<std::uint32_t> ring;
                for(std::size_t i=0;i<n;++i){const auto q=quantExact(poly.v[i].p);auto it=ids.find(q);std::uint32_t id;
                    if(it==ids.end()){id=std::uint32_t(pts.size());ids.emplace(q,id);pts.push_back({poly.v[i].p[u]-poly.v[0].p[u],poly.v[i].p[w]-poly.v[0].p[w]});vertexOf.push_back(i);}else id=it->second;
                    ring.push_back(id);}
                std::vector<::polytri::DEdge> edges;for(std::size_t i=0;i<n;++i){const auto x=ring[i],y=ring[(i+1)%n];if(x!=y)edges.push_back({x,y});}
                // Opposite edge pairs (a spike) cancel before loop extraction.
                {std::vector<::polytri::DEdge> kept;for(const auto&e:edges){auto r=std::find(kept.begin(),kept.end(),::polytri::DEdge{e.second,e.first});if(r!=kept.end())kept.erase(r);else kept.push_back(e);}edges=std::move(kept);}
                std::vector<::polytri::Tri> tris;std::string error;
                auto floatPositive=[&](const std::vector<::polytri::Tri>& ts){for(const auto&t:ts){Vec q[3];for(int j=0;j<3;++j){const Vec p=poly.v[vertexOf[t[j]]].p;q[j]=Vec{double(float(p.x)),double(float(p.y)),double(float(p.z))};}
                    if(dot(cross(q[1]-q[0],q[2]-q[0]),normal)<=0)return false;}return true;};
                if(!edges.empty()&&::polytri::triangulate(pts,edges,3e-8,tris,error)&&floatPositive(tris)){const std::size_t first=store.size();for(auto i:vertexOf)store.emplace_back(poly.v[i]);
                    for(const auto&t:tris)triangles.push_back({poly.material,{&store[first+t[0]],&store[first+t[1]],&store[first+t[2]]}});
                    ++indexed.stats.earClippedFragments;continue;}}}
        JoinedVertex*c=&store.emplace_back(center);const std::size_t first=store.size();
        for(auto&v:poly.v)store.emplace_back(v);
        for(std::size_t i=0;i<n;++i){JoinedVertex*a=&store[first+i];JoinedVertex*b=&store[first+(i+1)%n];if(length(cross(a->p-c->p,b->p-c->p))<1e-16)continue;triangles.push_back({poly.material,{c,a,b}});}}
    // Joined polygons, buckets and the vertex maps are dead from here on; release
    // them before the native stages rather than at return.
    std::vector<JoinedPolygon>().swap(polygons);buckets=JoinBuckets();
    canonical=CanonicalMap();cavitySkin=CanonicalMap();
    // The renderer stores float positions. Validate that actual representation,
    // removing triangles collapsed by conversion rather than treating their
    // nanometre sliver edges as a four-face surface. Never bridge a visible gap.
    stages.mark("triangulate");
    // Lookup-only key maps (never iterated): hashed, same first-wins semantics.
    FlatMap<Quant,const std::array<double,64>*,QuantHash> nativeWeights;
    FlatMap<Quant,Quant,QuantHash> nativeOriginalPosition;
    FlatMap<Quant,std::uint32_t,QuantHash> nativeFirstSourceFace;
    FlatMap<std::array<Quant,3>,char,QuantArrayHash<3>> nativeFaces;nativeFaces.reserve(triangles.size()); // polygons were already released above
    constexpr double weld=3e-7; // <0.001 of even the High voxel edge.
    auto weldBucket=[weld](Vec p){return Cell{int(std::floor(p.x/weld)),int(std::floor(p.y/weld)),int(std::floor(p.z/weld))};};
    WeldIndex welded;
    // Original model vertices have priority and retain their exact positions.
    const WeldBase* weldBase=extras?extras->weldBase:nullptr;
    FlatMap<Quant,char,QuantHash> originalPositions;
    auto isOriginal=[&](const Quant& q){return (weldBase&&weldBase->positions.count(q))||originalPositions.count(q);};
    auto addOriginal=[&](const Quant& q){return !(weldBase&&weldBase->positions.count(q))&&originalPositions.try_emplace(q,0).second;};
    if(!weldBase){originalPositions.reserve(s.v.size());for(auto&v:s.v){if(originalPositions.try_emplace(quantExact(v.p),0).second)welded.insert(weldBucket(v.p),v.p);}}
    // Tile-lattice vertices shared with statically emitted pieces never move.
    if(extras&&extras->fixedPositions)for(const auto&p0:*extras->fixedPositions){const Vec p{double(float(p0.x)),double(float(p0.y)),double(float(p0.z))};
        if(addOriginal(quantExact(p)))welded.insert(weldBucket(p),p);}
    stages.mark("np_original_positions");
    // A query that returned itself (an original position, or a new weld
    // representative) returns itself again: nothing inserted later can be
    // closer than distance 0 or precede it with an equal distance.
    struct BitsHash{std::size_t operator()(const std::array<std::uint64_t,3>&k)const{return std::size_t(quantMix(k[0]^quantMix(k[1]^quantMix(k[2]))));}};
    FlatMap<std::array<std::uint64_t,3>,char,BitsHash> selfCanonical;selfCanonical.reserve(store.size());
    auto bits=[](Vec p){std::array<std::uint64_t,3>k;std::memcpy(k.data(),&p,24);return k;};
    auto canonicalPositionSlow=[&](Vec p){
        if(isOriginal(quantExact(p)))return p;
        Cell c=weldBucket(p);double best=weld*weld;Vec result=p;const bool found=weldBase?welded.nearest(weldBase->welded,c,p,best,result):welded.nearest(c,p,best,result);
        if(!found)welded.insert(c,p);return result;
    };
    auto canonicalPosition=[&](Vec p){auto key=bits(p);if(selfCanonical.count(key))return p;
        Vec out=canonicalPositionSlow(p);if(!std::memcmp(&out,&p,sizeof(Vec)))selfCanonical.emplace(key,0);return out;};
    // A clipped vertex on an authored source edge must use that edge's source
    // weights. Independent triangle clipping can leave a tiny third-vertex
    // barycentric term, even though both source faces share the same edge.
    // This only applies to an exact source-face edge within 1 nm of the
    // renderer's already canonicalized position; it cannot invent a seam.
    auto authoredEdgeWeights=[&](const JoinedVertex&v,std::array<double,64>&out){
        if(v.sourceFace>=s.f.size())return false;
        const auto&face=s.f[v.sourceFace];double nearest=1e-18;int choice=-1;double fraction=0;
        for(int edge=0;edge<3;++edge){auto a=s.v[face[edge]].p,b=s.v[face[(edge+1)%3]].p;
            Vec d=b-a;double dd=dot(d,d);if(dd<=1e-20)continue;
            double t=dot(v.p-a,d)/dd;if(t<0||t>1)continue;
            double distance2=dot(v.p-(a+d*t),v.p-(a+d*t));
            if(distance2<nearest){nearest=distance2;choice=edge;fraction=t;}}
        if(choice<0)return false;
        out={};
        const auto&from=s.v[face[choice]],&to=s.v[face[(choice+1)%3]];
        for(int k=0;k<4;++k){out[from.ids[k]]+=(1-fraction)*from.w[k];out[to.ids[k]]+=fraction*to.w[k];}
        return true;
    };
    // Prepared triangles are compacted in place (slot index <= source index,
    // same order); their unwelded keys run in a parallel vector.
    // Every corner of a shared vertex gets the same float/weld result: the
    // first corner (in triangle order, exactly where the former per-corner
    // call happened) computes it. A result equal to its input is stable (the
    // former self-canonical memo); otherwise each later corner recomputes it,
    // which has no side effect, and splits off its own copy if it differs.
    std::vector<std::array<Quant,3>>preparedUnwelded;preparedUnwelded.reserve(triangles.size());
    std::vector<IndexedTriangle> prepared;prepared.reserve(triangles.size());
    struct VertexState {Quant unwelded{};Vec input;bool processed=false,stable=false;};
    std::vector<VertexState> vertexState(store.size());
    for(auto&triangle:triangles){std::array<Quant,3> keys,unwelded;
        for(int k=0;k<3;++k){JoinedVertex*v=triangle.v[k];auto&state=vertexState[std::size_t(v-store.data())];
            if(!state.processed){state.processed=true;state.unwelded=quantExact(v->p);state.input={double(float(v->p.x)),double(float(v->p.y)),double(float(v->p.z))};
                const Vec p=canonicalPosition(state.input);state.stable=!std::memcmp(&p,&state.input,sizeof(Vec));v->p=p;}
            else if(!state.stable){const Vec p=canonicalPosition(state.input);
                if(std::memcmp(&p,&v->p,sizeof(Vec))){indexed.extra.push_back(*v);indexed.extra.back().p=p;triangle.v[k]=&indexed.extra.back();}}
            unwelded[k]=state.unwelded;keys[k]=quantExact(triangle.v[k]->p);}
        if(keys[0]==keys[1]||keys[1]==keys[2]||keys[2]==keys[0])continue;
        if(length(cross(triangle.v[1]->p-triangle.v[0]->p,triangle.v[2]->p-triangle.v[0]->p))<1e-16)continue;
        auto face=keys;std::sort(face.begin(),face.end());if(!nativeFaces.try_emplace(face,0).second)continue;
        prepared.push_back(triangle);preparedUnwelded.push_back(unwelded);
    }
    std::vector<IndexedTriangle>().swap(triangles);
    stages.mark("native_prepare");
    // Collect the original edge solution before visiting cavity polygons: a
    // cavity vertex may be emitted earlier than the source face at that key.
    std::unordered_map<Quant,std::array<double,64>,QuantHash>edgeSkin;
    for(const auto&triangle:prepared)for(int corner=0;corner<3;++corner)if(const auto&v=vx(triangle,corner);true){
        std::array<double,64>value;if(!authoredEdgeWeights(v,value))continue;
        auto[it,fresh]=edgeSkin.try_emplace(quantExact(v.p),value);
        if(!fresh)for(unsigned bone=0;bone<s.bones;++bone)
            if(std::abs(value[bone]-it->second[bone])>1e-6)
                throw CavityProvenanceError("authored_edge_skin_conflict");
    }
    // Float conversion can displace the same authored edge by more than the
    // 1 nm single-face test above. Admit a slightly wider correction ONLY when
    // two source faces at this exact rendered key share both original vertex
    // IDs. This is source topology evidence, not a nearby-bone projection.
    // (key, face) -> first corner's weights/unwelded key; first position per
    // key. A stable sort by (key, face) over corners in scan order reproduces
    // the former nested std::map (first emplace wins, ascending faces).
    struct FaceCorner {Quant key;std::uint32_t face;const std::array<double,64>*weights;Quant unwelded;};
    std::vector<FaceCorner> corners;FlatMap<Quant,Vec,QuantHash> positionAtKey;
    for(std::size_t index=0;index<prepared.size();++index)for(int corner=0;corner<3;++corner){const auto&v=vx(prepared[index],corner);
        if(v.sourceFace<s.f.size()){const auto key=quantExact(v.p);
            corners.push_back({key,v.sourceFace,&v.w,preparedUnwelded[index][corner]});
            positionAtKey.try_emplace(key,v.p);}}
    std::stable_sort(corners.begin(),corners.end(),[](const FaceCorner&a,const FaceCorner&b){return a.key<b.key||(a.key==b.key&&a.face<b.face);});
    corners.erase(std::unique(corners.begin(),corners.end(),[](const FaceCorner&a,const FaceCorner&b){return a.key==b.key&&a.face==b.face;}),corners.end());
    for(std::size_t groupBegin=0;groupBegin<corners.size();){std::size_t groupEnd=groupBegin+1;while(groupEnd<corners.size()&&corners[groupEnd].key==corners[groupBegin].key)++groupEnd;
        const std::size_t groupSize=groupEnd-groupBegin;const std::size_t firstIndex=groupBegin;groupBegin=groupEnd;
        if(groupSize<2||groupSize>8)continue;
        const Quant itemKey=corners[firstIndex].key;const auto p=positionAtKey.at(itemKey);
        for(std::size_t i=firstIndex;i<firstIndex+groupSize;++i)for(std::size_t j=i+1;j<firstIndex+groupSize;++j){
            const auto&first=corners[i];const auto&second=corners[j];
            if(first.unwelded!=second.unwelded)continue;
            double disagreement=0;
            for(unsigned bone=0;bone<s.bones;++bone)disagreement=std::max(disagreement,
                std::abs((*first.weights)[bone]-(*second.weights)[bone]));
            if(disagreement<=1e-6||disagreement>1e-5)continue;
            std::array<std::uint32_t,2>shared{};unsigned count=0;
            for(auto a:s.f[first.face])for(auto b:s.f[second.face])if(a==b&&count<2)shared[count++]=a;
            if(count!=2)continue;
            const auto&from=s.v[shared[0]],&to=s.v[shared[1]];Vec d=to.p-from.p;
            const double dd=dot(d,d);if(dd<=1e-20)continue;
            const double t=dot(p-from.p,d)/dd;if(t<0||t>1)continue;
            // At yâ‰ˆ1.36 m, half an IEEE float ULP is 59.6 nm. Include that
            // unavoidable renderer rounding, with exact shared source IDs.
            if(dot(p-(from.p+d*t),p-(from.p+d*t))>4.9e-15)continue; // 70 nm.
            std::array<double,64>value{};
            for(int k=0;k<4;++k){value[from.ids[k]]+=(1-t)*from.w[k];value[to.ids[k]]+=t*to.w[k];}
            auto[it,fresh]=edgeSkin.emplace(itemKey,value);
            if(!fresh)for(unsigned bone=0;bone<s.bones;++bone)
                if(std::abs(value[bone]-it->second[bone])>1e-6)
                    throw CavityProvenanceError("authored_shared_edge_skin_conflict");
        }
    }
    stages.mark("edge_skin");
    nativeWeights.reserve(prepared.size());nativeOriginalPosition.reserve(prepared.size());nativeFirstSourceFace.reserve(prepared.size());
    for(std::size_t index=0;index<prepared.size();++index){auto&triangle=prepared[index];const auto&unwelded=preparedUnwelded[index];
        for(int k=0;k<3;++k){auto&v=*triangle.v[k];auto key=quantExact(v.p);
            auto sourceSkin=edgeSkin.find(key);if(sourceSkin!=edgeSkin.end()){
                // A position-only seed (seam or pocket) on an authored edge takes the edge's skin: every
                // chunk sharing it applies the same rule, so both sides stay identical.
                // Chunked regions: seam seeds shift the harmonic cavity skin by far less than a visible
                // amount (observed <1e-4); the vertex takes the edge's skin below either way.
                const double jumpLimit=extras&&extras->seamSkin?5e-3:1e-5;
                /* authored edge skin: adopt the source skin below (a rough-cut disagreement no longer cancels the surface) */
                if(!v.hardware_corner)v.w=sourceSkin->second;
            }
            // Tile-local: a vertex at a canonical lattice point (an exterior
            // vertex there) takes the canonical skin -- bit-identical with the
            // static pieces sharing it -- when it agrees to 1e-6 (same skin, up
            // to float rounding of the authored-edge parameter).
            if(extras&&extras->fixedSkin){auto fixed=extras->fixedSkin->find(key);
                if(fixed!=extras->fixedSkin->end()){double d=0;for(unsigned bone=0;bone<s.bones;++bone)d=std::max(d,std::abs(v.w[bone]-fixed->second[bone]));if(d<=1e-6&&!v.hardware_corner)v.w=fixed->second;}}
            auto [it,fresh]=nativeWeights.try_emplace(key,&v.w);
            if(fresh){nativeOriginalPosition.emplace(key,unwelded[k]);
                nativeFirstSourceFace.emplace(key,v.sourceFace);}
            else{
                // Collapsing a generated edge merges its skin interpolation
                // too. Exact coincident vertices must still agree beforehand;
                // only a bounded, nonzero spatial weld may unify two solutions.
                if(nativeOriginalPosition.at(key)==unwelded[k])for(unsigned bone=0;bone<s.bones;++bone)
                    if(std::abs(v.w[bone]-(*it->second)[bone])>1e-6){
#ifdef GFY_CUT_DIAGNOSTICS
                        std::cout<<"weight_conflict "<<std::setprecision(14)<<v.p.x<<' '<<v.p.y<<' '<<v.p.z
                                 <<' '<<unwelded[k][0]*1e-8<<' '<<unwelded[k][1]*1e-8<<' '<<unwelded[k][2]*1e-8
                                 <<' '<<triangle.material<<' '<<v.sourceFace<<' '<<nativeFirstSourceFace.at(key)<<' '<<bone
                                 <<' '<<v.w[bone]<<' '<<(*it->second)[bone]<<'\n';
                        for(auto faceId:{v.sourceFace,nativeFirstSourceFace.at(key)}){
                            if(faceId>=s.f.size())continue;
                            std::cout<<"conflict_source_face "<<faceId;
                            for(auto index:s.f[faceId]){
                                const auto&sv=s.v[index];
                                std::cout<<' '<<index<<' '<<sv.p.x<<' '<<sv.p.y<<' '<<sv.p.z;
                                for(int j=0;j<4;++j)std::cout<<' '<<sv.ids[j]<<':'<<sv.w[j];
                            }
                            std::cout<<'\n';
                        }
#endif
                        throw CavityProvenanceError("native_float_seam_weight_conflict");}
                if(!v.hardware_corner)v.w=*it->second;}}
    }
    // Every prepared triangle is kept (or the loop threw): take the storage.
    std::vector<IndexedTriangle> nativeTriangles=std::move(prepared);
    stages.mark("native_weights");
    EdgeCountMap retainedCounts;
    result.numericalSpurs=removeFloatCavitySpurs(nativeTriangles,weld,body.preserveCrossings,&retainedCounts);stages.mark("spurs_float");
    if(body.preserveCrossings)result.numericalSpurs+=removeSubmicronSpurs(nativeTriangles,1e-6,&retainedCounts);stages.mark("spurs_submicron");
    // Exactly the edge counts of the final triangles once emptied keys go.
    {EdgeCountMap retained;retained.reserve(retainedCounts.size());for(const auto&e:retainedCounts)if(e.second)retained.try_emplace(e.first,e.second);retainedCounts=std::move(retained);}
    // Tile-local regions: repair rim T-junctions. A cavity rim vertex can lie inside an exterior cut edge (the
    // skin is split into lattice pieces at tile planes, the wall is clipped at its own envelope points). The
    // one triangle owning such an open edge is split at that vertex; the new corner interpolates the edge (exact
    // skin, UV, provenance) and the rim vertex takes the same skin, so both sides share one row.
    if(extras){
        for(int round=0;round<4;++round){
            auto at=[](const Quant& q){return Vec{q[0]*1e-8,q[1]*1e-8,q[2]*1e-8};};
            // Only edges the gate would reject: seams and static-piece openings are open by construction.
            std::vector<Edge> open;for(const auto&e:retainedCounts)if(e.second==1&&!(extras->extraOpening&&extras->extraOpening(at(e.first[0]),at(e.first[1]))))open.push_back(e.first);
            if(open.empty())break;
            std::sort(open.begin(),open.end());
            FlatMap<Edge,char,QuantArrayHash<2>> openSet;openSet.reserve(open.size());for(const auto&e:open)openSet.try_emplace(e,0);
            FlatMap<Edge,std::pair<std::size_t,int>,QuantArrayHash<2>> owner;owner.reserve(open.size());
            constexpr double cell=5e-5;auto cellOf=[&](Vec p){return Cell{int(std::floor(p.x/cell)),int(std::floor(p.y/cell)),int(std::floor(p.z/cell))};};
            FlatMap<Cell,std::vector<JoinedVertex*>,CellHash> points;FlatMap<Quant,char,QuantHash> seenPoint;
            FlatMap<Quant,std::vector<JoinedVertex*>,QuantHash> copies;bool copiesBuilt=false; // every vertex object at a position (built on the first split)
            for(std::size_t t=0;t<nativeTriangles.size();++t)for(int k=0;k<3;++k){
                const auto e=edgeKeyExact(nativeTriangles[t].v[k]->p,nativeTriangles[t].v[(k+1)%3]->p);if(!openSet.count(e))continue;owner.try_emplace(e,std::make_pair(t,k));
                for(int j:{k,(k+1)%3}){JoinedVertex*v=nativeTriangles[t].v[j];if(seenPoint.try_emplace(quantExact(v->p),0).second)points[cellOf(v->p)].push_back(v);}}
            bool changed=false;
            for(const auto&e:open){auto o=owner.find(e);if(o==owner.end())continue;const std::size_t ti=o->second.first;const int k=o->second.second;
                JoinedVertex*a=nativeTriangles[ti].v[k];JoinedVertex*b=nativeTriangles[ti].v[(k+1)%3];JoinedVertex*c=nativeTriangles[ti].v[(k+2)%3];
                if(!(edgeKeyExact(a->p,b->p)==e))continue; // this triangle was split earlier in the round
                const Vec d=b->p-a->p;const double dd=dot(d,d);if(dd<1e-20)continue;
                JoinedVertex*best=nullptr;double bestT=2;
                const double len=std::sqrt(dd);const int steps=int(len/(cell*.5))+1;Cell last{INT32_MIN,0,0};
                for(int s0=0;s0<=steps;++s0){const Cell cc=cellOf(a->p+d*(double(s0)/steps));if(cc==last)continue;last=cc;
                    for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy)for(int dz=-1;dz<=1;++dz){auto it=points.find(Cell{cc.x+dx,cc.y+dy,cc.z+dz});if(it==points.end())continue;
                        for(JoinedVertex*p:it->second){if(p==a||p==b||quantExact(p->p)==quantExact(a->p)||quantExact(p->p)==quantExact(b->p))continue;
                            const double t=dot(p->p-a->p,d)/dd;if(t<=1e-6||t>=1-1e-6)continue;
                            if(length(p->p-(a->p+d*t))>2e-7)continue;if(t<bestT){bestT=t;best=p;}}}}
                if(!best)continue;
                const double t=bestT;JoinedVertex nv=*a;
                nv.p=best->p;nv.normal=a->normal+(b->normal-a->normal)*t;nv.tangent=a->tangent+(b->tangent-a->tangent)*t;nv.binormal=a->binormal+(b->binormal-a->binormal)*t;
                for(int q=0;q<3;++q)nv.sourceBary[q]=a->sourceBary[q]+(b->sourceBary[q]-a->sourceBary[q])*t;for(int q=0;q<2;++q)nv.uv[q]=a->uv[q]+(b->uv[q]-a->uv[q])*t;
                for(std::uint32_t bone=0;bone<s.bones;++bone)nv.w[bone]=a->w[bone]+(b->w[bone]-a->w[bone])*t;
                indexed.extra.push_back(nv);JoinedVertex*V=&indexed.extra.back();
                if(!copiesBuilt){copiesBuilt=true;for(auto&tr:nativeTriangles)for(auto*v:tr.v){auto&list=copies[quantExact(v->p)];if(std::find(list.begin(),list.end(),v)==list.end())list.push_back(v);}}
                {auto it=copies.find(quantExact(best->p));if(it!=copies.end())for(auto*copy:it->second)copy->w=V->w;best->w=V->w;}
                IndexedTriangle second=nativeTriangles[ti];second.v[k]=V; // (V,b,c)
                nativeTriangles[ti].v[(k+1)%3]=V;                          // (a,V,c)
                nativeTriangles.push_back(second);
                --retainedCounts[e];++retainedCounts[edgeKeyExact(a->p,V->p)];++retainedCounts[edgeKeyExact(V->p,b->p)];retainedCounts[edgeKeyExact(V->p,c->p)]+=2;
                ++result.rimTJunctionsRepaired;changed=true;}
            if(!changed)break;}
        stages.mark("rim_tjunctions");}
    stages.mark("spurs");
    indexed.triangles=std::move(nativeTriangles);auto&finalTriangles=indexed.triangles;
    // Lookup-only hashed maps. The one order-sensitive walk (the gate below,
    // whose numerical-opening budget is consumed in key order) visits the
    // non-manifold/open edges sorted exactly as the former std::map did.
    // The triangles and their vertex store are final from here on, so skin can point into them.
    using EdgeCounts=EdgeCountMap;
    EdgeCounts edgeCounts=std::move(retainedCounts),original;original.reserve(s.f.size()*2);
    FlatMap<Quant,const std::array<double,64>*,QuantHash>skin;skin.reserve(finalTriangles.size());
    for(auto&f:s.f)for(int k=0;k<3;++k)++original[edgeKeyExact(s.v[f[k]].p,s.v[f[(k+1)%3]].p)];
    for(auto&t:finalTriangles){if(t.material)++result.cavityTriangles;for(int k=0;k<3;++k){auto&v=*t.v[k];auto[it,fresh]=skin.try_emplace(quantExact(v.p),&v.w);if(!fresh)for(std::uint32_t bone=0;bone<s.bones;++bone)result.maxJoinWeightConflict=std::max(result.maxJoinWeightConflict,std::abs((*it->second)[bone]-v.w[bone]));}}
    stages.mark("edge_maps");
    // The occupancy envelope closes the source's pre-existing eye/mouth/neck
    // openings with temporary triangles. Those triangles are not visible skin.
    // Cutting across an opening subdivides its old rim and makes a new cavity
    // edge on the temporary closure. Both remain part of the ORIGINAL opening,
    // not a crack in the joined wound. Check exact geometry, not a larger
    // distance tolerance or a blanket exemption for nearby edges.
    using FaceKey=std::array<Quant,3>;
    auto faceKey=[](const Source& src,const std::array<std::uint32_t,3>& f){FaceKey key{quantExact(src.v[f[0]].p),quantExact(src.v[f[1]].p),quantExact(src.v[f[2]].p)};std::sort(key.begin(),key.end());return key;};
    // Membership tests only: hashed faces and a per-face flag.
    std::unordered_map<FaceKey,char,QuantArrayHash<3>>sourceFaces;sourceFaces.reserve(s.f.size());for(auto&f:s.f)sourceFaces.try_emplace(faceKey(s,f),0);
    // Evaluated lazily for the few envelope faces an open edge actually queries.
    std::vector<signed char>closureState(body.source.f.size(),-1);
    auto closure=[&](std::uint32_t f){auto&state=closureState[f];if(state<0)state=sourceFaces.count(faceKey(body.source,body.source.f[f]))?0:1;return state!=0;};
    auto position=[](Quant q){return Vec{q[0]*1e-8,q[1]*1e-8,q[2]*1e-8};};
    std::vector<Edge>originalBoundary;for(const auto&entry:original)if(entry.second==1)originalBoundary.push_back(entry.first);
    // Any-match search over the original boundary segments. A segment can only
    // accept point a if a lies within weld of it, so a cell index of the
    // segments' weld-padded bounds is an exact candidate filter.
    constexpr double boundaryCell=.01;
    auto boundaryKey=[&](Vec p){return Cell{int(std::floor(p.x/boundaryCell)),int(std::floor(p.y/boundaryCell)),int(std::floor(p.z/boundaryCell))};};
    FlatMap<Cell,std::vector<std::uint32_t>,CellHash> boundaryCells;
    for(std::uint32_t i=0;i<originalBoundary.size();++i){Vec p=position(originalBoundary[i][0]),q=position(originalBoundary[i][1]);Bounds bb;bb.add(p);bb.add(q);
        const Cell lo=boundaryKey(bb.lo-Vec{2*weld,2*weld,2*weld}),hi=boundaryKey(bb.hi+Vec{2*weld,2*weld,2*weld});
        if(double(hi.x-lo.x+1)*double(hi.y-lo.y+1)*double(hi.z-lo.z+1)>4096){boundaryCells[Cell{INT32_MIN,0,0}].push_back(i);continue;} // huge: always a candidate
        for(int x=lo.x;x<=hi.x;++x)for(int y=lo.y;y<=hi.y;++y)for(int z=lo.z;z<=hi.z;++z)boundaryCells[Cell{x,y,z}].push_back(i);}
    auto onOriginalOpening=[&](const Edge&edge){
        Vec a=position(edge[0]),b=position(edge[1]);
        auto tryEntry=[&](const Edge&entry){Vec p=position(entry[0]),d=position(entry[1])-p;double dd=dot(d,d);if(dd<1e-20)return false;
            auto onSegment=[&](Vec v){double t=dot(v-p,d)/dd;return t>=-1e-8&&t<=1+1e-8&&length(v-(p+d*std::clamp(t,0.,1.)))<=weld;};
            return onSegment(a)&&onSegment(b);};
        {auto it=boundaryCells.find(boundaryKey(a));if(it!=boundaryCells.end())for(auto i:it->second)if(tryEntry(originalBoundary[i]))return true;
         auto huge=boundaryCells.find(Cell{INT32_MIN,0,0});if(huge!=boundaryCells.end())for(auto i:huge->second)if(tryEntry(originalBoundary[i]))return true;}
        Bounds bounds;bounds.add(a);bounds.add(b);bounds.lo=bounds.lo-Vec{weld,weld,weld};bounds.hi=bounds.hi+Vec{weld,weld,weld};
        std::vector<std::uint32_t>nearby;body.query(0,bounds,nearby);
        std::vector<std::pair<double,double>>spans;
        for(auto f:nearby)if(closure(f)){auto&ids=body.source.f[f];
            auto onFace=[&](Vec p){return length(p-closestTriangle(p,body.source.v[ids[0]].p,body.source.v[ids[1]].p,body.source.v[ids[2]].p))<=weld;};
            if(onFace(a)&&onFace(b))return true;
            std::pair<double,double>span;
            if(segmentTriangleInterval(a,b,body.source.v[ids[0]].p,body.source.v[ids[1]].p,body.source.v[ids[2]].p,weld,span))spans.push_back(span);
        }
        return coversSegment(std::move(spans));
    };
    // Authored clothing sheets can already share a nonmanifold edge. A cut
    // may subdivide it; preserve only its exact original face incidence. New
    // cavity faces and repeated faces never qualify for this exception.
    std::map<Edge,std::set<std::uint32_t>> originalIncidence;
    for(std::uint32_t f=0;f<s.f.size();++f)for(int k=0;k<3;++k){auto edge=edgeKeyExact(s.v[s.f[f][k]].p,s.v[s.f[f][(k+1)%3]].p);if(original[edge]>2)originalIncidence[edge].insert(f);}
    std::unordered_map<Edge,std::vector<const JoinedTriangle*>,QuantArrayHash<2>> crowded;std::deque<JoinedTriangle> crowdedCopies;
    // Only edges used more than twice, in the former map's key order.
    std::vector<const EdgeCounts::value_type*> irregular;
    for(const auto&item:edgeCounts)if(item.second!=2)irregular.push_back(&item);
    std::sort(irregular.begin(),irregular.end(),[](auto a,auto b){return a->first<b->first;});
    bool anyCrowded=false;for(auto e:irregular)anyCrowded=anyCrowded||e->second>2;
    if(anyCrowded){EdgeCountMap crowdedEdges;for(auto e:irregular)if(e->second>2)crowdedEdges.try_emplace(e->first,e->second);
        // The gate helpers read whole triangles: copy only those at crowded edges.
        for(const auto&t:finalTriangles){const JoinedTriangle*copy=nullptr;
            for(int k=0;k<3;++k){auto edge=edgeKeyExact(t.v[k]->p,t.v[(k+1)%3]->p);if(!crowdedEdges.count(edge))continue;
                if(!copy){crowdedCopies.push_back({t.material,{*t.v[0],*t.v[1],*t.v[2]}});copy=&crowdedCopies.back();}crowded[edge].push_back(copy);}}}
    // Built on first use only (a pure lookup structure; same answers).
    std::shared_ptr<const ClosureIndex> closureIndex;
    auto inheritedClosureSheet=[&](const Edge&edge){
        const JoinedTriangle*sheet=nullptr;for(auto*t:crowded.at(edge))if(!t->material){if(sheet)return false;sheet=t;}
        if(!sheet||sheet->v[0].sourceFace>=s.f.size())return false;
        if(!closureIndex)closureIndex=envelopeClosureIndex(s,body,weld);
        return closureSheetMember(*closureIndex,sourceFaces,surfaceFaceKey(s,s.f[sheet->v[0].sourceFace]))&&onOriginalOpening(edge);
    };
    auto inheritedJunction=[&](const Edge&edge){
        std::set<std::uint32_t> faces;
        for(const auto*t:crowded.at(edge)){
            if(t->material || t->v[0].sourceFace!=t->v[1].sourceFace || t->v[0].sourceFace!=t->v[2].sourceFace || !faces.insert(t->v[0].sourceFace).second)return false;
        }
        for(const auto&candidate:originalIncidence){
            if(!std::includes(candidate.second.begin(),candidate.second.end(),faces.begin(),faces.end()))continue;
            Vec p=position(candidate.first[0]),d=position(candidate.first[1])-p;double dd=dot(d,d);if(dd<1e-20)continue;
            bool on=true;for(auto q:edge){Vec v=position(q);double t=dot(v-p,d)/dd;on=on&&t>=-1e-8&&t<=1+1e-8&&length(v-(p+d*std::clamp(t,0.,1.)))<=weld;}
            if(on)return true;
        }return false;
    };
    // Render vertices are float32. A generated edge shorter than one micron
    // can survive round-off beside a collapsed triangle. Do not move source
    // vertices or reject the entire shot for that render-only residual. Keep
    // a strict length/count budget and verify identical endpoint skin so it
    // cannot expand into a visible crack when bent. This is NOT a declaration
    // of watertight topology: the residual is separately counted and measured.
    stages.mark("gate_setup");
    for(auto entry:irregular){
        // Tile-local: past 64 rejected edges the candidate is rejected anyway (a
        // gross failure, see LiveDamageWorker); stop the costly opening searches.
        if(extras&&result.newOpen+result.nonmanifold>64)break;
        const auto&item=*entry;bool rejected=false;if(item.second>2){if(!inheritedJunction(item.first)){
        if(body.preserveCrossings&&inheritedInteriorSheet(item.first,crowded.at(item.first),body,g.pitch,inheritedClosureSheet(item.first)))++result.interiorSheetJunctions;
        else{++result.nonmanifold;rejected=true;
#ifdef GFY_COLLIDER_DIAGNOSTICS
            {const auto&e=item.first;std::cerr<<"nonmanifold_edge "<<double(e[0][0])*1e-8<<','<<double(e[0][1])*1e-8<<','<<double(e[0][2])*1e-8<<" - "<<double(e[1][0])*1e-8<<','<<double(e[1][1])*1e-8<<','<<double(e[1][2])*1e-8<<" tris="<<crowded.at(item.first).size()<<std::endl;}
#endif
            // Diagnostic class: a crowded edge carried by two or more cavity
            // triangles (overlapping cavity polygons, independent of the skin).
            {std::size_t cav=0;for(const auto*t:crowded.at(item.first))cav+=t->material!=0;if(cav>=2)++result.nonmanifoldCavityDoubled;}}}}else if(item.second==1){
        // A seam with a statically emitted lattice piece (same canonical vertices):
        // a cheap exact position test, checked first (tile-local regions only).
        if(extras&&extras->extraOpening&&extras->extraOpening(position(item.first[0]),position(item.first[1])))++result.originalOpen;
        else if(onOriginalOpening(item.first))++result.originalOpen;
        else if(body.preserveCrossings&&nativeEdgeCovered(item.first,edgeCounts,skin,1e-6))++result.numericalSeams;
        else if(body.preserveCrossings&&recordNumericalOpening(item.first,skin,s.bones,result)){}
        else{++result.newOpen;rejected=true;}}
#ifdef GFY_CUT_DIAGNOSTICS
        if(rejected){std::cout<<"rejected_edge "<<item.second;for(auto q:item.first)for(auto value:q)std::cout<<' '<<std::setprecision(14)<<value*1e-8;std::cout<<'\n';
         if(rejected&&item.second==1){for(const auto&t:finalTriangles)for(int k=0;k<3;++k)if(edgeKeyExact(t.v[k]->p,t.v[(k+1)%3]->p)==item.first)std::cout<<"rejected_open_incident material="<<t.material<<" a_ext="<<canonical.count(quantExact(t.v[k]->p))<<" b_ext="<<canonical.count(quantExact(t.v[(k+1)%3]->p))<<std::endl;}
            if(item.second>2)for(const auto*t:crowded.at(item.first)){
                std::cout<<"rejected_incident "<<t->material;
                for(const auto&v:t->v)std::cout<<' '<<v.sourceFace;
                for(const auto&v:t->v)std::cout<<' '<<std::setprecision(14)<<v.p.x<<' '<<v.p.y<<' '<<v.p.z;
                std::cout<<'\n';}}
#endif
    }
    stages.mark("gate_validate");
    // Perimeter comes from the intentional pre-join cut boundary only.
    EdgeCounts outer;outer.reserve(exteriorMesh.polygons.size()*8);std::unordered_map<Quant,Vec,QuantHash>coords;
    for(auto&f:exteriorMesh.polygons)for(std::size_t i=0;i<f.polygon.size();++i){Vec a=f.polygon[i].p,b=f.polygon[(i+1)%f.polygon.size()].p;++outer[edgeKeyExact(a,b)];coords[quantExact(a)]=a;coords[quantExact(b)]=b;}
    // Sum in the former std::map key order: identical floating result.
    std::vector<const Edge*>single;for(auto&item:outer)if(item.second==1)single.push_back(&item.first);
    std::sort(single.begin(),single.end(),[](auto a,auto b){return *a<*b;});
    for(auto e:single){auto found=original.find(*e);if(!(found!=original.end()&&found->second==1))result.cutPerimeter+=length(coords[(*e)[1]]-coords[(*e)[0]]);}
    stages.mark("perimeter");
    return indexed;
}
void writeJoined(const std::string&path,const JoinResult&mesh,std::uint32_t bones){std::ofstream out(path,std::ios::binary);out.write("VXJOIN01",8);std::uint32_t count=std::uint32_t(mesh.triangles.size());out.write(reinterpret_cast<const char*>(&count),4);out.write(reinterpret_cast<const char*>(&bones),4);for(auto&t:mesh.triangles){std::uint32_t material=t.material;out.write(reinterpret_cast<const char*>(&material),4);for(auto&v:t.v){float pos[3]={float(v.p.x),float(v.p.y),float(v.p.z)},uv[2]={float(v.uv[0]),float(v.uv[1])};out.write(reinterpret_cast<const char*>(pos),12);out.write(reinterpret_cast<const char*>(uv),8);for(std::uint32_t b=0;b<bones;++b){float w=float(v.w[b]);out.write(reinterpret_cast<const char*>(&w),4);}}}}
