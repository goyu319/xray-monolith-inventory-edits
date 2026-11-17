#pragma once
#include "BloodCellLayer.h"
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <unordered_set>

namespace BloodBakeAsync {
using Clock=std::chrono::steady_clock;
struct Limits {
    std::size_t maxJobs=1024, maxQueuedBytes=64u*1024u*1024u;
    // All bodies' tables together (the CPU image of each body's texture), and deltas waiting for the owner.
    std::size_t maxLayerBytes=192u*1024u*1024u, maxCompletedBytes=64u*1024u*1024u;
    // One body lineage's table. At the cap the body's own oldest stains make room for the new one.
    std::size_t maxBodyLayerBytes=BloodCellLayer::Layer::DefaultMaxBytes;
    std::size_t maxSplatCells=1048576;
};
enum class SubmitResult { Accepted, Backpressure, Stopped, Canceled, Invalid };
enum class Failure : unsigned { None, LayerPressure, CompletedPressure, Allocation, Prepare, Bake, Finalize, RegistryPressure };
struct Accounting {
    std::atomic<std::size_t> jobs{0}, queuedBytes{0}, layerBytes{0}, completedBytes{0};
    std::atomic<std::uint64_t> submitted{0}, completed{0}, canceled{0};
    std::atomic<std::uint64_t> baked{0}, published{0}, dropped{0}, truncated{0}, evictedOthers{0}, evictedSelf{0}, waits{0};
    // cells of a body's own oldest stains dropped at its cap; cells outside the 2 x 4 x 4 m window; whole-image publications
    std::atomic<std::uint64_t> cellsEvicted{0}, cellsOutside{0}, fullPublications{0};
    std::atomic<std::size_t> peakLayerBytes{0}, peakCompletedBytes{0};
    std::atomic<std::uint64_t> ownerFrames{0}; // owner heartbeat: GPU-quota evictions wait for the owner to take the last one
    std::atomic<std::size_t> owed{0}; // bodies with baked stains not yet in a delta + cleared notices not yet published
};
inline void notePeak(std::atomic<std::size_t>& peak,std::size_t value)noexcept {
    auto old=peak.load(std::memory_order_relaxed);
    while(value>old&&!peak.compare_exchange_weak(old,value,std::memory_order_relaxed)){}
}
inline bool reserve(std::atomic<std::size_t>& counter,std::size_t n,std::size_t limit)noexcept {
    auto old=counter.load(std::memory_order_relaxed);
    for(;;){if(n>limit||old>limit-n)return false;if(counter.compare_exchange_weak(old,old+n,std::memory_order_acq_rel,std::memory_order_relaxed))return true;}
}
// Bounded MPSC pointer ring. Every producer makes at most eight CAS attempts;
// contention returns backpressure with ownership retained by the caller.
template<class T,std::size_t N> class Ring {
    struct Slot {std::atomic<std::size_t> sequence;T* value=nullptr;};
    std::array<Slot,N> slots_;
    alignas(64) std::atomic<std::size_t> enqueue_{0};
    alignas(64) std::size_t dequeue_=0; // one consumer only
public:
    Ring(){static_assert((N&(N-1))==0,"power_of_two");for(std::size_t i=0;i<N;++i)slots_[i].sequence.store(i,std::memory_order_relaxed);}
    bool push(T* value)noexcept {
        auto p=enqueue_.load(std::memory_order_relaxed);
        for(unsigned tries=0;tries<8;++tries){auto& s=slots_[p&(N-1)];const auto q=s.sequence.load(std::memory_order_acquire);
            const auto d=static_cast<std::intptr_t>(q)-static_cast<std::intptr_t>(p);
            if(d==0){if(enqueue_.compare_exchange_weak(p,p+1,std::memory_order_relaxed)){s.value=value;s.sequence.store(p+1,std::memory_order_release);return true;}}
            else if(d<0)return false;else p=enqueue_.load(std::memory_order_relaxed);
        }return false;
    }
    T* pop()noexcept {
        auto& s=slots_[dequeue_&(N-1)];if(s.sequence.load(std::memory_order_acquire)!=dequeue_+1)return nullptr;
        T* value=s.value;s.sequence.store(dequeue_+N,std::memory_order_release);++dequeue_;return value;
    }
};
struct Job {
    std::vector<std::array<BloodCellLayer::Vertex,3>> faces;
    BloodCellLayer::Stamp stamp;
    // Explicit own-cut identity only. All-zero ordinary marks keep append
    // semantics. The namespace distinguishes object damage sessions sharing
    // one body/piece lineage; revision is monotonic within a cut.
    std::uint64_t markNamespace=0,markKey=0,markRevision=0;
    bool markPreserveOldCoverage=false; // explicit grow-only own-cut route
    std::function<BloodCellLayer::TextureSample(float,float)> sample;
    // Worker-only hook for captured projection and immutable texture decode.
    // It must not access CKinematics, immediate D3D context, or live containers.
    std::function<void(Job&)> prepare;
    std::size_t capturedBytes=0; // caller accounts immutable captured input
    std::size_t bytes()const noexcept {return sizeof(Job)+faces.capacity()*sizeof(faces[0])+capturedBytes;}
};
struct Delta {
    std::uint64_t sequence=0, changed=0, bakeMicroseconds=0;
    // The table's height in rows (BloodCellLayer::Width texels each). fullSnapshot: `rows` is the whole image and becomes a
    // new texture; else `rows` holds the rows named by dirtyRows (ascending), copied into the live texture in place.
    std::uint32_t height=0;
    bool fullSnapshot=false;
    // bloodfix: the body's layer was dropped under pressure. The owner releases the body's GPU banks and draws it without
    // blood; a later delta starts the body's blood again from an empty layer. Always published before any later delta.
    bool cleared=false;
    std::vector<std::uint32_t> dirtyRows;
    std::vector<BloodCellLayer::U4> rows;
    // Optional worker-prepared GPU resources; portable scheduler never uses it.
    std::shared_ptr<void> prepared;
private:
    std::shared_ptr<Accounting> accounting_;std::size_t credit_=0;
    friend class Service;
public:
    ~Delta(){if(accounting_)accounting_->completedBytes.fetch_sub(credit_,std::memory_order_relaxed);}
};
class State {
    BloodCellLayer::Layer layer_;
    Ring<Delta,2> completed_;
    std::atomic<unsigned> completionCount_{0};
    std::atomic<bool> canceled_{false}, snapshot_{false};
    std::atomic<std::uint64_t> completedSequence_{0}, failureCount_{0};
    std::atomic<std::uint64_t> retryGeneration_{0};
    std::atomic<Failure> failure_{Failure::None};
    std::atomic<std::uint64_t> evictions_{0}, dropped_{0};
    std::shared_ptr<Accounting> accounting_;
    std::size_t layerCredit_=0;
    std::uint64_t sequence_=0;
    std::function<void(Delta&)> finalize_;
    bool registered_=false; // worker only
    bool snapshotFailed_=false;
    std::uint64_t snapshotFailedGeneration_=0;
    // Worker only (bloodfix): stains baked but not yet in a delta; the cleared notice owed to the owner; the order of the
    // last bake among all bodies (the eviction order); the time before which a refused publication is not repeated.
    bool unpublished_=false, clearOwed_=false;
    std::uint64_t pendingChanged_=0, pendingMicros_=0, lastBaked_=0;
    unsigned publishRefusals_=0;
    Clock::time_point retryAt_{};
    friend class Service;
    void fail(Failure why)noexcept {if(failure_.exchange(why,std::memory_order_relaxed)!=why)failureCount_.fetch_add(1,std::memory_order_relaxed);}
public:
    explicit State(BloodCellLayer::Frame f,std::shared_ptr<Accounting> a,std::function<void(Delta&)> finalize={}):layer_(f),accounting_(std::move(a)),finalize_(std::move(finalize)){}
    ~State(){while(auto* p=completed_.pop())delete p;accounting_->layerBytes.fetch_sub(layerCredit_,std::memory_order_relaxed);}
    State(const State&)=delete;State& operator=(const State&)=delete;
    // Single render-thread consumer. Packet memory is never mutated afterward.
    std::unique_ptr<Delta> tryTakeCompleted()noexcept {
        auto* p=completed_.pop();if(p)completionCount_.fetch_sub(1,std::memory_order_release);return std::unique_ptr<Delta>(p);
    }
    void cancel()noexcept {canceled_.store(true,std::memory_order_release);}
    bool canceled()const noexcept{return canceled_.load(std::memory_order_acquire);}
    void requestSnapshot()noexcept {snapshot_.store(true,std::memory_order_release);retryGeneration_.fetch_add(1,std::memory_order_release);}
    std::uint64_t completedSequence()const noexcept{return completedSequence_.load(std::memory_order_acquire);}
    std::uint64_t failureCount()const noexcept{return failureCount_.load(std::memory_order_relaxed);}
    Failure failure()const noexcept{return failure_.load(std::memory_order_relaxed);}
    // How often this body's blood was dropped under pressure, and how many of its stains could not be baked at all.
    std::uint64_t evictionCount()const noexcept{return evictions_.load(std::memory_order_relaxed);}
    std::uint64_t droppedCount()const noexcept{return dropped_.load(std::memory_order_relaxed);}
    // Kept for callers of the earlier revision: a refused snapshot is repeated after this. Jobs are no longer held.
    void retryFailed()noexcept {retryGeneration_.fetch_add(1,std::memory_order_release);}
};

class Service {
    struct Work {
        std::weak_ptr<State> state;
        std::unique_ptr<Job> job;
        std::size_t credit=0;
        bool prepared=false;
        unsigned attempts=0;
        Clock::time_point retryAt{};
    };
    struct Control {
        Limits limits;std::shared_ptr<Accounting> accounting=std::make_shared<Accounting>();
        Ring<Work,1024> input;Ring<Delta,1024> retired;
        std::atomic<bool> stop{false};
        std::atomic<unsigned> producers{0};
        std::function<void()> workerInit;
        std::function<void(Delta&)> finalize;
        // worker only
        std::uint64_t bakeClock=0, quotaEvictionFrame=0;
        Clock::time_point quotaEvictionAt{};
        bool quotaEvicted=false;
    };
    using Registry=std::array<std::shared_ptr<State>,1024>;
    enum class Step { Baked, Wait, Dropped };
    std::shared_ptr<Control> control_;
    std::thread worker_;
    static void release(Control& c,Work* w)noexcept {
        c.accounting->queuedBytes.fetch_sub(w->credit,std::memory_order_relaxed);
        c.accounting->jobs.fetch_sub(1,std::memory_order_relaxed);delete w;
    }
    // The two worker-only flags of a body are counted in Accounting::owed (0 = every baked stain is in a published delta).
    static void owe(Control& c,bool& flag,bool value)noexcept {
        if(flag==value)return;flag=value;
        if(value)c.accounting->owed.fetch_add(1,std::memory_order_relaxed);else c.accounting->owed.fetch_sub(1,std::memory_order_relaxed);
    }
    static void wait(State& s,unsigned& refusals)noexcept {
        // 5, 10, 20 ... 200 ms: a refusal is never repeated every pass
        const unsigned shift=(std::min)(refusals,6u);if(refusals<1000u)++refusals;
        s.retryAt_=Clock::now()+std::chrono::milliseconds((std::min)(200u,5u<<shift));
    }
    // The body's layer goes; its credit returns at once. The owner is told with a `cleared` delta (publishCleared).
    static bool clearLayer(Control& c,State& s,bool self)noexcept {
        try{s.layer_=BloodCellLayer::Layer(s.layer_.frame());}catch(...){return false;}
        c.accounting->layerBytes.fetch_sub(s.layerCredit_,std::memory_order_relaxed);s.layerCredit_=0;
        owe(c,s.unpublished_,false);owe(c,s.clearOwed_,true);s.pendingChanged_=s.pendingMicros_=0;s.publishRefusals_=0;s.retryAt_=Clock::time_point{};
        s.evictions_.fetch_add(1,std::memory_order_relaxed);
        (self?c.accounting->evictedSelf:c.accounting->evictedOthers).fetch_add(1,std::memory_order_relaxed);
        return true;
    }
    // Drop oldest: the body whose last stain is the oldest gives up its blood for the new stain. Never the asking body.
    static bool evictOldest(Control& c,State& asking,Registry& states)noexcept {
        State* victim=nullptr;
        for(auto& s:states)if(s&&s.get()!=&asking&&!s->canceled()&&s->layerCredit_&&(!victim||s->lastBaked_<victim->lastBaked_))victim=s.get();
        return victim&&clearLayer(c,*victim,false);
    }
    static bool publishCleared(Control& c,State& s) {
        if(s.completionCount_.load(std::memory_order_acquire)>=2)return false;
        const auto credit=sizeof(Delta);
        if(!reserve(c.accounting->completedBytes,credit,c.limits.maxCompletedBytes)){s.fail(Failure::CompletedPressure);return false;}
        std::unique_ptr<Delta> d;
        try{d=std::make_unique<Delta>();}catch(...){c.accounting->completedBytes.fetch_sub(credit,std::memory_order_relaxed);throw;}
        d->accounting_=c.accounting;d->credit_=credit;d->sequence=s.sequence_;d->fullSnapshot=true;d->cleared=true;
        try{if(s.finalize_)s.finalize_(*d);else if(c.finalize)c.finalize(*d);}
        catch(const std::bad_alloc&){throw;}catch(...){s.fail(Failure::Finalize);return false;}
        s.completionCount_.fetch_add(1,std::memory_order_release);
        if(!s.completed_.push(d.get())){s.completionCount_.fetch_sub(1,std::memory_order_release);return false;}
        d.release();owe(c,s.clearOwed_,false);return true;
    }
    static bool publish(Control& c,State& s,bool full,std::uint64_t changed,std::uint64_t micros) {
        if(s.completionCount_.load(std::memory_order_acquire)>=2)return false;
        if(s.clearOwed_&&!publishCleared(c,s))return false; // the owner drops the old banks before any later content
        if(s.completionCount_.load(std::memory_order_acquire)>=2)return false;
        // A resized or rebuilt table goes out whole (the preparer makes a new texture of it); else only its changed rows.
        full=full||s.layer_.rebuilt();
        std::vector<std::uint32_t> dirty;if(!full)dirty=s.layer_.dirtyRows();
        const std::size_t rowTexels=BloodCellLayer::Width;
        const std::size_t texels=full?s.layer_.image().size():dirty.size()*rowTexels;
        const auto credit=sizeof(Delta)+texels*sizeof(BloodCellLayer::U4)+dirty.capacity()*sizeof(std::uint32_t);
        if(!reserve(c.accounting->completedBytes,credit,c.limits.maxCompletedBytes)){s.fail(Failure::CompletedPressure);return false;}
        notePeak(c.accounting->peakCompletedBytes,c.accounting->completedBytes.load(std::memory_order_relaxed));
        std::unique_ptr<Delta> d;
        try{d=std::make_unique<Delta>();}catch(...){c.accounting->completedBytes.fetch_sub(credit,std::memory_order_relaxed);throw;}
        d->accounting_=c.accounting;d->credit_=credit;d->sequence=s.sequence_;d->changed=changed;d->bakeMicroseconds=micros;
        d->height=s.layer_.rows();d->fullSnapshot=full;
        if(full)d->rows=s.layer_.image();
        else{d->rows.resize(texels);const auto& image=s.layer_.image();
            for(std::size_t n=0;n<dirty.size();++n)std::copy(image.begin()+std::size_t(dirty[n])*rowTexels,image.begin()+(std::size_t(dirty[n])+1)*rowTexels,d->rows.begin()+n*rowTexels);
            d->dirtyRows=std::move(dirty);}
        try{if(s.finalize_)s.finalize_(*d);else if(c.finalize)c.finalize(*d);}
        catch(const std::bad_alloc&){throw;}catch(...){s.fail(Failure::Finalize);return false;}
        // One producer, one consumer, with count reserved before publication.
        s.completionCount_.fetch_add(1,std::memory_order_release);
        if(!s.completed_.push(d.get())){s.completionCount_.fetch_sub(1,std::memory_order_release);return false;}
        d.release();s.layer_.acknowledge();s.completedSequence_.store(s.sequence_,std::memory_order_release);s.failure_.store(Failure::None,std::memory_order_relaxed);
        if(full)c.accounting->fullPublications.fetch_add(1,std::memory_order_relaxed);
        owe(c,s.unpublished_,false);s.pendingChanged_=s.pendingMicros_=0;s.publishRefusals_=0;s.retryAt_=Clock::time_point{};
        c.accounting->published.fetch_add(1,std::memory_order_relaxed);return true;
    }
    // The GPU quota (the preparer's bad_alloc) frees only when the owner has taken a `cleared` delta. One body gives up its
    // blood, then the owner gets 30 of its frames and a quarter second before another one does.
    static void quotaPressure(Control& c,State& s,Registry& states)noexcept {
        s.fail(Failure::Allocation);c.accounting->waits.fetch_add(1,std::memory_order_relaxed);
        const auto frames=c.accounting->ownerFrames.load(std::memory_order_relaxed);const auto now=Clock::now();
        const bool settled=!c.quotaEvicted||(frames-c.quotaEvictionFrame>=30&&now-c.quotaEvictionAt>=std::chrono::milliseconds(250));
        if(settled){
            bool freed=evictOldest(c,s,states);
            // No other body holds blood and this one still does not fit after a long wait: its own older blood goes.
            if(!freed&&s.publishRefusals_>=40&&s.layerCredit_)freed=clearLayer(c,s,true);
            if(freed){c.quotaEvicted=true;c.quotaEvictionFrame=frames;c.quotaEvictionAt=now;}
        }
    }
    // Everything baked on this body since its last publication goes out in one delta.
    static bool publishPending(Control& c,State& s,Registry& states)noexcept {
        // A snapshot request (device recovery) is answered first: the new preparer has no table for changed rows yet, and
        // the whole image carries everything baked.
        if(s.snapshot_.load(std::memory_order_acquire))return snapshot(c,s);
        if(!s.unpublished_&&!s.clearOwed_)return false;
        if(s.completionCount_.load(std::memory_order_acquire)>=2||Clock::now()<s.retryAt_)return false;
        try{
            if(!s.unpublished_){if(publishCleared(c,s))return true;}
            else if(publish(c,s,false,s.pendingChanged_,s.pendingMicros_))return true;
            if(s.completionCount_.load(std::memory_order_acquire)>=2)return false; // the owner has not taken the last two yet
            if(s.failure()==Failure::Finalize&&s.publishRefusals_>=40&&s.layerCredit_){
                // the preparer refuses this body's content for good (not memory): its blood is dropped rather than repeated forever
                s.dropped_.fetch_add(1,std::memory_order_relaxed);c.accounting->dropped.fetch_add(1,std::memory_order_relaxed);clearLayer(c,s,true);return true;
            }
            c.accounting->waits.fetch_add(1,std::memory_order_relaxed);
        }
        catch(const std::bad_alloc&){quotaPressure(c,s,states);}
        catch(...){s.fail(Failure::Bake);}
        wait(s,s.publishRefusals_);return false;
    }
    static Step bake(Control& c,State& s,Work& w,Registry& states)noexcept {
        auto refuse=[&](Failure why,unsigned tries){
            s.fail(why);
            if(++w.attempts>=tries)return Step::Dropped;
            w.retryAt=Clock::now()+std::chrono::milliseconds(50);return Step::Wait;
        };
        if(!w.prepared){
            try{if(w.job->prepare)w.job->prepare(*w.job);w.job->prepare={};w.prepared=true;w.attempts=0;}
            catch(const std::bad_alloc&){return refuse(Failure::Allocation,40);}
            catch(...){return refuse(Failure::Prepare,3);} // e.g. a texture that cannot be decoded: dropped, the body's later stains go on
        }
        try{
            if(!BloodCellLayer::valid(w.job->stamp)||!w.job->sample)throw std::invalid_argument("blood_async_job");
            const bool keyed=w.job->markNamespace||w.job->markKey||w.job->markRevision;
            if(keyed&&(!w.job->markNamespace||!w.job->markKey||!w.job->markRevision))throw std::invalid_argument("blood_async_partial_key");
            if(w.job->markPreserveOldCoverage&&!keyed)throw std::invalid_argument("blood_async_unkeyed_growth");
            const auto begin=Clock::now();BloodCellLayer::Splat splat(w.job->stamp);bool cut=false;
            auto admit=[&](BloodCellLayer::Coord cell,BloodCellLayer::Vec3){
                if(c.stop.load(std::memory_order_relaxed)||s.canceled())throw std::runtime_error("blood_async_canceled");
                // A stain over the cell limit is cut at the limit (raster order); it was refused whole before.
                if(splat.size()>=c.limits.maxSplatCells&&splat.samples().find(cell)==splat.samples().end()){cut=true;return false;}
                return true;};
            const bool stale=keyed&&s.layer_.keyedStale(w.job->markNamespace,w.job->markKey,w.job->markRevision);
            if(!stale)for(const auto& tri:w.job->faces)BloodCellLayer::rasterizeTriangle(splat,s.layer_.frame(),tri,w.job->sample,admit);
            if(cut)c.accounting->truncated.fetch_add(1,std::memory_order_relaxed);
            // Memory. The table grows inside the body's cap; the growth is reserved against the global limit first, and
            // when that is short the least recently stained OTHER body's blood goes. With nothing left to free the table
            // keeps its size and drops its own oldest stains for the new one (BloodCellLayer::Layer does that at any cap).
            std::size_t growth=stale?0:s.layer_.growthFor(s.layer_.newCells(splat),c.limits.maxBodyLayerBytes);
            while(growth&&!reserve(c.accounting->layerBytes,growth,c.limits.maxLayerBytes)){
                if(evictOldest(c,s,states))continue;
                growth=0;s.fail(Failure::LayerPressure);c.accounting->waits.fetch_add(1,std::memory_order_relaxed);
            }
            const std::size_t allowed=(std::min)(c.limits.maxBodyLayerBytes,(std::max)(s.layer_.bytes()+growth,BloodCellLayer::Layer::bytesFor(1)));
            auto settle=[&]{ // the credit follows the table's real size
                const std::size_t held=s.layerCredit_+growth,now=s.layer_.bytes();
                if(now<held)c.accounting->layerBytes.fetch_sub(held-now,std::memory_order_relaxed);
                else if(now>held)c.accounting->layerBytes.fetch_add(now-held,std::memory_order_relaxed);
                s.layerCredit_=now;notePeak(c.accounting->peakLayerBytes,c.accounting->layerBytes.load(std::memory_order_relaxed));};
            BloodCellLayer::Layer::Applied applied;
            try{applied=keyed?s.layer_.applyKeyed(splat,w.job->markNamespace,w.job->markKey,w.job->markRevision,w.job->markPreserveOldCoverage,allowed):s.layer_.apply(splat,allowed);}
            catch(...){settle();throw;}
            settle();
            const std::uint64_t changed=applied.changed+applied.evicted;
            c.accounting->cellsEvicted.fetch_add(applied.evicted,std::memory_order_relaxed);c.accounting->cellsOutside.fetch_add(applied.outside,std::memory_order_relaxed);
            // A stain that changed nothing in the table (off the receiver, a stale cut revision) owes no delta.
            ++s.sequence_;if(s.layer_.dirty())owe(c,s.unpublished_,true);s.pendingChanged_+=changed;s.lastBaked_=++c.bakeClock;
            s.pendingMicros_+=static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(Clock::now()-begin).count());
            c.accounting->baked.fetch_add(1,std::memory_order_relaxed);
            return Step::Baked;
        }
        catch(const std::bad_alloc&){return refuse(Failure::Allocation,40);}
        catch(...){s.fail(Failure::Bake);return Step::Dropped;}
    }
    static bool snapshot(Control& c,State& s)noexcept {
        const auto generation=s.retryGeneration_.load(std::memory_order_acquire);
        if(s.canceled()||!s.snapshot_.load(std::memory_order_acquire)||(s.snapshotFailed_&&s.snapshotFailedGeneration_==generation))return false;
        if(Clock::now()<s.retryAt_)return false;
        s.snapshotFailed_=false;
        // Clear before construction; a new device-generation request made
        // during finalization remains set and cannot be accidentally erased.
        s.snapshot_.store(false,std::memory_order_release);
        try{if(publish(c,s,true,0,0))return true;
            if(s.failure()==Failure::Finalize){s.snapshotFailed_=true;s.snapshotFailedGeneration_=generation;}}
        catch(const std::bad_alloc&){s.fail(Failure::Allocation);}catch(...){s.fail(Failure::Bake);s.snapshotFailed_=true;s.snapshotFailedGeneration_=generation;}
        s.snapshot_.store(true,std::memory_order_release);wait(s,s.publishRefusals_);return false;
    }
    static void run(std::shared_ptr<Control> c)noexcept {
        try{if(c->workerInit)c->workerInit();}catch(...){}
        // Fixed scheduler storage cannot fail after popping an accepted job.
        // Strong registry ownership also moves ordinary Layer destruction off
        // render: collection happens once the registry is the sole owner.
        std::array<Work*,1024> backlog{};std::size_t backlogCount=0;
        Registry states;
        while(!c->stop.load(std::memory_order_acquire)){
            bool progress=false;
            while(auto* p=c->retired.pop())delete p;
            for(auto& s:states)if(s&&s.use_count()==1){owe(*c,s->unpublished_,false);owe(*c,s->clearOwed_,false);s.reset();}
            while(auto* w=c->input.pop()){
                auto s=w->state.lock();if(!s||s->canceled()){c->accounting->canceled.fetch_add(1,std::memory_order_relaxed);release(*c,w);continue;}
                backlog[backlogCount++]=w; // maxJobs credit proves capacity
            }
            // Device recovery seeds the replacement preparer's page cache
            // before any later ordinary delta; it carries every baked stain.
            for(auto& s:states)if(s&&snapshot(*c,*s))progress=true;
            // First in, first out per body: a body whose head job waits holds back only its own later jobs.
            std::array<State*,1024> held{};std::size_t heldCount=0;
            auto isHeld=[&](State* s){return std::find(held.begin(),held.begin()+heldCount,s)!=held.begin()+heldCount;};
            auto erase=[&](std::size_t i){for(std::size_t n=i+1;n<backlogCount;++n)backlog[n-1]=backlog[n];--backlogCount;};
            for(std::size_t i=0;i<backlogCount;){auto* w=backlog[i];auto s=w->state.lock();
                if(!s||s->canceled()){c->accounting->canceled.fetch_add(1,std::memory_order_relaxed);release(*c,w);erase(i);progress=true;continue;}
                if(!s->registered_){for(auto& slot:states)if(!slot){slot=s;s->registered_=true;break;}}
                if(!s->registered_){s->fail(Failure::RegistryPressure);++i;continue;}
                if(isHeld(s.get())){++i;continue;}
                if(s->snapshot_.load(std::memory_order_acquire)&&!snapshot(*c,*s)){held[heldCount++]=s.get();++i;continue;}
                if(Clock::now()<w->retryAt){held[heldCount++]=s.get();++i;continue;}
                const auto step=bake(*c,*s,*w,states);
                if(step==Step::Wait){held[heldCount++]=s.get();++i;continue;}
                if(step==Step::Baked)c->accounting->completed.fetch_add(1,std::memory_order_relaxed);
                else{s->dropped_.fetch_add(1,std::memory_order_relaxed);c->accounting->dropped.fetch_add(1,std::memory_order_relaxed);}
                release(*c,w);erase(i);progress=true;
                // Shown as soon as the body's ring has room; later stains of a burst ride in the next delta.
                publishPending(*c,*s,states);
                // Deltas and cleared notices of the other bodies must not wait for a long backlog.
                for(auto& other:states)if(other&&other.get()!=s.get()&&!other->canceled())publishPending(*c,*other,states);
                if(c->stop.load(std::memory_order_relaxed))break;
            }
            for(auto& s:states)if(s&&!s->canceled()&&publishPending(*c,*s,states))progress=true;
            if(!progress)std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        // Producers which observed the old stop value may still be publishing.
        // Drain until their bounded, nonblocking critical regions have ended.
        while(c->producers.load(std::memory_order_acquire)){
            while(auto* p=c->retired.pop())delete p;
            while(auto* w=c->input.pop())release(*c,w);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        while(auto* p=c->retired.pop())delete p;
        while(auto* w=c->input.pop())release(*c,w);
        for(std::size_t i=0;i<backlogCount;++i)release(*c,backlog[i]);
        // Worker-only CPU storage is reclaimed before the explicit join ends.
        for(auto& s:states)if(s){owe(*c,s->unpublished_,false);owe(*c,s->clearOwed_,false);s->layer_=BloodCellLayer::Layer(s->layer_.frame());s->accounting_->layerBytes.fetch_sub(s->layerCredit_,std::memory_order_relaxed);s->layerCredit_=0;s->canceled_.store(true,std::memory_order_release);}
    }
public:
    explicit Service(Limits limits={},std::function<void()> workerInit={},std::function<void(Delta&)> finalize={}):control_(std::make_shared<Control>()) {
        control_->limits=limits;control_->workerInit=std::move(workerInit);control_->finalize=std::move(finalize);
        worker_=std::thread(&Service::run,control_);
    }
    Service(const Service&)=delete;Service& operator=(const Service&)=delete;
    // Safety fallback never joins on an arbitrary render/body destruction path.
    // Engine must call shutdownAndJoin from RuntimeGeometryStageShutdown.
    ~Service(){requestStop();if(worker_.joinable())worker_.detach();}
    std::shared_ptr<State> makeState(BloodCellLayer::Frame f={},std::function<void(Delta&)> finalize={}){return std::make_shared<State>(f,control_->accounting,std::move(finalize));}
    SubmitResult trySubmit(const std::shared_ptr<State>& s,std::unique_ptr<Job>& job) {
        auto& c=*control_;c.producers.fetch_add(1,std::memory_order_acq_rel);
        struct Leave {Control& c;~Leave(){c.producers.fetch_sub(1,std::memory_order_release);}} leave{c};
        if(c.stop.load(std::memory_order_acquire))return SubmitResult::Stopped;
        if(!s||!job)return SubmitResult::Invalid;if(s->canceled())return SubmitResult::Canceled;
        const bool keyed=job->markNamespace||job->markKey||job->markRevision;
        if(keyed&&(!job->markNamespace||!job->markKey||!job->markRevision))return SubmitResult::Invalid;
        if(job->markPreserveOldCoverage&&!keyed)return SubmitResult::Invalid;
        const auto bytes=job->bytes();
        if(!reserve(c.accounting->jobs,1,(std::min)(c.limits.maxJobs,std::size_t(1024))))return SubmitResult::Backpressure;
        if(!reserve(c.accounting->queuedBytes,bytes,c.limits.maxQueuedBytes)){c.accounting->jobs.fetch_sub(1,std::memory_order_relaxed);return SubmitResult::Backpressure;}
        std::unique_ptr<Work> w;
        try{w=std::make_unique<Work>();}catch(...){c.accounting->jobs.fetch_sub(1,std::memory_order_relaxed);c.accounting->queuedBytes.fetch_sub(bytes,std::memory_order_relaxed);return SubmitResult::Backpressure;}
        w->state=s;w->credit=bytes;w->job=std::move(job);
        if(!c.input.push(w.get())){job=std::move(w->job);c.accounting->jobs.fetch_sub(1,std::memory_order_relaxed);c.accounting->queuedBytes.fetch_sub(bytes,std::memory_order_relaxed);return SubmitResult::Backpressure;}
        w.release();c.accounting->submitted.fetch_add(1,std::memory_order_relaxed);return SubmitResult::Accepted;
    }
    // Retire completed packets on the worker so large vector destruction is not
    // hidden in the frame budget. On saturation caller retains the packet.
    bool tryRetire(std::unique_ptr<Delta>& d)noexcept {
        if(!d)return true;auto& c=*control_;c.producers.fetch_add(1,std::memory_order_acq_rel);
        struct Leave {Control& c;~Leave(){c.producers.fetch_sub(1,std::memory_order_release);}} leave{c};
        if(c.stop.load(std::memory_order_acquire)||!c.retired.push(d.get()))return false;d.release();return true;
    }
    const Accounting& accounting()const noexcept{return *control_->accounting;}
    const Limits& limits()const noexcept{return control_->limits;}
    // Once per owner frame (bloodfix): lets a GPU-quota eviction wait until the owner has had time to take it.
    void ownerFrame()noexcept{control_->accounting->ownerFrames.fetch_add(1,std::memory_order_relaxed);}
    void requestStop()noexcept {control_->stop.store(true,std::memory_order_release);}
    void shutdownAndJoin(){requestStop();if(worker_.joinable())worker_.join();}
};

// One object shared by the entire render pump, reset once per Device.dwFrame.
// The deadline controls admission of indivisible driver calls. It cannot preempt
// a driver call or OS descheduling; report measured overshoot instead of claiming
// a hard real-time guarantee. GuardUs reserves measured call headroom.
class FrameBudget {
    Clock::time_point begin_{},deadline_{};std::uint64_t frame_=~std::uint64_t(0);
public:
    void beginFrame(std::uint64_t frame,std::uint32_t microseconds=500)noexcept {if(frame==frame_)return;frame_=frame;begin_=Clock::now();deadline_=begin_+std::chrono::microseconds((std::min)(microseconds,500u));}
    bool permits(std::uint32_t guardUs=50)const noexcept{return Clock::now()+std::chrono::microseconds(guardUs)<deadline_;}
    double elapsedMicroseconds()const noexcept{return std::chrono::duration<double,std::micro>(Clock::now()-begin_).count();}
};
} // namespace BloodBakeAsync
