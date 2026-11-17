#include "RoomExposureCache.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <new>
#include <type_traits>

namespace painter { namespace combat { namespace core {
namespace {
struct Cancelled {};
struct TransactionExhausted {};
struct RevisionExhausted {};
void poll(const std::function<bool()>& f){if(f&&f())throw Cancelled{};}
bool same(const ExposureGeometryStamp& a,const ExposureGeometryStamp& b){return a.level_epoch==b.level_epoch&&a.map_hash==b.map_hash
    &&a.geometry_revision==b.geometry_revision&&a.data_revision==b.data_revision&&a.source_basis==b.source_basis;}
bool point_equal(Vec3 a,Vec3 b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
bool same(const ExposureDestination& a,const ExposureDestination& b){return a.building==b.building&&a.near_room==b.near_room
    &&a.node==b.node&&point_equal(a.point,b.point)&&a.paired_openings==b.paired_openings;}
bool destination_valid(const ExposureDestination& d){return d.building!=invalid_room&&d.node!=invalid_node&&finite(d.point);}
Vec3 sub(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
double dot(Vec3 a,Vec3 b){return double(a.x)*b.x+double(a.y)*b.y+double(a.z)*b.z;}
double dist2(Vec3 a,Vec3 b){return dot(sub(a,b),sub(a,b));}
enum Line : unsigned char {unknown=0,blocked=1,clear=2};
Line aperture(const ExposureOpening& p,Vec3 a,Vec3 b,double* crossing=nullptr){
    const double n=std::hypot(double(p.normal.x),double(p.normal.z));
    const double aa=((double(a.x)-p.centre.x)*p.normal.x+(double(a.z)-p.centre.z)*p.normal.z)/n;
    const double bb=((double(b.x)-p.centre.x)*p.normal.x+(double(b.z)-p.centre.z)*p.normal.z)/n;
    const double denom=aa-bb;
    if(!std::isfinite(denom))return unknown;
    if(denom==0)return aa==0?unknown:blocked;
    const double f=aa/denom;if(f<0||f>1)return blocked;
    const double x=double(a.x)+(double(b.x)-a.x)*f,z=double(a.z)+(double(b.z)-a.z)*f;
    const double y=double(a.y)+(double(b.y)-a.y)*f;
    const double tangent=((x-p.centre.x)*-p.normal.z+(z-p.centre.z)*p.normal.x)/n;
    if(y>=p.y0&&y<=p.y1&&std::abs(tangent)<=p.half_width){if(crossing)*crossing=f;return clear;}
    return blocked;
}
Line through(const ExposureSourceOpening&link,const ExposureOpening*openings,Vec3 source,Vec3 destination,
    std::uint32_t building,std::uint32_t room,bool allow_pair){
    const auto&first=openings[link.opening];
    const bool same_building=building==first.building||(building==invalid_room&&room==invalid_room);
    if(!same_building)return blocked;
    // The first leg of a supplied pair also describes a valid direct opening
    // into its intermediate room; no duplicate source contribution is added.
    if(room==first.near_room)return aperture(first,source,destination);
    if(!allow_pair||link.second_opening==ExposureSourceOpening::no_second)return blocked;
    const auto&second=openings[link.second_opening];
    if(room!=second.near_room)return blocked;
    double a=0,b=0;const Line one=aperture(first,source,destination,&a),two=aperture(second,source,destination,&b);
    if(one==blocked||two==blocked)return blocked;
    if(one==unknown||two==unknown)return unknown;
    if(a==b)return unknown; // coincident crossing cannot certify an intermediate room
    return a<b?clear:blocked;
}
bool policy_valid(const ExposureCachePolicy& p){return p.column_capacity&&p.journal_records>=2
    &&std::isfinite(p.source_floor_m)&&p.source_floor_m>=0
    &&std::isfinite(p.source_range_m)&&p.source_range_m>0&&std::isfinite(p.observation_range_m)&&p.observation_range_m>0
    &&std::isfinite(p.half_fov_deg)&&p.half_fov_deg>=0&&p.half_fov_deg<=180;}
bool checked_add(std::size_t& n,std::size_t x){if(x>(std::numeric_limits<std::size_t>::max)()-n)return false;n+=x;return true;}
template<class T> bool extent(std::size_t& bytes,std::size_t n,std::size_t& offset){
    const auto padding=(alignof(T)-bytes%alignof(T))%alignof(T);
    if(!checked_add(bytes,padding)||n>(std::numeric_limits<std::size_t>::max)()/sizeof(T))return false;
    offset=bytes;return checked_add(bytes,n*sizeof(T));
}
template<class T> T* initialize(unsigned char* base,std::size_t offset,std::size_t count){
    T* p=reinterpret_cast<T*>(base+offset);for(std::size_t i=0;i<count;++i)::new(static_cast<void*>(p+i))T{};return p;
}
}
struct RoomExposureCache::Impl {
    struct Source {ExposureSource value;std::size_t link_begin=0,links=0;};
    struct Sight {ActorKey actor;std::uint64_t owner=0,seq=0,authority=0;TimeMs at=0;};
    struct Column {
        ExposureDestination key;bool used=false,complete=false,source_unknown=false;
        std::uint64_t touch=0,revision=0,sequence=0,attempt=0;TimeMs at=0,until=0;
        std::size_t source=0,link=0,seen=0,seen_clear=0,red=0,blocked_count=0,unknown_count=0;
        // Independent static cursor survives sight revision/outer rollback.
        // Pricing cannot force a cold line scan to restart at source zero.
        std::size_t static_source=0,static_link=0;
        std::uint64_t static_attempt=0;
        bool static_complete=false,static_unknown=false,static_has_unknown=false;
    };
    struct Undo {void* address=nullptr;std::size_t bytes=0;unsigned char before[192];};
    struct SightUndo {std::size_t index=0;Sight before;};
    struct CaptureBackup {
        std::uint64_t authority=0,sequence=0,group=0,roster=0;
        std::uint32_t side=0,faction=0;TimeMs now=0;
        std::size_t observer_count=0,source=0,observer=0,link=0;
        bool contact=false,capture=false,observation_turn=false;
    } backup;
    struct Progress {
        std::size_t column=0,source=0,observer=0,link=0;
        std::uint64_t revision=1,attempt=0;bool observation_turn=false;
    } progress;
    ExposureGeometryStamp stamp;Source* sources=nullptr;ExposureOpening* openings=nullptr;ExposureSourceOpening* links=nullptr;
    Sight* sights=nullptr;ExposureObserver* observers=nullptr;Column* columns=nullptr;unsigned char* cells=nullptr;Undo* undo=nullptr;
    ExposureObserver* observer_staging=nullptr;ExposureObserver* observer_backup=nullptr;
    SightUndo* transaction_undo=nullptr;std::uint64_t* sight_tokens=nullptr;
    std::size_t source_count=0,opening_count=0,link_count=0,observer_count=0,column_count=0,payload=0,charged=0,undo_count=0;
    std::uint64_t authority=1,touches=0,evictions=0,sequence=0,group=0,roster=0;
    std::uint32_t side=0,faction=0;TimeMs now=0;bool contact=false,capture=false,paired_complete=false;
    std::uint64_t transaction_token=0;
    std::size_t transaction_count=0,transaction_bytes=0,restore_observer=0;
    bool transaction_active=false,transaction_failed=false,rolling_back=false,revision_exhausted=false;
    template<class T> void remember(T& value){static_assert(std::is_trivially_copyable<T>::value,"journal holds value records only");
        static_assert(sizeof(T)<=sizeof(Undo::before),"journal record too small");
        auto& u=undo[undo_count++];u.address=&value;u.bytes=sizeof(T);std::memcpy(u.before,&value,sizeof(T));}
    void rollback(){while(undo_count){const auto& u=undo[--undo_count];std::memcpy(u.address,u.before,u.bytes);}}
    void revise(){if(progress.revision==(std::numeric_limits<std::uint64_t>::max)())throw RevisionExhausted{};++progress.revision;}
    bool closed()const{return revision_exhausted||transaction_failed||rolling_back;}
    void remember_sight(std::size_t i,std::size_t capacity){
        if(!transaction_active||sight_tokens[i]==transaction_token)return;
        if(transaction_count==capacity)throw TransactionExhausted{};
        transaction_undo[transaction_count++]={i,sights[i]};sight_tokens[i]=transaction_token;
    }
    TimeMs age(const ExposureCachePolicy& p)const{return contact?p.seen_contact_ms:p.seen_ms;}
    bool seen(std::size_t i,const ExposureCachePolicy& p)const{return sights[i].authority==authority&&fresh(sights[i].at,now,age(p));}
    TimeMs expiry(std::size_t i,const ExposureCachePolicy& p)const{return saturated_after(saturated_after(sights[i].at,age(p)),1);}
    bool priced(const Column& c)const{return c.complete&&c.revision==progress.revision&&(!c.until||now<c.until);}
    void reset_price(Column& c){c.complete=false;c.source_unknown=false;c.source=c.link=c.seen=c.seen_clear=c.red=c.blocked_count=c.unknown_count=0;
        c.until=0;c.revision=progress.revision;c.sequence=sequence;c.at=now;}
    void finish_source(Column& c,Line line,const ExposureCachePolicy& p){
        const bool known=seen(c.source,p);if(known){++c.seen;const auto until=expiry(c.source,p);if(!c.until||until<c.until)c.until=until;}
        if(line==unknown)++c.unknown_count;else if(line==blocked)++c.blocked_count;else if(known)++c.seen_clear;else ++c.red;
        ++c.source;c.link=0;c.source_unknown=false;
        if(c.source==source_count){c.complete=true;c.sequence=sequence;c.at=now;}
    }
    void finish_static(Column& c,Line line){
        c.static_has_unknown=c.static_has_unknown||line==unknown;
        c.static_link=0;c.static_unknown=false;
        if(++c.static_source==source_count){c.static_complete=true;c.static_attempt=progress.attempt;}
    }
    bool next_observer(){progress.link=0;if(++progress.observer==observer_count){progress.observer=0;progress.source=(progress.source+1)%source_count;return true;}return false;}
    void next_source(){progress.observer=progress.link=0;progress.source=(progress.source+1)%source_count;}
};
RoomExposureCache::RoomExposureCache(ExposureCachePolicy p):policy_(p){}
RoomExposureCache::~RoomExposureCache(){reset();}
void RoomExposureCache::reset(){if(impl_)impl_->~Impl();impl_=nullptr;storage_.reset();status_=ExposureCacheStatus::invalid;}
ExposureCacheStatus RoomExposureCache::bind(const ExposureGeometryInput& input){
    if(impl_&&impl_->transaction_active)return ExposureCacheStatus::transaction_busy;
    reset();if(!policy_.memory_bytes)return status_=ExposureCacheStatus::disabled;
    if(!policy_valid(policy_)||!input.stamp.level_epoch||!input.stamp.geometry_revision||!input.stamp.source_basis||input.sources.empty())return status_;
    for(std::size_t i=0;i<input.sources.size();++i){const auto& s=input.sources[i];
        if(!s.id||(i&&s.id<=input.sources[i-1].id)||s.building==invalid_room||s.room==invalid_room||s.node==invalid_node||!finite(s.point))return status_;}
    for(std::size_t i=0;i<input.openings.size();++i){const auto& o=input.openings[i];
        if(!o.id||o.building==invalid_room||o.far_room==invalid_room||o.near_room==o.far_room||!finite(o.centre)||!finite(o.normal)
            ||std::hypot(double(o.normal.x),double(o.normal.z))<=0||!std::isfinite(o.half_width)||o.half_width<=0
            ||!std::isfinite(o.y0)||!std::isfinite(o.y1)||o.y1<=o.y0)return status_;
        for(std::size_t j=0;j<i;++j)if(input.openings[j].id==o.id)return status_;}
    bool has_pairs=false;
    for(std::size_t i=0;i<input.links.size();++i){const auto& l=input.links[i];
        if(l.source>=input.sources.size()||l.opening>=input.openings.size())return status_;
        if(i){const auto&old=input.links[i-1];if(l.source<old.source||(l.source==old.source&&(l.opening<old.opening||
            (l.opening==old.opening&&l.second_opening<old.second_opening))))return status_;}
        const auto& s=input.sources[l.source];const auto& o=input.openings[l.opening];if(s.building!=o.building||s.room!=o.far_room)return status_;
        if(l.second_opening!=ExposureSourceOpening::no_second){
            has_pairs=true;if(l.second_opening>=input.openings.size()||l.second_opening==l.opening)return status_;
            const auto& second=input.openings[l.second_opening];
            const double dx=double(o.centre.x)-second.centre.x,dz=double(o.centre.z)-second.centre.z;
            if(o.near_room==invalid_room||second.building!=o.building||second.far_room!=o.near_room||second.near_room==s.room||
                !std::isfinite(input.pair_max_m)||input.pair_max_m<=0||dx*dx+dz*dz>double(input.pair_max_m)*input.pair_max_m)return status_;
        }
    }
    if((has_pairs||input.paired_links_complete)&&(!std::isfinite(input.pair_max_m)||input.pair_max_m<=0))return status_;
    std::size_t bytes=0,im=0,ss=0,os=0,ls=0,si=0,ob=0,st=0,co=0,ce=0,un=0,tb=0,tu=0,tm=0;
    if(policy_.column_capacity>(std::numeric_limits<std::size_t>::max)()/input.sources.size())return status_=ExposureCacheStatus::memory_limit;
    if(!extent<Impl>(bytes,1,im)||!extent<Impl::Source>(bytes,input.sources.size(),ss)||!extent<ExposureOpening>(bytes,input.openings.size(),os)
        ||!extent<ExposureSourceOpening>(bytes,input.links.size(),ls)||!extent<Impl::Sight>(bytes,input.sources.size(),si)
        ||!extent<ExposureObserver>(bytes,policy_.observer_capacity,ob)||!extent<ExposureObserver>(bytes,policy_.observer_capacity,st)
        ||!extent<Impl::Column>(bytes,policy_.column_capacity,co)
        ||!extent<unsigned char>(bytes,input.sources.size()*policy_.column_capacity,ce)||!extent<Impl::Undo>(bytes,policy_.journal_records,un))
        return status_=ExposureCacheStatus::memory_limit;
    const auto transaction_begin=bytes;
    if(policy_.transaction_records&&(!extent<ExposureObserver>(bytes,policy_.observer_capacity,tb)
        ||!extent<Impl::SightUndo>(bytes,policy_.transaction_records,tu)||!extent<std::uint64_t>(bytes,input.sources.size(),tm)))
        return status_=ExposureCacheStatus::memory_limit;
    std::size_t charged=bytes;if(!checked_add(charged,sizeof(*this))||!checked_add(charged,policy_.allocation_overhead_bytes)||charged>policy_.memory_bytes)
        return status_=ExposureCacheStatus::memory_limit;
    try{storage_.reset(new unsigned char[bytes]);}catch(const std::bad_alloc&){return status_=ExposureCacheStatus::memory_limit;}
    impl_=initialize<Impl>(storage_.get(),im,1);auto& d=*impl_;d.stamp=input.stamp;d.source_count=input.sources.size();d.opening_count=input.openings.size();d.link_count=input.links.size();d.paired_complete=input.paired_links_complete;
    d.sources=initialize<Impl::Source>(storage_.get(),ss,d.source_count);d.openings=initialize<ExposureOpening>(storage_.get(),os,d.opening_count);
    d.links=initialize<ExposureSourceOpening>(storage_.get(),ls,d.link_count);d.sights=initialize<Impl::Sight>(storage_.get(),si,d.source_count);
    d.observers=initialize<ExposureObserver>(storage_.get(),ob,policy_.observer_capacity);d.columns=initialize<Impl::Column>(storage_.get(),co,policy_.column_capacity);
    d.observer_staging=initialize<ExposureObserver>(storage_.get(),st,policy_.observer_capacity);
    d.cells=initialize<unsigned char>(storage_.get(),ce,d.source_count*policy_.column_capacity);d.undo=initialize<Impl::Undo>(storage_.get(),un,policy_.journal_records);
    d.payload=bytes;d.charged=charged;
    d.transaction_bytes=bytes-transaction_begin;
    if(policy_.transaction_records){d.observer_backup=initialize<ExposureObserver>(storage_.get(),tb,policy_.observer_capacity);
        d.transaction_undo=initialize<Impl::SightUndo>(storage_.get(),tu,policy_.transaction_records);
        d.sight_tokens=initialize<std::uint64_t>(storage_.get(),tm,d.source_count);}
    for(std::size_t i=0;i<d.source_count;++i)d.sources[i].value=input.sources[i];
    for(std::size_t i=0;i<d.opening_count;++i)d.openings[i]=input.openings[i];
    for(std::size_t i=0;i<d.link_count;++i){d.links[i]=input.links[i];auto& s=d.sources[d.links[i].source];if(!s.links)s.link_begin=i;++s.links;}
    return status_=ExposureCacheStatus::pending;
}
ExposureCacheTransaction RoomExposureCache::begin_transaction(const std::function<bool()>& stop){
    ExposureCacheTransaction out;if(!impl_)return out;auto& d=*impl_;
    if(d.transaction_active){out.status=ExposureCacheStatus::transaction_busy;return out;}
    if(!policy_.transaction_records){out.status=ExposureCacheStatus::disabled;return out;}
    if(d.revision_exhausted||transaction_serial_==(std::numeric_limits<std::uint64_t>::max)())return out;
    try{poll(stop);for(std::size_t i=0;i<d.observer_count;++i){poll(stop);d.observer_backup[i]=d.observers[i];}poll(stop);}
    catch(const Cancelled&){out.status=ExposureCacheStatus::cancelled;return out;}
    catch(...){out.status=ExposureCacheStatus::query_failure;return out;}
    d.backup={d.authority,d.sequence,d.group,d.roster,d.side,d.faction,d.now,d.observer_count,
        d.progress.source,d.progress.observer,d.progress.link,d.contact,d.capture,d.progress.observation_turn};
    d.transaction_token=++transaction_serial_;d.transaction_active=true;d.transaction_failed=false;d.rolling_back=false;
    d.transaction_count=d.restore_observer=0;
    out.token=d.transaction_token;out.active=true;out.status=ExposureCacheStatus::pending;return out;
}
ExposureCacheTransaction RoomExposureCache::commit_transaction(std::uint64_t token,const std::function<bool()>& stop){
    ExposureCacheTransaction out;out.token=token;
    if(!impl_||!token||!impl_->transaction_active||token!=impl_->transaction_token)return out;auto& d=*impl_;out.active=true;
    if(d.rolling_back){out.status=ExposureCacheStatus::transaction_busy;return out;}
    if(d.transaction_failed){out.status=ExposureCacheStatus::transaction_exhausted;return out;}
    if(d.revision_exhausted)return out;
    try{poll(stop);}catch(const Cancelled&){out.status=ExposureCacheStatus::cancelled;return out;}
    catch(...){out.status=ExposureCacheStatus::query_failure;return out;}
    d.transaction_active=false;d.transaction_count=0;d.transaction_token=0;out.active=false;out.committed=true;out.status=ExposureCacheStatus::complete;return out;
}
ExposureCacheTransaction RoomExposureCache::rollback_transaction(std::uint64_t token,const std::function<bool()>& stop){
    ExposureCacheTransaction out;out.token=token;
    if(!impl_||!token||!impl_->transaction_active||token!=impl_->transaction_token)return out;auto& d=*impl_;out.active=true;
    // An interrupted restore never exposes a mixture of old and new authority.
    d.rolling_back=true;d.capture=false;
    try{
        poll(stop);
        while(d.transaction_count){poll(stop);const auto& u=d.transaction_undo[--d.transaction_count];d.sights[u.index]=u.before;++out.restored;}
        while(d.restore_observer<d.backup.observer_count){poll(stop);const auto i=d.restore_observer++;d.observers[i]=d.observer_backup[i];++out.restored;}
        poll(stop);
    }catch(const Cancelled&){out.status=ExposureCacheStatus::cancelled;return out;}
    catch(...){out.status=ExposureCacheStatus::query_failure;return out;}
    const auto& b=d.backup;d.authority=b.authority;d.sequence=b.sequence;d.group=b.group;d.roster=b.roster;
    d.side=b.side;d.faction=b.faction;d.now=b.now;d.observer_count=b.observer_count;d.contact=b.contact;
    d.progress.source=b.source;d.progress.observer=b.observer;d.progress.link=b.link;d.progress.observation_turn=b.observation_turn;
    // Do not restore a former pricing revision: tentative columns could have
    // that value again later and revive a discarded seen-source discount.
    try{d.revise();}catch(const RevisionExhausted&){d.revision_exhausted=true;}
    d.capture=b.capture&&!d.revision_exhausted;d.transaction_failed=d.rolling_back=d.transaction_active=false;d.transaction_token=0;
    out.active=false;out.rolled_back=true;out.status=d.revision_exhausted?ExposureCacheStatus::invalid:ExposureCacheStatus::complete;return out;
}
bool RoomExposureCache::begin_capture(const ExposureCapture& capture,const std::function<bool()>& stop,bool reset_observations){
    if(!impl_)return false;auto& d=*impl_;
    if(d.closed())return false;
    // A failed intake closes the view, so a caller cannot accidentally consume
    // a previously active side's discounts after asking for a different side.
    const bool had_capture=d.capture;d.capture=false;
    if(!same(capture.stamp,d.stamp)||!capture.snapshot_seq||!capture.now_ms||capture.observers.size()>policy_.observer_capacity
        ||(d.sequence&&(capture.snapshot_seq<=d.sequence||capture.now_ms<d.now)))return false;
    try{poll(stop);
    for(std::size_t i=0;i<capture.observers.size();++i){poll(stop);const auto& o=capture.observers[i];
        if(!valid_key(o.actor)||!o.owner||o.faction!=capture.faction||o.snapshot_seq!=capture.snapshot_seq||o.observed_at_ms>capture.now_ms
            ||(i&&o.actor.id<=capture.observers[i-1].actor.id))return false;
        d.observer_staging[i]=o;}
    bool roster=had_capture&&!reset_observations&&capture.side==d.side&&capture.group==d.group&&capture.faction==d.faction&&capture.roster_revision==d.roster
        &&capture.observers.size()==d.observer_count;
    for(std::size_t i=0;roster&&i<capture.observers.size();++i){poll(stop);roster=capture.observers[i].actor==d.observers[i].actor&&capture.observers[i].owner==d.observers[i].owner;}
    poll(stop);
    if(!roster){if(d.authority==(std::numeric_limits<std::uint64_t>::max)())return false;d.revise();++d.authority;d.progress.source=d.progress.observer=d.progress.link=0;}
    else if(capture.contact_active!=d.contact)d.revise();
    std::swap(d.observers,d.observer_staging);
    d.side=capture.side;d.group=capture.group;d.faction=capture.faction;d.roster=capture.roster_revision;d.observer_count=capture.observers.size();
    d.sequence=capture.snapshot_seq;d.now=capture.now_ms;d.contact=capture.contact_active;d.capture=true;return true;
    }catch(const RevisionExhausted&){d.revision_exhausted=true;return false;}
    catch(...){return false;}
}
bool RoomExposureCache::matches_capture(const ExposureCapture& capture)const{
    if(!impl_||!impl_->capture||impl_->closed())return false;const auto&d=*impl_;
    if(!same(capture.stamp,d.stamp)||capture.side!=d.side||capture.group!=d.group||capture.faction!=d.faction||
        capture.roster_revision!=d.roster||capture.snapshot_seq!=d.sequence||capture.now_ms!=d.now||
        capture.contact_active!=d.contact||capture.observers.size()!=d.observer_count)return false;
    for(std::size_t n=0;n<d.observer_count;++n){const auto&a=capture.observers[n];const auto&b=d.observers[n];
        if(a.actor!=b.actor||a.owner!=b.owner||a.snapshot_seq!=b.snapshot_seq||a.observed_at_ms!=b.observed_at_ms||
            a.faction!=b.faction||a.building!=b.building||a.room!=b.room||a.eligible!=b.eligible||
            !point_equal(a.eye,b.eye)||!point_equal(a.head,b.head))return false;
    }
    return true;
}
ExposureCacheQuote RoomExposureCache::quote(const ExposureDestination& key)const{
    ExposureCacheQuote out;out.status=status_;if(!impl_)return out;const auto& d=*impl_;out.source_basis=d.stamp.source_basis;
    out.total_sources=out.unknown_sources=d.source_count;out.observation_revision=d.progress.revision;
    out.transaction_token=d.transaction_active?d.transaction_token:0;
    if(d.transaction_failed){out.status=ExposureCacheStatus::transaction_exhausted;return out;}
    if(d.closed()||!d.capture||!destination_valid(key)){out.status=ExposureCacheStatus::invalid;return out;}
    if(key.paired_openings&&!d.paired_complete){out.status=ExposureCacheStatus::unsupported;return out;}
    out.status=ExposureCacheStatus::queued;
    for(std::size_t i=0;i<policy_.column_capacity;++i){const auto& c=d.columns[i];if(!c.used||!same(c.key,key))continue;
        out.status=ExposureCacheStatus::pending;if(!d.priced(c)||c.unknown_count)return out;
        out.status=ExposureCacheStatus::complete;out.complete=true;out.pending=false;out.seen_sources=c.seen;out.seen_clear_sources=c.seen_clear;
        out.red_sources=c.red;out.blocked_sources=c.blocked_count;out.unknown_sources=0;out.priced_snapshot_seq=c.sequence;out.priced_at_ms=c.at;
        out.seen_valid_until_ms=c.until;return out;}
    return out;
}
ExposureCacheStatus RoomExposureCache::enqueue(const ExposureDestination& key,const std::function<bool()>& stop){
    if(!impl_)return status_;auto& d=*impl_;if(d.transaction_failed)return ExposureCacheStatus::transaction_exhausted;
    if(d.closed()||!d.capture||!destination_valid(key))return ExposureCacheStatus::invalid;
    if(key.paired_openings&&!d.paired_complete)return ExposureCacheStatus::unsupported;
    std::size_t oldest=0;
    try{poll(stop);
    for(std::size_t i=0;i<policy_.column_capacity;++i){auto& c=d.columns[i];
        poll(stop);
        if(c.used&&same(c.key,key)){c.touch=++d.touches;return quote(key).status;}
        if(!c.used){oldest=i;break;}if(c.touch<d.columns[oldest].touch)oldest=i;}
    auto& c=d.columns[oldest];if(c.used){++d.evictions;--d.column_count;}c.used=false;
    // Closing a static column before clearing prevents a cancelled eviction
    // from pairing old keys with partially cleared cells. Static work is a
    // disposable optimization, not side provenance requiring outer undo.
    auto* cells=d.cells+oldest*d.source_count;
    for(std::size_t i=0;i<d.source_count;++i){poll(stop);cells[i]=0;}poll(stop);
    c=Impl::Column{};c.used=true;c.key=key;c.touch=++d.touches;d.reset_price(c);
    ++d.column_count;return ExposureCacheStatus::queued;
    }catch(const Cancelled&){return ExposureCacheStatus::cancelled;}
    catch(...){return ExposureCacheStatus::query_failure;}
}
ExposureCacheAdvance RoomExposureCache::advance(const EngineServices& svc,ExposureWorkQuota& quota,const std::function<bool()>& stop){
    ExposureCacheAdvance out;out.status=status_;if(!impl_)return out;auto& d=*impl_;
    if(d.transaction_failed){out.status=ExposureCacheStatus::transaction_exhausted;return out;}
    if(d.closed()||!d.capture){out.status=ExposureCacheStatus::invalid;return out;}
    const auto saved=d.progress;const auto start_work=quota.work_used,start_queries=quota.queries_used;d.undo_count=0;
    if(d.progress.attempt==(std::numeric_limits<std::uint64_t>::max)()){out.status=ExposureCacheStatus::invalid;return out;}
    ++d.progress.attempt;
    auto query=[&](Vec3 a,Vec3 b){poll(stop);--quota.queries_left;++quota.queries_used;Segment s{a,b};float hit=-2;svc.rays(&s,&hit,1);poll(stop);
        const double length=std::sqrt(dist2(a,b));if(!std::isfinite(hit)||!std::isfinite(length))return unknown;
        return hit==-1.f||double(hit)>=length?clear:hit>=0?blocked:unknown;};
    std::size_t idle_columns=0,observation_sources_left=d.source_count;
    const auto next_observer=[&](){if(d.next_observer())--observation_sources_left;};
    try{
        poll(stop);
        while(quota.work_left&&(d.column_count||d.observer_count)){
            poll(stop);
            // One observation sweep per immutable capture view is enough for
            // this call. A ray-less sweep cannot create any new sight facts.
            // Keep pricing cached/aperture-only columns after rays run out.
            const bool can_observe=d.observer_count&&observation_sources_left&&svc.rays&&quota.queries_left;
            if(!can_observe&&(!d.column_count||idle_columns>=policy_.column_capacity))break;
            // Reserve the worst touched-record count before beginning a unit.
            if(policy_.journal_records-d.undo_count<2){out.journal_exhausted=true;break;}
            --quota.work_left;++quota.work_used;
            const bool observe=can_observe&&(d.progress.observation_turn||!d.column_count);
            d.progress.observation_turn=!d.progress.observation_turn;
            if(observe){
                const auto index=d.progress.source;const auto& s=d.sources[index];
                if(d.seen(index,policy_)){d.next_source();--observation_sources_left;continue;}
                const auto& o=d.observers[d.progress.observer];const auto delta=sub(s.value.point,o.eye);
                const double length=std::sqrt(dot(delta,delta)),heading=std::sqrt(dot(o.head,o.head));
                const bool can=o.eligible&&fresh(o.observed_at_ms,d.now,policy_.observer_max_age_ms)&&finite(o.eye)&&finite(o.head)
                    &&std::isfinite(length)&&std::isfinite(heading)&&length>0&&heading>0&&length<=policy_.observation_range_m
                    &&dot(delta,o.head)>=length*heading*std::cos(double(policy_.half_fov_deg)*3.14159265358979323846/180.0);
                if(!can){next_observer();continue;}
                bool through=o.building==s.value.building&&o.room==s.value.room;
                if(!through){
                    if(d.progress.link>=s.links){next_observer();continue;}
                    const auto&link=d.links[s.link_begin+d.progress.link];
                    through=core::through(link,d.openings,s.value.point,o.eye,o.building,o.room,true)==clear;
                    if(!through){if(++d.progress.link==s.links)next_observer();continue;}
                }
                const auto line=query(o.eye,s.value.point);
                if(line==clear){auto& sight=d.sights[index];d.remember_sight(index,policy_.transaction_records);d.remember(sight);d.revise();
                    sight={o.actor,o.owner,o.snapshot_seq,d.authority,o.observed_at_ms};
                    idle_columns=0;d.next_source();--observation_sources_left;}else next_observer();
            }else{
                const auto index=d.progress.column;d.progress.column=(index+1)%policy_.column_capacity;auto& c=d.columns[index];
                if(!c.used||(d.priced(c)&&(!c.unknown_count||c.attempt==d.progress.attempt))){
                    if(idle_columns<policy_.column_capacity)++idle_columns;
                    continue;
                }
                const auto before=c;
                d.remember(c);
                if(c.revision!=d.progress.revision||c.complete||(c.until&&d.now>=c.until))d.reset_price(c);
                if(c.static_complete&&c.static_has_unknown&&c.static_attempt!=d.progress.attempt){
                    c.static_complete=c.static_has_unknown=false;c.static_source=c.static_link=0;c.static_unknown=false;d.reset_price(c);
                }
                if(!c.static_complete){
                    const auto static_index=c.static_source;
                    const auto& s=d.sources[c.static_source];auto& cell=d.cells[index*d.source_count+c.static_source];
                    if(cell){d.finish_static(c,static_cast<Line>(cell));}
                    else if((policy_.source_floor_m>0&&std::fabs(double(s.value.point.y)-c.key.point.y)>policy_.source_floor_m)||
                        dist2(s.value.point,c.key.point)>double(policy_.source_range_m)*policy_.source_range_m){d.remember(cell);cell=blocked;d.finish_static(c,blocked);}
                    else if(c.static_link>=s.links){d.remember(cell);cell=c.static_unknown?unknown:blocked;d.finish_static(c,static_cast<Line>(cell));}
                    else{
                        const auto path=through(d.links[s.link_begin+c.static_link],d.openings,s.value.point,c.key.point,c.key.building,c.key.near_room,c.key.paired_openings);
                        if(path!=clear){c.static_unknown=c.static_unknown||path==unknown;++c.static_link;
                            if(c.static_link==s.links){d.remember(cell);cell=c.static_unknown?unknown:blocked;d.finish_static(c,static_cast<Line>(cell));}}
                        else if(!svc.rays){d.finish_static(c,unknown);}
                        else if(quota.queries_left){const auto line=query(s.value.point,c.key.point);d.remember(cell);cell=line;d.finish_static(c,line);}
                    }
                    // In the ordinary forward pass price the same freshly
                    // resolved source now; separate cursors must not double
                    // the work or discard spare work after the ray quota.
                    if(c.static_source!=static_index&&c.source==static_index)d.finish_source(c,static_cast<Line>(cell),policy_);
                }else{
                    const auto cell=d.cells[index*d.source_count+c.source];d.finish_source(c,static_cast<Line>(cell),policy_);
                }
                if(c.complete){c.attempt=d.progress.attempt;if(!c.unknown_count&&c.revision==d.progress.revision)++out.columns_completed;}
                const bool changed=c.source!=before.source||c.complete!=before.complete||c.revision!=before.revision
                    ||c.static_source!=before.static_source||c.static_link!=before.static_link||c.static_complete!=before.static_complete;
                if(changed)idle_columns=0;else if(idle_columns<policy_.column_capacity)++idle_columns;
            }
        }
        poll(stop);out.committed=true;out.status=ExposureCacheStatus::pending;d.undo_count=0;
    }catch(const Cancelled&){d.rollback();d.progress=saved;out.cancelled=true;out.status=ExposureCacheStatus::cancelled;out.columns_completed=0;}
    catch(const TransactionExhausted&){d.rollback();d.progress=saved;d.transaction_failed=true;d.capture=false;
        out.journal_exhausted=true;out.status=ExposureCacheStatus::transaction_exhausted;out.columns_completed=0;}
    catch(const RevisionExhausted&){d.rollback();d.progress=saved;d.revision_exhausted=true;d.capture=false;out.status=ExposureCacheStatus::invalid;out.columns_completed=0;}
    catch(...){d.rollback();d.progress=saved;out.status=ExposureCacheStatus::query_failure;out.columns_completed=0;}
    out.work=quota.work_used-start_work;out.queries=quota.queries_used-start_queries;return out;
}
ExposureCacheSight RoomExposureCache::sight(std::uint64_t source)const{
    ExposureCacheSight out;out.source=source;if(!impl_||!impl_->capture||impl_->closed())return out;const auto& d=*impl_;
    out.transaction_token=d.transaction_active?d.transaction_token:0;
    std::size_t lo=0,hi=d.source_count;while(lo<hi){const auto mid=lo+(hi-lo)/2;if(d.sources[mid].value.id<source)lo=mid+1;else hi=mid;}
    if(lo==d.source_count||d.sources[lo].value.id!=source||!d.seen(lo,policy_))return out;
    const auto& s=d.sights[lo];out.valid=true;out.observer=s.actor;out.owner=s.owner;out.snapshot_seq=s.seq;out.observed_at_ms=s.at;
    out.valid_until_ms=d.expiry(lo,policy_);return out;
}
ExposureCacheStats RoomExposureCache::stats()const{ExposureCacheStats out;if(!impl_)return out;const auto& d=*impl_;
    out.charged_bytes=d.charged;out.payload_bytes=d.payload;out.column_capacity=policy_.column_capacity;out.columns=d.column_count;out.sources=d.source_count;
    out.evictions=d.evictions;out.observation_revision=d.progress.revision;
    out.transaction_storage_bytes=d.transaction_bytes;out.transaction_capacity=policy_.transaction_records;out.transaction_records=d.transaction_count;
    out.transaction_token=d.transaction_token;out.transaction_active=d.transaction_active;out.transaction_failed=d.transaction_failed;out.rolling_back=d.rolling_back;return out;}
}}}
