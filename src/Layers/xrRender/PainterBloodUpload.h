#pragma once
#include "BloodBakeAsync.h"
#include <d3d11.h>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

namespace PainterBloodUpload {
template<class T> struct Com {
    T* p=nullptr;
    Com()=default;
    ~Com(){if(p)p->Release();}
    Com(const Com&)=delete; Com& operator=(const Com&)=delete;
};
inline void require(HRESULT hr,const char* why){if(hr==E_OUTOFMEMORY)throw std::bad_alloc();if(FAILED(hr))throw std::runtime_error(why);}
inline std::atomic<std::size_t>& nativePayload(){static auto* bytes=new std::atomic<std::size_t>{0};return *bytes;}
// All bodies' native blood textures together (both banks, staging rows). r__blood_bake_memory sets it before the first stain.
inline std::atomic<std::size_t>& nativePayloadLimit(){static auto* bytes=new std::atomic<std::size_t>{512u*1024u*1024u};return *bytes;}
inline std::atomic<std::size_t>& nativePayloadPeak(){static auto* bytes=new std::atomic<std::size_t>{0};return *bytes;}
struct Texture {
    Com<ID3D11Texture2D> texture;
    Com<ID3D11ShaderResourceView> view;
    unsigned width=0,height=0,texelBytes=16;
    std::size_t chargedBytes=0;
    unsigned rowBytes()const noexcept{return width*texelBytes;}
    ~Texture(){if(view.p){view.p->Release();view.p=nullptr;}if(texture.p){texture.p->Release();texture.p=nullptr;}
        if(chargedBytes)nativePayload().fetch_sub(chargedBytes,std::memory_order_relaxed);}
};
inline std::shared_ptr<Texture> makeTexture(ID3D11Device* device,unsigned width,unsigned height,
    const void* initial=nullptr,bool view=true)
{
    if(!device||!width||!height||width>16384||height>16384)throw std::runtime_error("blood_upload_texture_dimensions");
    auto result=std::make_shared<Texture>();result->width=width;result->height=height;
    const auto bytes=std::size_t(width)*height*result->texelBytes;
    // Scheduler retains and retries the already-applied FIFO stamp after
    // earlier packets release credits; quota pressure is never a bad asset.
    if(!BloodBakeAsync::reserve(nativePayload(),bytes,nativePayloadLimit().load(std::memory_order_relaxed)))throw std::bad_alloc();
    result->chargedBytes=bytes;BloodBakeAsync::notePeak(nativePayloadPeak(),nativePayload().load(std::memory_order_relaxed));
    D3D11_TEXTURE2D_DESC d{};d.Width=width;d.Height=height;d.MipLevels=d.ArraySize=1;
    d.Format=DXGI_FORMAT_R32G32B32A32_UINT;d.SampleDesc.Count=1;d.Usage=D3D11_USAGE_DEFAULT;
    d.BindFlags=view?D3D11_BIND_SHADER_RESOURCE:0;
    D3D11_SUBRESOURCE_DATA data{};data.pSysMem=initial;data.SysMemPitch=width*result->texelBytes;
    require(device->CreateTexture2D(&d,initial?&data:nullptr,&result->texture.p),"blood_upload_create_texture");
    if(view)require(device->CreateShaderResourceView(result->texture.p,nullptr,&result->view.p),"blood_upload_create_view");
    return result;
}
struct Prepared {
    ID3D11Device* deviceIdentity=nullptr; // resources pin its lifetime
    // The body's table this packet belongs to: the new one (full) or the live one its rows go into.
    std::shared_ptr<Texture> table;
    std::shared_ptr<Texture> rows;        // the changed rows, in targetRows order (no view: copy source only)
    std::vector<unsigned> targetRows;     // ascending
    // The owner hands the table it replaced here; the packet's retirement releases it on the worker.
    std::shared_ptr<Texture> retired;
    std::uint64_t sequence=0;
    unsigned height=0;
    bool full=false;
    bool cleared=false; // the worker dropped the body's layer (or it is empty): the owner releases the body's texture
};

// One instance per serial body lineage worker. The live table is created here (free-threaded device) and pinned by every
// packet that refers to it; the owner only binds it and copies rows into it.
class Preparer {
    Com<ID3D11Device> device_;
    std::shared_ptr<Texture> table_;
public:
    explicit Preparer(ID3D11Device* device){if(!device)throw std::runtime_error("blood_upload_device");device_.p=device;device->AddRef();}
    Preparer(const Preparer&)=delete;Preparer&operator=(const Preparer&)=delete;
    ID3D11Device* device()const noexcept{return device_.p;}
    void prepare(BloodBakeAsync::Delta& delta);
};
inline void Preparer::prepare(BloodBakeAsync::Delta& delta){
    auto prepared=std::make_shared<Prepared>();prepared->deviceIdentity=device_.p;prepared->sequence=delta.sequence;prepared->height=delta.height;
    // The layer was dropped under pressure, or a snapshot of a body without blood: forget the table (its memory returns
    // when the owner has released its references too); the body's next delta is a whole new table.
    if(delta.cleared||(delta.fullSnapshot&&!delta.height)){table_.reset();prepared->cleared=true;prepared->full=true;delta.prepared=prepared;return;}
    const unsigned width=BloodCellLayer::Width;
    if(delta.height<=BloodCellLayer::FixedRows||delta.height>16384)throw std::runtime_error("blood_upload_table_height");
    if(delta.fullSnapshot){
        if(delta.rows.size()!=std::size_t(delta.height)*width)throw std::runtime_error("blood_upload_image_size");
        prepared->table=makeTexture(device_.p,width,delta.height,delta.rows.data(),true);prepared->full=true;
        // Commit cached resource state only after the complete packet exists.
        table_=prepared->table;delta.prepared=std::move(prepared);return;
    }
    if(!table_||table_->height!=delta.height)throw std::runtime_error("blood_upload_rows_without_table");
    if(delta.dirtyRows.empty()||delta.rows.size()!=delta.dirtyRows.size()*width)throw std::runtime_error("blood_upload_row_count");
    prepared->targetRows.reserve(delta.dirtyRows.size());
    for(const auto row:delta.dirtyRows){
        if(row>=delta.height||(!prepared->targetRows.empty()&&prepared->targetRows.back()>=row))throw std::runtime_error("blood_upload_row_order");
        prepared->targetRows.push_back(row);}
    prepared->rows=makeTexture(device_.p,width,unsigned(delta.dirtyRows.size()),delta.rows.data(),false);
    prepared->table=table_;delta.prepared=std::move(prepared);
}

// Used once for the whole renderer frame, shared by every body's pump and
// material/adoption work. Separate segments exclude time spent drawing scenes.
// A driver call cannot be preempted, so every operation is measured and the next one of a kind is admitted only when that
// kind's recent cost still fits (bloodfix); measured overruns are retained rather than hidden.
struct FrameBudget {
    using Clock=std::chrono::steady_clock;
    // Copy: a row copy into the live table. ColdCopy: a packet's first copy (its staging texture is new: the driver
    // allocates on first use). Publish: binding a whole new table. Material: one
    // step of a body material's blood clone. Admit: one stain handed to the worker.
    enum Kind : unsigned {Copy,ColdCopy,Publish,Material,Admit,Kinds};
    double limitSeconds=.0005,guardSeconds=.00010,usedSeconds=0;
    unsigned maxOperations=512,operations=0,depth=0,measured=0,batches=0;
    std::size_t maxCopiedBytes=4u*1024u*1024u,copiedBytes=0;
    Clock::time_point started;
    // The cost to expect of one operation of a kind: the second largest of its last 16 measured costs (one outlier - a
    // descheduled thread - does not hold a kind back for 16 operations), before two samples a fixed first guess.
    struct Costs {
        std::array<std::array<float,16>,Kinds> recent{};std::array<unsigned,Kinds> count{};
        std::array<float,Kinds> expect{{3e-6f,150e-6f,20e-6f,150e-6f,20e-6f}};
        std::array<float,Kinds> worst{};
        void note(Kind kind,float seconds)noexcept{
            auto& ring=recent[kind];ring[count[kind]%16u]=seconds;++count[kind];worst[kind]=(std::max)(worst[kind],seconds);
            const unsigned n=(std::min)(count[kind],16u);if(n<2)return;
            float first=0,second=0;for(unsigned i=0;i<n;++i){const float v=ring[i];if(v>first){second=first;first=v;}else if(v>second)second=v;}
            expect[kind]=second;
        }
    };
    static Costs& costs(){static Costs value;return value;}
    explicit FrameBudget(double seconds=.0005,unsigned maxOps=512):limitSeconds(seconds),maxOperations(maxOps){}
    void enter(){if(depth++==0)started=Clock::now();}
    void leave(){if(--depth==0)usedSeconds+=std::chrono::duration<double>(Clock::now()-started).count();}
    double seconds()const{return usedSeconds+(depth?std::chrono::duration<double>(Clock::now()-started).count():0.);}
    // Bookkeeping that is not one measured operation (loops, retiring packets).
    bool allow()const{return operations<maxOperations&&copiedBytes<maxCopiedBytes&&seconds()<limitSeconds-guardSeconds;}
    // One indivisible operation of this kind may start: it is expected to end inside the budget. The frame's first
    // operation always may (inside the guard), so an operation that alone costs more than the budget still runs - alone.
    bool allow(Kind kind)const{
        if(operations>=maxOperations||copiedBytes>=maxCopiedBytes)return false;
        const double now=seconds();
        if(!measured)return now<limitSeconds-guardSeconds;
        return now+double(costs().expect[kind])*1.25+5e-6<=limitSeconds;
    }
    // Every row copy of one delta, admitted together: a delta is never split over frames. It may start when all of its
    // copies are expected to fit the frame's time and caps.
    bool allowBatch(Kind first,Kind rest,unsigned count,std::size_t bytes)const{
        if(!count)return true;
        const double now=seconds();
        if(operations+count<=maxOperations&&copiedBytes+bytes<=maxCopiedBytes){
            if(!measured){if(now<limitSeconds-guardSeconds)return true;}
            else if(now+(double(costs().expect[first])+double(costs().expect[rest])*double(count-1u))*1.25+5e-6<=limitSeconds)return true;
        }
        // A delta these numbers never admit would never be shown: as the frame's first delta it runs while half the frame's
        // time is still left.
        return !batches&&now<limitSeconds*.5;
    }
    void operation()noexcept{++operations;}
    struct Segment {FrameBudget& budget;explicit Segment(FrameBudget& b):budget(b){budget.enter();}~Segment(){budget.leave();}};
    // Times one operation and feeds its kind's expectation.
    struct Measure {
        FrameBudget& budget;Kind kind;Clock::time_point begin;
        Measure(FrameBudget& b,Kind k):budget(b),kind(k),begin(Clock::now()){}
        ~Measure(){costs().note(kind,std::chrono::duration<float>(Clock::now()-begin).count());++budget.measured;}
    };
};

class Upload {
    std::shared_ptr<const BloodBakeAsync::Delta> packet_;
    std::shared_ptr<Prepared> prepared_;
    std::shared_ptr<Texture> active_;
    unsigned cursor_=0;
    std::uint64_t sequence_=0;
    std::uint64_t copiedBytes_=0,copies_=0,publishes_=0;
    // Each driver command copies at most 128 KiB: 16 adjacent rows of the table.
    static unsigned rowsPerCopy(const Texture& source)noexcept{return (std::max)(1u,131072u/source.rowBytes());}
    unsigned run()const noexcept{
        unsigned count=1;const unsigned limit=rowsPerCopy(*prepared_->rows);const auto& rows=prepared_->targetRows;
        while(count<limit&&cursor_+count<rows.size()&&rows[cursor_+count]==rows[cursor_]+count)++count;
        return count;
    }
    // The driver commands one delta needs: runs of adjacent rows, 16 at most.
    unsigned runs()const noexcept{
        unsigned count=0;const unsigned limit=rowsPerCopy(*prepared_->rows);const auto& rows=prepared_->targetRows;
        for(std::size_t at=0;at<rows.size();++count){std::size_t n=1;while(n<limit&&at+n<rows.size()&&rows[at+n]==rows[at]+n)++n;at+=n;}
        return count;
    }
    void finish()noexcept{sequence_=prepared_->sequence;++publishes_;packet_.reset();prepared_.reset();cursor_=0;}
public:
    bool idle()const noexcept{return !packet_;}
    bool hasActive()const noexcept{return bool(active_);}
    const Texture* active()const noexcept{return active_.get();}
    // The newest generation whose every row is in the live texture.
    std::uint64_t sequence()const noexcept{return sequence_;}
    std::uint64_t copiedBytes()const noexcept{return copiedBytes_;}
    std::uint64_t copies()const noexcept{return copies_;}
    std::uint64_t publishes()const noexcept{return publishes_;}
    // A `cleared` delta: the body's texture is released (the caller rebinds its engine texture first). Only when idle.
    bool clear(){if(!idle())return false;*this=Upload();return true;}
    // False: the packet is not for the table this body shows (the caller asks the worker for a whole snapshot).
    bool begin(std::shared_ptr<const BloodBakeAsync::Delta> packet){
        if(!idle()||!packet||!packet->prepared)return false;
        auto prepared=std::static_pointer_cast<Prepared>(packet->prepared);
        if(!prepared||prepared->cleared||prepared->sequence<sequence_||!prepared->table)return false;
        if(!prepared->full&&(prepared->table!=active_||!prepared->rows))return false;
        packet_=std::move(packet);prepared_=std::move(prepared);cursor_=0;return true;
    }
    // publish(table, sequence) binds a whole new table (it must only adopt the prepared handles). Return false to keep
    // the packet for a later frame. Changed rows go into the live table without a publish call, in one frame.
    template<class Publish> void pump(ID3D11DeviceContext* context,FrameBudget& budget,Publish publish){
        FrameBudget::Segment segment(budget);
        if(!context||!packet_)return;
        if(prepared_->full){
            if(!budget.allow(FrameBudget::Publish))return;
            {FrameBudget::Measure measure(budget,FrameBudget::Publish);if(!publish(*prepared_->table,prepared_->sequence))return;}
            budget.operation();
            // All GPU references used in a submitted command are retained by D3D11; the replaced table is released on
            // the worker with this packet.
            prepared_->retired=std::move(active_);active_=prepared_->table;finish();return;
        }
        // All of the delta's rows now, or none this frame: no draw ever lies between two of its copies, so the shader reads
        // the table before the delta or after it and nothing else.
        const auto& rows=prepared_->targetRows;auto& source=*prepared_->rows;
        if(!budget.allowBatch(FrameBudget::ColdCopy,FrameBudget::Copy,runs(),std::size_t(source.rowBytes())*rows.size()))return;
        ++budget.batches;
        for(cursor_=0;cursor_<rows.size();){
            // A packet's first copy touches a staging texture the driver has not used yet.
            const unsigned n=run();const auto kind=cursor_?FrameBudget::Copy:FrameBudget::ColdCopy;
            D3D11_BOX box{0,cursor_,0,source.width,cursor_+n,1};
            {FrameBudget::Measure measure(budget,kind);
             context->CopySubresourceRegion(active_->texture.p,0,0,rows[cursor_],0,source.texture.p,0,&box);}
            const auto bytes=std::size_t(source.rowBytes())*n;copiedBytes_+=bytes;budget.copiedBytes+=bytes;++copies_;budget.operation();cursor_+=n;
        }
        finish();
    }
};
} // namespace PainterBloodUpload
