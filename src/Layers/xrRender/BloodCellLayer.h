#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace BloodCellLayer {
constexpr std::uint32_t Width=512;                    // texels per row, 16 bytes each: 8 KiB per row
constexpr std::uint32_t Abi=0x424C4434u;              // header texel 1, y: 'BLD4'; painter_blood_cells.h draws no blood for any other value
constexpr std::uint32_t FixedRows=4;                  // row 0 header, rows 1-2 filter, row 3 stamps
constexpr std::uint32_t FilterBase=Width, StampBase=Width*3u, BucketBase=Width*FixedRows; // texel indices
constexpr std::uint32_t MaxStamps=512, BucketSlots=4, BucketsPerRow=Width/2u, StampShift=23;
constexpr float LayerCounts[4]={1.f,2.f,3.f,5.f};    // the 2-bit layer code
constexpr std::uint32_t CellsX=1024, CellsY=2048, CellsZ=2048; // the window, in cells (10 + 11 + 11 key bits)
constexpr std::int32_t CoordLimit=1048576;
struct Vec3 { float x=0,y=0,z=0; };
struct Vec2 { float x=0,y=0; };
struct Coord {
    std::int32_t x=0,y=0,z=0;
    bool operator==(const Coord& r) const noexcept { return x==r.x && y==r.y && z==r.z; }
};
inline std::uint32_t hash(Coord b) noexcept {
    std::uint32_t h=std::uint32_t(b.x)*73856093u ^ std::uint32_t(b.y)*19349663u ^ std::uint32_t(b.z)*83492791u;
    h^=h>>16; h*=0x7feb352du; h^=h>>15; h*=0x846ca68bu; h^=h>>16; return h;
}
struct CoordHash { std::size_t operator()(Coord b) const noexcept { return hash(b); } };
inline float sat(float x) noexcept { return (std::max)(0.f,(std::min)(1.f,x)); }
inline std::uint32_t bits(float v) noexcept { std::uint32_t r;std::memcpy(&r,&v,4);return r; }
inline float value(std::uint32_t v) noexcept { float r;std::memcpy(&r,&v,4);return r; }
struct U4 { std::uint32_t x=0,y=0,z=0,w=0; };
static_assert(sizeof(U4)==16,"GPU texel ABI");
struct TextureSample { float r=0,g=0,b=0,a=0; };
struct Stamp {
    float landed=0, lifetime=60, heat_birth=0;
    // color_intensity retained only for source compatibility; never baked.
    float color_intensity=1, opacity=1; // opacity is per-mark opacity only
    bool heat_enabled=true;
};
inline bool valid(const Stamp& s) noexcept {
    return std::isfinite(s.landed)&&std::isfinite(s.lifetime)&&(s.lifetime>0||s.lifetime==-1.f)&&
        std::isfinite(s.heat_birth)&&std::isfinite(s.color_intensity)&&s.color_intensity>=0&&
        std::isfinite(s.opacity)&&s.opacity>=0;
}
// The texture is sampled at the projected wallmark UV by the caller. It is NOT
// replaced with a circle, noise, darkness-only alpha, or a uniform stain.
inline bool valid(TextureSample s) noexcept {
    return std::isfinite(s.r)&&std::isfinite(s.g)&&std::isfinite(s.b)&&std::isfinite(s.a);
}
// The wallmark shader's coverage of one texel (painter_blood_shading.h: saturate(a * 1.7 - 0.7)).
inline float coverage(TextureSample s) noexcept { return sat(s.a*1.7f-.7f); }
inline float alpha(TextureSample s,const Stamp& p) noexcept { return sat(coverage(s)*p.opacity); }
// The window's low corner and the cell size. A body's rest pose stands on y = 0 around x = z = 0.
struct Frame { Vec3 origin{-1.024f,-.512f,-2.048f}; float pitch=.002f; };
inline bool valid(Frame f) noexcept {
    return std::isfinite(f.origin.x)&&std::isfinite(f.origin.y)&&std::isfinite(f.origin.z)&&std::isfinite(f.pitch)&&f.pitch>0;
}
inline bool coordinate(Frame f,Vec3 p,Coord& out) noexcept {
    // Keep coordinate arithmetic at shader float precision; using a double
    // quotient here selects the neighboring cell at exact float boundaries.
    const double x=std::floor((p.x-f.origin.x)/f.pitch),y=std::floor((p.y-f.origin.y)/f.pitch),z=std::floor((p.z-f.origin.z)/f.pitch);
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||x < -CoordLimit||x>=CoordLimit||y < -CoordLimit||y>=CoordLimit||z < -CoordLimit||z>=CoordLimit)return false;
    out={std::int32_t(x),std::int32_t(y),std::int32_t(z)};return true;
}
struct Pending { TextureSample texel; };
// One Splat spans all projected receiver triangles. Duplicate triangle hits on
// a cell choose the highest coverage once; they never multiply the same mark.
class Splat {
    std::unordered_map<Coord,Pending,CoordHash> pending_;
public:
    Stamp stamp;
    explicit Splat(Stamp s):stamp(s) { if(!valid(s))throw std::invalid_argument("blood_stamp"); }
    bool sampleCell(Coord c,TextureSample t) {
        if(c.x < -CoordLimit||c.x>=CoordLimit||c.y < -CoordLimit||c.y>=CoordLimit||c.z < -CoordLimit||c.z>=CoordLimit||!valid(t)||alpha(t,stamp)<=0)return false;
        auto it=pending_.find(c);
        if(it==pending_.end())pending_.emplace(c,Pending{t});
        else if(alpha(t,stamp)>alpha(it->second.texel,stamp))it->second.texel=t;
        return true;
    }
    std::size_t size()const noexcept{return pending_.size();}
    const std::unordered_map<Coord,Pending,CoordHash>& samples()const noexcept{return pending_;}
};
struct MarkIdentity {
    std::uint64_t nameSpace=0,key=0;
    bool operator==(const MarkIdentity& r)const noexcept{return nameSpace==r.nameSpace&&key==r.key;}
};
struct MarkIdentityHash {std::size_t operator()(MarkIdentity k)const noexcept{
    return std::hash<std::uint64_t>{}(k.nameSpace)^(std::hash<std::uint64_t>{}(k.key)+0x9e3779b97f4a7c15ull+(k.nameSpace<<6)+(k.nameSpace>>2));
}};

// ---- the cell: key and payload ------------------------------------------------------------------------------------------
inline bool keyOf(Coord c,std::uint32_t& key) noexcept {
    if(c.x<0||c.y<0||c.z<0||c.x>=std::int32_t(CellsX)||c.y>=std::int32_t(CellsY)||c.z>=std::int32_t(CellsZ))return false;
    key=std::uint32_t(c.x)|(std::uint32_t(c.y)<<10)|(std::uint32_t(c.z)<<21);return true;
}
inline Coord coordOf(std::uint32_t key) noexcept { return {std::int32_t(key&1023u),std::int32_t((key>>10)&2047u),std::int32_t(key>>21)}; }
// The same mixer as painter_blood_cells.h (painter_blood_hash).
inline std::uint32_t mix(std::uint32_t k) noexcept { k^=k>>16; k*=0x7feb352du; k^=k>>15; k*=0x846ca68bu; k^=k>>16; return k; }
// cover: the coverage of ONE layer (before the live opacity control and the fade); layers: the 2-bit code of LayerCounts.
struct Blended { float r=0,g=0,b=0,cover=0; std::uint32_t layers=0,stamp=0; };
// R5 G5 B5, coverage 6 bits (1..63: a stored cell is never 0), layer code 2 bits, stamp 9 bits. Zero payload = empty slot.
inline std::uint32_t pack(const Blended& v) noexcept {
    const auto q=[](float x,float n){return std::uint32_t(sat(x)*n+.5f);};
    const std::uint32_t cover=(std::max)(1u,q(v.cover,63.f));
    return q(v.r,31.f)|(q(v.g,31.f)<<5)|(q(v.b,31.f)<<10)|(cover<<15)|((v.layers&3u)<<21)|((v.stamp&511u)<<StampShift);
}
inline Blended unpack(std::uint32_t p) noexcept {
    return {float(p&31u)/31.f,float((p>>5)&31u)/31.f,float((p>>10)&31u)/31.f,float((p>>15)&63u)/63.f,(p>>21)&3u,p>>StampShift};
}
// The shader's fade of one stain at a time (painter_blood_cells.h): the native wallmark's inverse fade in 8 bits.
inline float fadeAt(const Stamp& s,float now) noexcept {
    const float age=(std::max)(now-s.landed,0.f);
    const float elapsed=s.lifetime<0?0.f:(s.lifetime>0?sat(age/s.lifetime):1.f);
    return 1.f-std::floor(sat(1.f-s.opacity*(1.f-elapsed))*255.f)*(1.f/255.f);
}
inline bool expiredAt(const Stamp& s,float now) noexcept { return s.lifetime>0&&now-s.landed>=s.lifetime; }

class Layer {
public:
    struct Applied { std::size_t changed=0, outside=0, evicted=0; };
    static constexpr std::size_t DefaultMaxBytes=8u*1024u*1024u;
    static constexpr std::size_t MaxFill=85; // per cent of the slots
private:
    struct StampSlot { Stamp stamp; std::uint32_t refs=0; bool used=false, keyed=false; };
    struct KeyedMark { std::uint32_t stamp=0; std::uint64_t revision=0; };
    Frame frame_;
    std::vector<U4> image_;           // the texture's content, FixedRows + bucket rows; empty until the first cell
    std::vector<std::uint8_t> dirty_; // per row
    std::vector<StampSlot> stamps_;   // MaxStamps once the image exists
    std::unordered_map<MarkIdentity,KeyedMark,MarkIdentityHash> keyed_;
    std::uint32_t buckets_=0, random_=0x9e3779b9u;
    std::uint32_t baking_=MaxStamps;  // the stamp of the stain being baked: it keeps its row while it has no cell yet
    std::size_t cells_=0, evicted_=0;
    bool rebuilt_=false;              // the image was built anew (first cell, other size, cells removed): publish it whole

    static std::uint32_t rowsFor(std::uint32_t buckets) noexcept { return FixedRows+(buckets+BucketsPerRow-1u)/BucketsPerRow; }
    std::uint32_t* slot(std::uint32_t bucket,unsigned i) noexcept { auto& t=image_[BucketBase+2u*bucket+(i>>1)];return (i&1u)?&t.z:&t.x; }
    const std::uint32_t* slot(std::uint32_t bucket,unsigned i)const noexcept { const auto& t=image_[BucketBase+2u*bucket+(i>>1)];return (i&1u)?&t.z:&t.x; }
    void touchBucket(std::uint32_t bucket) noexcept { dirty_[(BucketBase+2u*bucket)/Width]=1; }
    void buckets(std::uint32_t key,std::uint32_t& a,std::uint32_t& b)const noexcept {
        a=mix(key)%buckets_;b=buckets_>1u?(a+1u+mix(key^0x9e3779b9u)%(buckets_-1u))%buckets_:a;
    }
    std::uint32_t* find(std::uint32_t key) noexcept {
        std::uint32_t a,b;buckets(key,a,b);
        for(unsigned i=0;i<BucketSlots;++i){auto* s=slot(a,i);if(s[1]&&s[0]==key)return s;}
        for(unsigned i=0;i<BucketSlots;++i){auto* s=slot(b,i);if(s[1]&&s[0]==key)return s;}
        return nullptr;
    }
    void setFilter(std::uint32_t key) noexcept {
        const auto c=coordOf(key);const std::uint32_t block=(std::uint32_t(c.x)>>5)+32u*((std::uint32_t(c.y)>>5)+64u*(std::uint32_t(c.z)>>5));
        auto& texel=image_[FilterBase+(block>>7)];std::uint32_t* word=&texel.x+((block>>5)&3u);const std::uint32_t bit=1u<<(block&31u);
        if(!(*word&bit)){*word|=bit;dirty_[(FilterBase+(block>>7))/Width]=1;}
    }
    // A key that is not in the table. False: no free slot after the kicks (`key`/`payload` then hold the entry left over).
    bool place(std::uint32_t& key,std::uint32_t& payload) noexcept {
        for(unsigned kick=0;kick<96;++kick){
            std::uint32_t a,b;buckets(key,a,b);
            for(const auto bucket:{a,b})for(unsigned i=0;i<BucketSlots;++i){auto* s=slot(bucket,i);if(!s[1]){s[0]=key;s[1]=payload;touchBucket(bucket);return true;}}
            random_^=random_<<13;random_^=random_>>17;random_^=random_<<5;
            const std::uint32_t bucket=(random_&4u)?a:b;auto* s=slot(bucket,random_&3u);
            std::swap(key,s[0]);std::swap(payload,s[1]);touchBucket(bucket);
        }
        return false;
    }
    void writeHeader() noexcept {
        image_[0]={bits(frame_.origin.x),bits(frame_.origin.y),bits(frame_.origin.z),bits(frame_.pitch)};
        image_[1]={buckets_,Abi,MaxStamps,0};dirty_[0]=1;
    }
    void writeStamp(std::uint32_t id) noexcept {
        const auto& s=stamps_[id].stamp;
        // heat birth below zero = this stain carries no heat
        image_[StampBase+id]={bits(s.landed),bits(s.lifetime),bits(s.heat_enabled?(std::max)(s.heat_birth,0.f):-1.f),bits(s.opacity)};
        dirty_[(StampBase+id)/Width]=1;
    }
    // A new image with this many buckets holding every cell `keep` accepts (and the entry left over from a failed place).
    template<class Keep> void rebuild(std::uint32_t buckets,Keep keep,std::uint32_t extraKey=0,std::uint32_t extraPayload=0) {
        for(;;){
            std::vector<U4> next(std::size_t(rowsFor(buckets))*Width);std::vector<std::uint8_t> dirty(rowsFor(buckets),1);
            std::vector<StampSlot> stamps=stamps_;if(stamps.empty())stamps.resize(MaxStamps);
            for(auto& s:stamps)s.refs=0;
            Layer scratch(frame_);scratch.image_.swap(next);scratch.dirty_.swap(dirty);scratch.buckets_=buckets;scratch.random_=random_;
            bool fits=true;std::size_t count=0;
            auto add=[&](std::uint32_t key,std::uint32_t payload){
                if(!fits||!payload||!keep(key,payload))return;
                scratch.setFilter(key);std::uint32_t k=key,p=payload;
                if(!scratch.place(k,p)){fits=false;return;}
                ++stamps[payload>>StampShift].refs;++count;};
            for(std::uint32_t b=0;b<buckets_;++b)for(unsigned i=0;i<BucketSlots;++i){const auto* s=slot(b,i);add(s[0],s[1]);}
            add(extraKey,extraPayload);
            if(!fits){buckets+=buckets/8u+1u;continue;} // a kick chain failed in the new size: a little larger
            image_.swap(scratch.image_);dirty_.swap(scratch.dirty_);buckets_=buckets;cells_=count;stamps_.swap(stamps);random_=scratch.random_;
            writeHeader();for(std::uint32_t id=0;id<MaxStamps;++id){if(stamps_[id].used&&!stamps_[id].refs&&!stamps_[id].keyed&&id!=baking_)stamps_[id].used=false;if(stamps_[id].used)writeStamp(id);}
            rebuilt_=true;return;
        }
    }
    static std::uint32_t bucketsFor(std::size_t cells) noexcept {
        // 66 % full after a resize; at least one row of buckets
        const std::size_t want=(cells*100u/66u+BucketSlots-1u)/BucketSlots;
        return std::uint32_t((std::max)(std::size_t(BucketsPerRow),((want+BucketsPerRow-1u)/BucketsPerRow)*BucketsPerRow));
    }
    static std::uint32_t maxBuckets(std::size_t maxBytes) noexcept {
        const std::size_t rows=maxBytes/(std::size_t(Width)*sizeof(U4));
        return std::uint32_t(rows>FixedRows?(rows-FixedRows)*BucketsPerRow:BucketsPerRow);
    }
    std::size_t capacity()const noexcept{return std::size_t(buckets_)*BucketSlots;}
    // Cells of stains the predicate names leave the table. Returns how many.
    template<class Drop> std::size_t dropStamps(Drop drop) {
        const auto before=cells_;
        rebuild(buckets_,[&](std::uint32_t,std::uint32_t payload){return !drop(payload>>StampShift);});
        evicted_+=before-cells_;return before-cells_;
    }
    // Room for `more` new cells within `maxBytes`: expired stains leave, the table grows by a third, and at the cap the
    // oldest stains' cells go.
    std::size_t reserveCells(std::size_t more,std::size_t maxBytes,float now,std::uint32_t protect=MaxStamps) {
        std::size_t evicted=0;
        if(image_.empty()){buckets_=0;stamps_.assign(MaxStamps,{});rebuild((std::min)(bucketsFor(more),maxBuckets(maxBytes)),[](std::uint32_t,std::uint32_t){return true;});}
        const std::uint32_t limit=(std::max)(maxBuckets(maxBytes),std::uint32_t(BucketsPerRow));
        auto full=[&](std::size_t cells,std::uint32_t buckets){return cells*100u>std::size_t(buckets)*BucketSlots*MaxFill;};
        if(full(cells_+more,buckets_)){
            // stains past their lifetime draw nothing: they leave before the table grows
            bool expired=false;for(std::uint32_t id=0;id<MaxStamps;++id)if(stamps_[id].used&&stamps_[id].refs&&!stamps_[id].keyed&&expiredAt(stamps_[id].stamp,now)){expired=true;break;}
            if(expired)evicted+=dropStamps([&](std::uint32_t id){return !stamps_[id].keyed&&expiredAt(stamps_[id].stamp,now);});
        }
        if(full(cells_+more,buckets_)){
            const std::uint32_t want=(std::min)(limit,(std::max)(bucketsFor(cells_+more),buckets_+buckets_/3u));
            if(want>buckets_)rebuild(want,[](std::uint32_t,std::uint32_t){return true;});
        }
        // At the body's cap: the oldest stains make room, until the new cells fit at 70 %.
        while(full(cells_+more,buckets_)&&cells_){
            std::vector<std::uint32_t> order;for(std::uint32_t id=0;id<MaxStamps;++id)if(id!=protect&&stamps_[id].used&&stamps_[id].refs)order.push_back(id);
            if(order.empty())break;
            std::sort(order.begin(),order.end(),[&](std::uint32_t a,std::uint32_t b){return stamps_[a].stamp.landed<stamps_[b].stamp.landed;});
            const std::size_t room=capacity()*70u/100u;const std::size_t need=cells_+more>room?cells_+more-room:1u;
            std::vector<std::uint8_t> gone(MaxStamps,0);std::size_t freed=0;
            for(const auto id:order){gone[id]=1;freed+=stamps_[id].refs;if(freed>=need)break;}
            evicted+=dropStamps([&](std::uint32_t id){return gone[id]!=0;});
        }
        return evicted;
    }
    std::uint32_t newStamp(const Stamp& stamp,bool keyed,float now) {
        for(int pass=0;pass<3;++pass){
            for(std::uint32_t id=0;id<MaxStamps;++id)if(!stamps_[id].used){stamps_[id]={stamp,0,true,keyed};writeStamp(id);return id;}
            // every id is in use: expired stains leave, then the oldest stain's cells take the next oldest stain's clock
            if(pass==0){
                // cut marks whose cells are all gone give their id back
                for(auto it=keyed_.begin();it!=keyed_.end();){auto& slot=stamps_[it->second.stamp];
                    if(slot.keyed&&!slot.refs){slot.used=slot.keyed=false;it=keyed_.erase(it);}else ++it;}
                bool expired=false;for(std::uint32_t id=0;id<MaxStamps;++id)if(stamps_[id].used&&!stamps_[id].keyed&&expiredAt(stamps_[id].stamp,now)){expired=true;break;}
                if(expired)dropStamps([&](std::uint32_t id){return !stamps_[id].keyed&&expiredAt(stamps_[id].stamp,now);});
            }else{
                std::uint32_t oldest=MaxStamps,second=MaxStamps;
                for(std::uint32_t id=0;id<MaxStamps;++id){if(stamps_[id].keyed)continue;
                    if(oldest==MaxStamps||stamps_[id].stamp.landed<stamps_[oldest].stamp.landed){second=oldest;oldest=id;}
                    else if(second==MaxStamps||stamps_[id].stamp.landed<stamps_[second].stamp.landed)second=id;}
                if(oldest==MaxStamps||second==MaxStamps)break;
                for(std::uint32_t b=0;b<buckets_;++b)for(unsigned i=0;i<BucketSlots;++i){auto* s=slot(b,i);
                    if(s[1]&&(s[1]>>StampShift)==oldest){s[1]=(s[1]&((1u<<StampShift)-1u))|(second<<StampShift);touchBucket(b);--stamps_[oldest].refs;++stamps_[second].refs;}}
                stamps_[oldest].used=false;
            }
        }
        throw std::length_error("blood_stamp_table_full");
    }
    void release(std::uint32_t id) noexcept { if(stamps_[id].refs)--stamps_[id].refs;if(!stamps_[id].refs&&!stamps_[id].keyed)stamps_[id].used=false; }
    // The stain's cells go into the table. replaceOwn: a growing cut's cells that already carry its stamp are replaced, not
    // blended (a revision never stacks on itself).
    Applied bake(const Splat& s,std::uint32_t id,std::size_t maxBytes,bool replaceOwn) {
        Applied result;const auto& stamp=s.stamp;const float now=stamp.landed;
        const float fadeNew=(std::max)(fadeAt(stamp,now),1.f/255.f);
        for(const auto& sample:s.samples()){
            std::uint32_t key;if(!keyOf(sample.first,key)){++result.outside;continue;}
            const auto& t=sample.second.texel;const float coverNew=coverage(t);if(coverNew<=0)continue;
            const bool faint=coverNew*63.f<.5f; // rounds to no coverage: never a new cell, still blended into an existing one
            if(auto* found=find(key)){
                const auto old=unpack(found[1]);Blended next{sat(t.r),sat(t.g),sat(t.b),coverNew,0,id};
                if(!(replaceOwn&&old.stamp==id)){
                    // The new stain over the older blood, at the new stain's landing. The older blood is `layers` layers of
                    // its coverage at the strength its own fade has left; with the new layer the cell lets through
                    // `through` of what is under it (opacity control 1) and its layers' coverages sum to `sum`. Kept: the
                    // layer count of the four that, at the same `through`, comes nearest to that sum - so the live opacity
                    // control acts on the cell as it did on the separate stains - and the colour of the composite.
                    const float eOld=old.cover*fadeAt(stamps_[old.stamp].stamp,now),eNew=coverNew*fadeNew,countOld=LayerCounts[old.layers];
                    const float throughOld=std::pow(1.f-eOld,countOld),through=throughOld*(1.f-eNew),sum=countOld*eOld+eNew;
                    float best=0,bestError=1e9f;
                    for(std::uint32_t code=0;code<4;++code){const float one=1.f-std::pow(through,1.f/LayerCounts[code]),error=std::fabs(LayerCounts[code]*one-sum);
                        if(error<bestError){bestError=error;best=one;next.layers=code;}}
                    next.cover=sat(best/fadeNew);
                    const float covered=1.f-through;
                    if(covered>0){const float wNew=eNew/covered,wOld=1.f-wNew;
                        next.r=sat(t.r)*wNew+old.r*wOld;next.g=sat(t.g)*wNew+old.g*wOld;next.b=sat(t.b)*wNew+old.b*wOld;}
                }
                const auto packed=pack(next);
                if(packed!=found[1]){
                    if(old.stamp!=id){release(old.stamp);++stamps_[id].refs;}
                    found[1]=packed;std::uint32_t a,b;buckets(key,a,b);
                    touchBucket(found>=slot(a,0)&&found<=slot(a,BucketSlots-1)?a:b);++result.changed;
                }
                continue;
            }
            if(faint)continue;
            if(cells_*100u>=capacity()*MaxFill){ // the estimate was short: make room for the rest
                result.evicted+=reserveCells(s.size(),maxBytes,now,id);
                if(cells_*100u>=capacity()*MaxFill)continue; // at the cap with nothing older to drop
            }
            std::uint32_t k=key,p=pack({sat(t.r),sat(t.g),sat(t.b),coverNew,0,id});
            setFilter(key);++cells_;++stamps_[id].refs;++result.changed; // counted first: a rebuild recounts from the table
            if(!place(k,p)){
                const std::uint32_t limit=(std::max)(maxBuckets(maxBytes),std::uint32_t(BucketsPerRow));
                if(buckets_<limit)rebuild((std::min)(limit,buckets_+buckets_/8u+BucketsPerRow),[](std::uint32_t,std::uint32_t){return true;},k,p);
                else{ // at the cap: the entry left over (one cell, of any stain) goes
                    release(p>>StampShift);--cells_;++evicted_;++result.evicted;
                }
            }
        }
        return result;
    }
public:
    explicit Layer(Frame f={}):frame_(f) { if(!valid(f))throw std::invalid_argument("blood_frame"); }
    const Frame& frame()const noexcept{return frame_;}
    // The texture: bytes() = rows() * Width * 16. Empty until the first cell.
    std::size_t bytes()const noexcept{return image_.size()*sizeof(U4);}
    std::uint32_t rows()const noexcept{return std::uint32_t(image_.size()/Width);}
    const std::vector<U4>& image()const noexcept{return image_;}
    std::size_t cellCount()const noexcept{return cells_;}
    std::size_t evictedCells()const noexcept{return evicted_;}
    std::uint32_t bucketCount()const noexcept{return buckets_;}
    unsigned stampCount()const noexcept{unsigned n=0;for(const auto& s:stamps_)n+=s.used?1u:0u;return n;}
    // The whole image must be published (first cell, a resize, removed cells); else only dirtyRows().
    bool rebuilt()const noexcept{return rebuilt_;}
    std::vector<std::uint32_t> dirtyRows()const {std::vector<std::uint32_t> out;for(std::uint32_t r=0;r<dirty_.size();++r)if(dirty_[r])out.push_back(r);return out;}
    bool dirty()const noexcept{return rebuilt_||std::find(dirty_.begin(),dirty_.end(),std::uint8_t(1))!=dirty_.end();}
    // Clear only after a successful publication.
    void acknowledge()noexcept {std::fill(dirty_.begin(),dirty_.end(),std::uint8_t(0));rebuilt_=false;}
    // How many of the stain's cells are new to the table, and how many lie outside the window.
    std::size_t newCells(const Splat& s)const {
        std::size_t count=0;if(image_.empty()){for(const auto& c:s.samples()){std::uint32_t key;if(keyOf(c.first,key))++count;}return count;}
        auto* self=const_cast<Layer*>(this);for(const auto& c:s.samples()){std::uint32_t key;if(keyOf(c.first,key)&&!self->find(key))++count;}
        return count;
    }
    static std::size_t outsideCells(const Splat& s) noexcept {std::size_t count=0;for(const auto& c:s.samples()){std::uint32_t key;if(!keyOf(c.first,key))++count;}return count;}
    // The size the texture would have with this many cells (the growth the caller reserves before apply).
    static std::size_t bytesFor(std::size_t cells,std::size_t maxBytes=DefaultMaxBytes) noexcept {
        if(!cells)return 0;
        return std::size_t(rowsFor((std::min)(bucketsFor(cells),(std::max)(maxBuckets(maxBytes),std::uint32_t(BucketsPerRow)))))*Width*sizeof(U4);
    }
    std::size_t growthFor(std::size_t more,std::size_t maxBytes)const noexcept {
        if(!more)return 0;
        if(!image_.empty()&&(cells_+more)*100u<=capacity()*MaxFill)return 0;
        std::uint32_t want=bucketsFor(cells_+more);if(!image_.empty())want=(std::max)(want,buckets_+buckets_/3u);
        want=(std::min)(want,(std::max)(maxBuckets(maxBytes),std::uint32_t(BucketsPerRow)));
        const std::size_t next=std::size_t(rowsFor(want))*Width*sizeof(U4);return next>bytes()?next-bytes():0;
    }
    // An ordinary stain: every cell it covers takes the blend and this stain's clock. The texture never passes maxBytes.
    Applied apply(const Splat& s,std::size_t maxBytes=DefaultMaxBytes) {
        if(!valid(s.stamp))throw std::invalid_argument("blood_stamp");
        const std::size_t more=newCells(s);Applied result;
        if(!more&&image_.empty()){result.outside=outsideCells(s);return result;}
        result.evicted=reserveCells(more,maxBytes,s.stamp.landed);
        const auto id=newStamp(s.stamp,false,s.stamp.landed);baking_=id;
        Applied baked;try{baked=bake(s,id,maxBytes,false);}catch(...){baking_=MaxStamps;if(!stamps_[id].refs)stamps_[id].used=false;throw;}
        baking_=MaxStamps;result.changed=baked.changed;result.outside=baked.outside;result.evicted+=baked.evicted;
        if(!stamps_[id].refs)stamps_[id].used=false;
        // a table that emptied by more than three quarters gives its rows back
        if(buckets_>BucketsPerRow&&cells_*5u<capacity()&&bucketsFor(cells_)<buckets_)rebuild(bucketsFor(cells_),[](std::uint32_t,std::uint32_t){return true;});
        return result;
    }
    bool keyedStale(std::uint64_t nameSpace,std::uint64_t key,std::uint64_t revision)const noexcept {
        const auto found=keyed_.find({nameSpace,key});return found!=keyed_.end()&&revision<=found->second.revision;
    }
    // A growing cut (one mark per namespace and key): its cells keep one stamp for all revisions; a newer revision replaces
    // the cells it covers again and adds the new ones. preserveOldCoverage (the native blade policy): cells of earlier
    // revisions outside the new footprint stay; without it they leave.
    Applied applyKeyed(const Splat& s,std::uint64_t nameSpace,std::uint64_t key,std::uint64_t revision,bool preserveOldCoverage=false,std::size_t maxBytes=DefaultMaxBytes) {
        if(!nameSpace||!key||!revision||!valid(s.stamp))throw std::invalid_argument("blood_keyed_stamp");
        Applied result;const MarkIdentity identity{nameSpace,key};auto prior=keyed_.find(identity);
        if(prior!=keyed_.end()&&revision<=prior->second.revision)return result;
        const std::size_t more=newCells(s);
        if(!more&&image_.empty()){result.outside=outsideCells(s);return result;}
        result.evicted=reserveCells(more,maxBytes,s.stamp.landed);
        std::uint32_t id;
        if(prior!=keyed_.end()&&stamps_[prior->second.stamp].used&&stamps_[prior->second.stamp].keyed){id=prior->second.stamp;stamps_[id].stamp=s.stamp;writeStamp(id);}
        else{id=newStamp(s.stamp,true,s.stamp.landed);if(prior==keyed_.end())prior=keyed_.emplace(identity,KeyedMark{}).first;prior->second.stamp=id;}
        prior->second.revision=revision;
        if(!preserveOldCoverage&&stamps_[id].refs){
            const auto before=cells_;
            rebuild(buckets_,[&](std::uint32_t cellKey,std::uint32_t payload){return (payload>>StampShift)!=id||s.samples().find(coordOf(cellKey))!=s.samples().end();});
            result.changed+=before-cells_;
        }
        const auto baked=bake(s,id,maxBytes,true);result.changed+=baked.changed;result.outside=baked.outside;result.evicted+=baked.evicted;
        return result;
    }
    // The shader's lookup (painter_blood_read), for tests and replays: the cell's blended value, if it holds blood.
    bool read(Coord c,Blended& out,Stamp* stamp=nullptr)const {
        std::uint32_t key;if(image_.empty()||!buckets_||!keyOf(c,key))return false;
        const std::uint32_t block=(std::uint32_t(c.x)>>5)+32u*((std::uint32_t(c.y)>>5)+64u*(std::uint32_t(c.z)>>5));
        const auto& texel=image_[FilterBase+(block>>7)];const std::uint32_t word=(&texel.x)[(block>>5)&3u];
        if(!((word>>(block&31u))&1u))return false;
        const auto* found=const_cast<Layer*>(this)->find(key);if(!found)return false;
        out=unpack(found[1]);
        if(stamp){const auto& row=image_[StampBase+out.stamp];stamp->landed=value(row.x);stamp->lifetime=value(row.y);
            stamp->heat_birth=value(row.z);stamp->heat_enabled=value(row.z)>=0;stamp->opacity=value(row.w);}
        return true;
    }
};

struct Vertex { Vec3 rest;Vec2 uv; };
inline float component(Vec3 p,int n)noexcept{return n==0?p.x:(n==1?p.y:p.z);}
inline void setComponent(Coord& c,int n,std::int32_t v)noexcept{if(n==0)c.x=v;else if(n==1)c.y=v;else c.z=v;}
inline double cross2(double ax,double ay,double bx,double by)noexcept{return ax*by-ay*bx;}
// Clip the actual receiver triangle against every intersected cell. Dominant
// projected columns bound work, but all depth cells reached by the triangle
// are visited (an oblique surface is not reduced to its center-depth sample).
struct ClipVertex { double q[3]{};double u=0,v=0; };
struct Polygon { std::array<ClipVertex,16> points;unsigned count=0; };
inline ClipVertex mix(ClipVertex a,ClipVertex b,double t)noexcept {
    ClipVertex r;for(int i=0;i<3;++i)r.q[i]=a.q[i]+(b.q[i]-a.q[i])*t;
    r.u=a.u+(b.u-a.u)*t;r.v=a.v+(b.v-a.v)*t;return r;
}
inline Polygon clipPlane(const Polygon& in,int axis,double bound,bool above) {
    Polygon out;if(!in.count)return out;
    auto inside=[&](const ClipVertex& p){return above?p.q[axis]>=bound:p.q[axis]<=bound;};
    auto add=[&](ClipVertex p){if(out.count>=out.points.size())throw std::length_error("blood_polygon_bounds");out.points[out.count++]=p;};
    ClipVertex a=in.points[in.count-1];bool ai=inside(a);
    for(unsigned i=0;i<in.count;++i){const auto b=in.points[i];const bool bi=inside(b);
        if(ai!=bi){const double den=b.q[axis]-a.q[axis];if(den!=0){auto p=mix(a,b,(bound-a.q[axis])/den);p.q[axis]=bound;add(p);}}
        if(bi)add(b);a=b;ai=bi;
    }return out;
}
inline Polygon clipUv(const Polygon& in,bool v_axis,double bound,bool above) {
    Polygon out;if(!in.count)return out;
    auto coordinate=[&](const ClipVertex& p){return v_axis?p.v:p.u;};
    auto inside=[&](const ClipVertex& p){return above?coordinate(p)>=bound:coordinate(p)<=bound;};
    auto add=[&](ClipVertex p){if(out.count>=out.points.size())throw std::length_error("blood_uv_polygon_bounds");out.points[out.count++]=p;};
    auto a=in.points[in.count-1];bool ai=inside(a);
    for(unsigned i=0;i<in.count;++i){const auto b=in.points[i];const bool bi=inside(b);
        if(ai!=bi){const double den=coordinate(b)-coordinate(a);if(den!=0){auto p=mix(a,b,(bound-coordinate(a))/den);if(v_axis)p.v=bound;else p.u=bound;add(p);}}
        if(bi)add(b);a=b;ai=bi;
    }return out;
}
inline double distance2(const ClipVertex& p,const double c[3])noexcept {
    double r=0;for(int i=0;i<3;++i){const double d=p.q[i]-c[i];r+=d*d;}return r;
}
inline ClipVertex closestOnPolygon(const Polygon& p,const double c[3]) {
    ClipVertex best=p.points[0];double best_d=distance2(best,c);
    // Closest point on every edge, including degenerate clipped faces.
    for(unsigned i=0;i<p.count;++i){const auto& a=p.points[i];const auto& b=p.points[(i+1)%p.count];double dot=0,den=0;
        for(int k=0;k<3;++k){const double d=b.q[k]-a.q[k];dot+=(c[k]-a.q[k])*d;den+=d*d;}
        const auto q=mix(a,b,den>0?(std::max)(0.,(std::min)(1.,dot/den)):0.);const double dist=distance2(q,c);
        if(dist<best_d){best=q;best_d=dist;}
    }
    // A clipped triangle is convex. A projection inside any triangle of its
    // fan is the nearest surface point, with the same affine decal UV.
    for(unsigned i=1;i+1<p.count;++i){const auto&a=p.points[0];const auto&b=p.points[i];const auto&d=p.points[i+1];
        double ab[3],ad[3],ac[3],aa=0,bb=0,cc=0,xx=0,yy=0;
        for(int k=0;k<3;++k){ab[k]=b.q[k]-a.q[k];ad[k]=d.q[k]-a.q[k];ac[k]=c[k]-a.q[k];aa+=ab[k]*ab[k];bb+=ab[k]*ad[k];cc+=ad[k]*ad[k];xx+=ac[k]*ab[k];yy+=ac[k]*ad[k];}
        const double den=aa*cc-bb*bb;if(den<=1e-20)continue;
        const double v=(xx*cc-yy*bb)/den,w=(yy*aa-xx*bb)/den;if(v<0||w<0||v+w>1)continue;
        ClipVertex q;for(int k=0;k<3;++k)q.q[k]=a.q[k]+v*ab[k]+w*ad[k];q.u=a.u+v*(b.u-a.u)+w*(d.u-a.u);q.v=a.v+v*(b.v-a.v)+w*(d.v-a.v);
        const double dist=distance2(q,c);if(dist<best_d){best=q;best_d=dist;}
    }return best;
}
struct RasterStats {std::uint64_t columns=0,voxels=0,clipped_vertices=0;};
template<class Sampler,class Admitter>
std::size_t rasterizeTriangle(Splat& splat,Frame f,const std::array<Vertex,3>& v,Sampler sample,Admitter admit,RasterStats* stats=nullptr) {
    if(!valid(f))throw std::invalid_argument("blood_frame");Polygon triangle;triangle.count=3;
    for(unsigned i=0;i<3;++i){const auto& q=v[i];if(!std::isfinite(q.rest.x)||!std::isfinite(q.rest.y)||!std::isfinite(q.rest.z)||!std::isfinite(q.uv.x)||!std::isfinite(q.uv.y))return 0;
        for(int k=0;k<3;++k){const double t=(double(component(q.rest,k))-component(f.origin,k))/f.pitch;if(t < -CoordLimit||t>=CoordLimit)return 0;triangle.points[i].q[k]=t;}
        triangle.points[i].u=q.uv.x;triangle.points[i].v=q.uv.y;
    }
    double a[3],b[3],n[3];for(int k=0;k<3;++k){a[k]=triangle.points[1].q[k]-triangle.points[0].q[k];b[k]=triangle.points[2].q[k]-triangle.points[0].q[k];}
    n[0]=a[1]*b[2]-a[2]*b[1];n[1]=a[2]*b[0]-a[0]*b[2];n[2]=a[0]*b[1]-a[1]*b[0];
    int d=0;if(std::fabs(n[1])>std::fabs(n[d]))d=1;if(std::fabs(n[2])>std::fabs(n[d]))d=2;if(std::fabs(n[d])<1e-12)return 0;
    const int u=(d+1)%3,w=(d+2)%3;
    // Cull the real projected decal footprint before its cell bounds. A large
    // source face touched by a tiny mark must not rasterize its whole body AABB.
    triangle=clipUv(triangle,false,0,true);triangle=clipUv(triangle,false,1,false);
    triangle=clipUv(triangle,true,0,true);triangle=clipUv(triangle,true,1,false);
    if(!triangle.count)return 0;
    if(stats)stats->clipped_vertices+=triangle.count;
    const auto extent=[&](int k,bool high){double r=triangle.points[0].q[k];for(unsigned i=1;i<triangle.count;++i)r=high?(std::max)(r,triangle.points[i].q[k]):(std::min)(r,triangle.points[i].q[k]);return std::int32_t(std::floor(r));};
    const auto x0=extent(u,false),x1=extent(u,true),y0=extent(w,false),y1=extent(w,true);
    if(std::uint64_t(x1-x0+1)*std::uint64_t(y1-y0+1)>16777216u)throw std::length_error("blood_receiver_triangle_bounds");
    if(stats)stats->columns+=std::uint64_t(x1-x0+1)*std::uint64_t(y1-y0+1);
    std::size_t written=0;
    for(std::int32_t iy=y0;iy<=y1;++iy)for(std::int32_t ix=x0;ix<=x1;++ix){
        auto column=clipPlane(triangle,u,ix,true);column=clipPlane(column,u,double(ix)+1,false);column=clipPlane(column,w,iy,true);column=clipPlane(column,w,double(iy)+1,false);if(!column.count)continue;
        double low=column.points[0].q[d],high=low;for(unsigned i=1;i<column.count;++i){low=(std::min)(low,column.points[i].q[d]);high=(std::max)(high,column.points[i].q[d]);}
        for(auto iz=std::int32_t(std::floor(low));iz<=std::int32_t(std::floor(high));++iz){
            if(stats)++stats->voxels;
            auto piece=clipPlane(column,d,iz,true);piece=clipPlane(piece,d,double(iz)+1,false);if(!piece.count)continue;
            Coord cell;setComponent(cell,u,ix);setComponent(cell,w,iy);setComponent(cell,d,iz);
            const double center[3]={cell.x+.5,cell.y+.5,cell.z+.5};const auto hit=closestOnPolygon(piece,center);
            if(hit.u < -1e-6||hit.u>1.000001||hit.v < -1e-6||hit.v>1.000001)continue;
            const Vec3 point{float(f.origin.x+hit.q[0]*f.pitch),float(f.origin.y+hit.q[1]*f.pitch),float(f.origin.z+hit.q[2]*f.pitch)};
            if(admit(cell,point)&&splat.sampleCell(cell,sample(sat(float(hit.u)),sat(float(hit.v)))))++written;
        }
    }return written;
}
} // namespace BloodCellLayer
