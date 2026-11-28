#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>
#include "PainterBurnRecession.h"
#include "PainterFireSpread.h"

// Worker-owned. Published snapshots contain values and lifetime serials only.
namespace painter_burn {
using U32=std::uint32_t;
using U64=std::uint64_t;
struct OwnerKey {
    U64 spawn{},generation{};
    bool operator==(OwnerKey q) const { return spawn==q.spawn&&generation==q.generation; }
};
enum Domain:std::uint8_t { Exterior=1, Anatomy=2, Backing=3 };
struct ContactCell { std::int32_t x{},y{},z{};std::uint16_t dose{};std::uint8_t heat{},domain{};U32 surface{},witness{}; };
constexpr U32 MaxSurface=524286,SingleCell=0x80000000u;
inline U32 makeWitness(double x,double y,double z,double nx,double ny,double nz) {
    auto sub=[](double r){const double cell=r*125.;return U32(std::min(63.,std::floor((cell-std::floor(cell))*64.)));};
    const double norm=std::abs(nx)+std::abs(ny)+std::abs(nz);
    nx/=norm;ny/=norm;nz/=norm;
    if(nz<0.){const double a=nx;nx=(1.-std::abs(ny))*(a<0.?-1.:1.);ny=(1.-std::abs(a))*(ny<0.?-1.:1.);}
    auto oct=[](double a){return U32(std::lround(std::clamp(a*.5+.5,0.,1.)*127.));};
    return sub(x)|(sub(y)<<6)|(sub(z)<<12)|(oct(nx)<<18)|(oct(ny)<<25);
}
inline bool makeContact(double x,double y,double z,double nx,double ny,double nz,U32 surface,unsigned domain,
                        std::uint16_t dose,std::uint8_t heat,ContactCell& out) {
    if(!surface||surface>MaxSurface||domain<Exterior||domain>Backing)return false;
    for(double r:{x,y,z})if(!std::isfinite(r)||std::abs(r)>1000000.)return false;
    const double len=std::sqrt(nx*nx+ny*ny+nz*nz);if(!std::isfinite(len)||len<1.e-8)return false;
    out={int(std::floor(x*125.)),int(std::floor(y*125.)),int(std::floor(z*125.)),dose,heat,std::uint8_t(domain),
        surface,makeWitness(x,y,z,nx/len,ny/len,nz/len)};return true;
}
inline bool matchesWitness(U32 w,double x,double y,double z,double nx,double ny,double nz) {
    auto sub=[](double a,U32 q){return (a*125.-std::floor(a*125.)-(q+.5)/64.)*.008;};
    const double dx=sub(x,w&63),dy=sub(y,(w>>6)&63),dz=sub(z,(w>>12)&63);
    double ax=((w>>18)&127)*(2./127.)-1.,ay=(w>>25)*(2./127.)-1.,az=1.-std::abs(ax)-std::abs(ay);
    const double t=std::max(-az,0.);ax+=ax<0.?t:-t;ay+=ay<0.?t:-t;
    const double nl=std::sqrt(ax*ax+ay*ay+az*az);
    return (nx*ax+ny*ay+nz*az)>=.80*nl && std::abs(dx*ax+dy*ay+dz*az)<=.00060*nl;
}
inline std::array<double,3> witnessPoint(const ContactCell& c) {
    return {{(c.x+((c.witness&63)+.5)/64.)*.008,(c.y+(((c.witness>>6)&63)+.5)/64.)*.008,
             (c.z+(((c.witness>>12)&63)+.5)/64.)*.008}};
}
struct Sample { std::uint16_t dose{};std::uint8_t heat{},unused{};U32 tick{}; };
static_assert(sizeof(Sample)==8,"packed burn cell");
struct Texel { U32 x{},y{},z{},w{}; };
static_assert(sizeof(Texel)==16,"R32G32B32A32_UINT");

constexpr unsigned Slots=4096,MaxFine=512,MaxEntries=3072,MaxOwners=512,TextureWidth=256;
constexpr U32 Dead=0xffffffffu;
inline unsigned halfLife(unsigned domain) { return domain==Backing?192u:domain==Exterior?128u:160u; }
inline double cooled(Sample s,U32 tick,unsigned domain) {
    const U32 age=tick-s.tick;
    if(!s.heat||age>=12*halfLife(domain))return 0.;
    return s.heat*std::exp2(-double(age)/halfLife(domain));
}
inline unsigned normalizedHeat(Sample s,U32 tick,unsigned domain) {
    return unsigned(std::lround(cooled(s,tick,domain)));
}
// Out-of-order events add their heat at the current sample time and retain their dose.
inline Sample add(Sample s,U32 tick,unsigned dose,unsigned heat,unsigned domain) {
    const bool newer=(!s.dose&&!s.heat)||U32(tick-s.tick)<0x80000000u;
    const U32 time=newer?tick:s.tick;
    Sample incoming{0,std::uint8_t(heat),0,tick};
    return {std::uint16_t(std::min(65535u,s.dose+dose)),std::uint8_t(std::min(255u,unsigned(std::lround(cooled(s,time,domain)+cooled(incoming,time,domain))))),0,time};
}
inline Sample strongest(Sample a,Sample b,unsigned domain) {
    if(!a.dose&&!a.heat)return b;
    if(!b.dose&&!b.heat)return a;
    const U32 t=U32(a.tick-b.tick)<0x80000000u?a.tick:b.tick;
    return {std::max(a.dose,b.dose),std::uint8_t(std::max(normalizedHeat(a,t,domain),normalizedHeat(b,t,domain))),0,t};
}
struct Clock16 {
    U32 previous{};U64 simulatedMs{};bool ready{};
    U32 observe(U32 deviceTime) {
        if(!ready){previous=deviceTime;ready=true;return U32(simulatedMs*16/1000);}
        const U32 elapsed=deviceTime-previous;
        previous=deviceTime;
        if(elapsed<0x80000000u)simulatedMs+=elapsed;
        return U32(simulatedMs*16/1000);
    }
};
struct Key { int x{},y{},z{};U32 tag{}; };
inline bool operator==(Key a,Key b){return a.x==b.x&&a.y==b.y&&a.z==b.z&&a.tag==b.tag;}
inline U32 hash(Key k) {
    U32 h=U32(k.x)*0x8da6b343u^U32(k.y)*0xd8163841u^U32(k.z)*0xcb1ab31fu^k.tag*0x9e3779b9u;
    h^=h>>16;h*=0x7feb352du;h^=h>>15;return h;
}
inline int div4(int v){return v/4-int(v%4<0);}
inline Key key(unsigned owner,int x,int y,int z,unsigned domain,U32 surface,bool single=false){
    const U32 tag=(surface<<12)|(owner<<2)|domain;
    return single?Key{x,y,z,tag|SingleCell}:Key{div4(x),div4(y),div4(z),tag};
}
inline unsigned lane(int x,int y,int z){return (U32(x)&3u)|((U32(y)&3u)<<2)|((U32(z)&3u)<<4);}
inline U32 packed(Sample s){return U32(s.dose)|(U32(s.heat)<<16);}
enum class Status { Applied, NoChange, StaleOwner, Duplicate, OldSequence, BadContact, Capacity };
struct Page { std::array<Sample,64> cell{};std::array<U32,64> witness{}; };
struct Entry {Key k{};Sample coarse{};U64 mask{};U32 fine{},witness{};};
struct Owner {
    OwnerKey key{};U64 latest{},seen{};U32 tag{};Recession recession;unsigned spreadSlot{};bool carriedDose{},seedAfterFork{};
    Status verify(U64 n) const {
        if(!n)return Status::BadContact;
        if(!seen||n>latest)return Status::Applied;
        const U64 age=latest-n;
        if(age>=64)return Status::OldSequence;
        return seen&(U64(1)<<age)?Status::Duplicate:Status::Applied;
    }
    void record(U64 n) {
        if(!seen||n>latest){const auto delta=n-latest;seen=(!seen||delta>=64)?1:(seen<<delta)|1;latest=n;}
        else seen|=U64(1)<<(latest-n);
    }
};
struct Family {
    explicit Family(unsigned fine=MaxFine,unsigned entries=MaxEntries):maxFine(std::min(fine,MaxFine)),maxEntries(std::min(entries,MaxEntries)){}
    Family(Family&&)=default;Family& operator=(Family&&)=default;
    Family(const Family&)=delete;Family& operator=(const Family&)=delete;
    bool hasLiveSpread(const Owner& owner) const {
        const unsigned slot=owner.spreadSlot;
        if(!slot||slot>grids.size()||!grids[slot-1])return false;
        const auto& inv=grids[slot-1]->inv;
        return inv[0]>0.f && inv[1]>0.f && inv[2]>0.f;
    }
    unsigned ownerTag(OwnerKey k) const {for(const auto& o:owners)if(o.key==k)return o.tag;return 0;}
    unsigned birth(OwnerKey k) {
        if(!k.spawn||owners.size()>=MaxOwners)return 0;
        for(auto& o:owners)if(o.key.spawn==k.spawn)return o.key==k?o.tag:0;
        unsigned tag=1;for(;tag<=MaxOwners;++tag)if(!std::any_of(owners.begin(),owners.end(),[tag](const Owner& o){return o.tag==tag;}))break;
        owners.push_back({k,0,0,tag,{}});++version;return tag;
    }
    bool rebind(OwnerKey old,U64 nextGeneration) {
        Owner* o=getOwner(old);if(!o||nextGeneration<=old.generation)return false;
        o->key.generation=nextGeneration;++version;return true;
    }
    bool cancel(OwnerKey k) {
        unsigned tag=ownerTag(k);if(!tag)return false;
        if(getOwner(k)->recession.count||getOwner(k)->spreadSlot)frontDirty[tag-1]=true;
        if(entries)for(unsigned i=0;i<Slots;++i){auto& e=(*entries)[i];
            if(((e.k.tag>>2)&1023u)!=tag||!e.k.tag||e.k.tag==Dead)continue;
            if(e.fine){pages[e.fine-1].reset();fineDirty[e.fine-1]=true;}
            e=Entry{};e.k.tag=Dead;dirDirty[i]=true;--occupied;
        }
        owners.erase(std::remove_if(owners.begin(),owners.end(),[k](const Owner& o){return o.key==k;}),owners.end());
        ++version;return true;
    }
void saveSpread(std::vector<U32>& out,U32 tick)const{
        std::array<bool,painter_fire::Slots> used{};U32 bindings=0;
        for(const auto& o:owners)if(o.spreadSlot){used[o.spreadSlot-1]=true;++bindings;}
        if(!bindings)return;
        constexpr U32 Tag=0x32525053;
        bool extendedAge=false;
        for(unsigned i=0;i<used.size();++i)if(used[i]){
            const auto& state=*grids[i];
            const U32 age=tick-state.heatTick;
            if(state.heatStrength && age>=2304u && age<4800u){extendedAge=true;break;}
        }
        out.insert(out.end(),{Tag,extendedAge?4u:3u,U32(std::count(used.begin(),used.end(),true)),bindings});
        for(U32 i=0;i<owners.size();++i){const auto& o=owners[i];if(o.spreadSlot)out.insert(out.end(),{i,o.spreadSlot|(o.seedAfterFork?0x100u:0u)});}
        for(unsigned i=0;i<used.size();++i)if(used[i]){
            const auto& state=*grids[i];
            const U32 age=tick-state.heatTick;
            const U32 strength=age<4800u?state.heatStrength:0u;
            out.insert(out.end(),{i+1,state.clock,strength?age:0u,strength});
            for(auto x:state.lo)out.push_back(burnBits(x));
            for(auto x:state.inv)out.push_back(burnBits(x));
            U32 count=0;auto countAt=out.size();out.push_back(0);
            for(int j=0;j<painter_fire::Cells;++j){
                if(!(state.arrival[j]||state.progress[j]||state.display[j]))continue;
                out.insert(out.end(),{U32(j),state.arrival[j],U32(state.progress[j])|(U32(state.display[j])<<16)});
                ++count;
            }
            out[countAt]=count;
        }
        out.push_back(rig?rig->valid:0u);
        if(rig)for(unsigned j=0;j<12;j++)if(rig->valid&(1u<<j))for(auto x:rig->joints[j])out.push_back(burnBits(x));
    }
    bool loadSpread(const std::vector<U32>& data,std::size_t& i,U32 now){
        if(data.size()-i<4||data[i++]!=0x32525053u)return false;
        const U32 revision=data[i++];if(revision!=1&&revision!=2&&revision!=3&&revision!=4)return false;
        const auto states=data[i++],bindings=data[i++];
        if(!states||states>painter_fire::Slots||!bindings||bindings>owners.size()||data.size()-i<2ull*bindings)return false;
        std::array<bool,painter_fire::Slots> used{},loaded{};
        for(U32 j=0;j<bindings;++j){
            const auto owner=data[i++],word=data[i++];if(word&~0x1ffu)return false;
            const auto slot=word&0xffu;
            if(owner>=owners.size()||owners[owner].spreadSlot||!slot||slot>used.size())return false;
            used[slot-1]=true;owners[owner].spreadSlot=slot;owners[owner].seedAfterFork=(word&0x100u)!=0;frontDirty[owners[owner].tag-1]=true;
        }
        if(states!=std::count(used.begin(),used.end(),true))return false;
        for(U32 n=0;n<states;++n){
            if(data.size()-i<9)return false;const auto slot=data[i++];if(!slot||slot>used.size()||!used[slot-1]||loaded[slot-1])return false;
            auto s=std::make_shared<painter_fire::State>();s->clock=data[i++];if(s->clock>=0x7fffffffu)return false;
            if(revision>=3){if(data.size()-i<2)return false;const auto age=data[i++],strength=data[i++];if(age>(revision>=4?4800u:2304u)||strength>255||(!strength&&age))return false;
                s->heatTick=strength?now-(revision==3&&age==2304u?4800u:age):0u;s->heatStrength=strength;}
            for(auto& x:s->lo){x=burnFloat(data[i++]);if(!std::isfinite(x)||std::abs(x)>10000.f)return false;}
            for(auto& x:s->inv){x=burnFloat(data[i++]);if(!std::isfinite(x)||x<=0.f||x>10000.f)return false;}
            const auto count=data[i++];if(count>painter_fire::Cells||data.size()-i<3ull*count)return false;
            U32 prev=0;for(U32 j=0;j<count;++j){
                const auto idx=data[i++],arrival=data[i++],values=data[i++];
                if(idx>=painter_fire::Cells||(j&&idx<=prev)||arrival>s->clock+1||!(arrival||values))return false;
                prev=idx;s->arrival[idx]=arrival;s->progress[idx]=painter_fire::H(values);s->display[idx]=painter_fire::H(values>>16);
                if(s->progress[idx]==65535||((s->display[idx]&2047u)==0&&s->display[idx]))return false;
            }
            s->initialized=true;s->needsSeeds=false;s->resumeGate=true;s->last=now;
            grids[slot-1]=std::move(s);spreadDirty[slot-1]=~U64(0);loaded[slot-1]=true;
        }
        if(revision>=2){
            if(i==data.size())return false;const auto mask=data[i++];if(mask>=4096)return false;
            if(mask){auto v=std::make_shared<painter_fire::Rig>();v->valid=mask;
                for(unsigned j=0;j<12;j++)if(mask&(1u<<j)){
                    if(data.size()-i<3)return false;
                    for(auto& x:v->joints[j]){x=burnFloat(data[i++]);if(!std::isfinite(x)||std::abs(x)>10000.f)return false;}
                }
                rig=std::move(v);
            }
        }
        rigDirty=true;return true;
    }
Status reach(OwnerKey key,const painter_fire::Shape& shape,const painter_fire::Pose& pose,
        const painter_fire::V* seeds,unsigned count,U32 tick,unsigned period,U64 salt){
        auto* owner=getOwner(key);if(!owner)return Status::StaleOwner;
        if(shape.active.empty()||!seeds||!count)return Status::BadContact;
        unsigned slot=owner->spreadSlot;
        // Forked parts share their last accepted grid until one really burns.
        bool shared=slot&&std::any_of(owners.begin(),owners.end(),[&](const Owner& o){return o.tag!=owner->tag&&o.spreadSlot==slot;});
        bool fresh=!slot||shared;
        if(fresh){
            unsigned next=1;
            for(;next<=painter_fire::Slots;next++)if(std::none_of(owners.begin(),owners.end(),[&](const Owner& o){return o.spreadSlot==next;}))break;
            if(next>painter_fire::Slots)return Status::Capacity;
            auto state=slot&&grids[slot-1]?std::make_shared<painter_fire::State>(*grids[slot-1]):std::make_shared<painter_fire::State>();
            if(shared)state->last=tick;grids[next-1]=std::move(state);
            slot=owner->spreadSlot=next;frontDirty[owner->tag-1]=true;spreadDirty[next-1]=~U64(0);
        }
        if(!rig&&pose.rig&&pose.rig->valid){rig=pose.rig;rigDirty=true;}
        auto& s=*grids[slot-1];if(owner->seedAfterFork){s.needsSeeds=true;owner->seedAfterFork=false;}
        const auto before=s.display;const auto clock=s.clock;
        const bool changed=s.step(shape,pose,seeds,count,tick,period,salt);
        const auto activeTicks=s.clock-clock;
        // Optical ages are frozen off-fire. The visor decays from a real
        // confirmed timestamp without asking the worker for refreshes.
        if(activeTicks){
            const U32 age=tick-s.heatTick;
            const float residual=s.heatStrength&&age<1536u?float(s.heatStrength)*std::exp2(-float(age)/128.f):0.f;
            s.heatStrength=std::min(255u,U32(std::lround(residual+31.875f*activeTicks)));
            s.heatTick=tick;
        }
        if(!changed&&!activeTicks&&!fresh)return Status::NoChange;
        for(unsigned chunk=0;chunk<32;chunk++)if(std::memcmp(s.display.data()+128*chunk,before.data()+128*chunk,128*sizeof(std::uint16_t)))spreadDirty[slot-1]|=U64(1)<<chunk;
        spreadDirty[slot-1]|=U64(1)<<32;++version;return Status::Applied;
    }
    template<class Write>void publishSpread(Write&& write){
        if(rigDirty){
            std::array<Texel,12> rows{};
            if(rig)for(unsigned i=0;i<12;i++)if(rig->valid&(1u<<i)){
                const auto& v=rig->joints[i];rows[i]={burnBits(v[0]),burnBits(v[1]),burnBits(v[2]),0x52494731u};
            }
            write(SpreadBase+painter_fire::Slots*painter_fire::Words,rows.data(),12);rigDirty=false;
        }
        for(unsigned slot=0;slot<painter_fire::Slots;slot++){
            const auto bits=spreadDirty[slot];if(!bits||!grids[slot])continue;
            const auto& s=*grids[slot];const unsigned base=SpreadBase+slot*painter_fire::Words;
            auto emit=[&](unsigned start,const Texel* data,unsigned n){
                while(n){unsigned take=std::min(n,TextureWidth-start%TextureWidth);write(start,data,take);data+=take;start+=take;n-=take;}
            };
            if(bits&(U64(1)<<32)){
                const Texel desc[2]={{burnBits(s.lo[0]),burnBits(s.lo[1]),burnBits(s.lo[2]),s.clock},
                    {burnBits(s.inv[0]),burnBits(s.inv[1]),burnBits(s.inv[2]),painter_fire::Magic}};emit(base,desc,2);
                const Texel heat={s.heatTick,s.heatStrength?0x48454154u:0u,s.heatStrength,0u};
                emit(SpreadBase+painter_fire::Slots*painter_fire::Words+12u+slot,&heat,1);
            }
            for(unsigned chunk=0;chunk<32;chunk++)if(bits&(U64(1)<<chunk)){
                std::array<Texel,16> encoded{};std::memcpy(encoded.data(),s.display.data()+128*chunk,sizeof(encoded));emit(base+2+16*chunk,encoded.data(),16);
                std::array<Texel,32> optical{};
                for(unsigned row=0;row<8;++row){
                    const auto* raw=s.display.data()+128u*chunk+16u*row;
                    for(unsigned group=0;group<4;++group)
                        optical[4u*row+group]=opticalRow(raw,group);
                }
                emit(OpticalBase+slot*OpticalSlotTexels+32u*chunk,optical.data(),unsigned(optical.size()));
            }
            spreadDirty[slot]=0;
        }
    }
    Status expose(OwnerKey k,const RecessionFront* fronts,std::size_t count){
        auto* owner=getOwner(k);if(!owner)return Status::StaleOwner;
        if(!fronts||count>Recession::Limit)return Status::BadContact;
        bool changed=false;for(std::size_t i=0;i<count;++i)changed|=owner->recession.update(fronts[i]);
        if(!changed)return Status::NoChange;
        frontDirty[owner->tag-1]=true;++version;return Status::Applied;
    }
    Status apply(OwnerKey k,U64 sequence,U32 tick,const ContactCell* cells,std::size_t count) {
        Owner* owner=getOwner(k);if(!owner)return Status::StaleOwner;
        if((!cells&&count)||count>MaxCells)return Status::BadContact;
        auto status=owner->verify(sequence);if(status!=Status::Applied)return status;
        struct Pending {unsigned slot,lane;ContactCell c;};
        std::vector<Pending> work;work.reserve(count);
        std::vector<std::pair<unsigned,U32>> created;
        auto undo=[&]{for(const auto& it:created){(*entries)[it.first]=Entry{};(*entries)[it.first].k.tag=it.second;--occupied;}};
        unsigned remainingFine=unsigned(maxFine-fineCount());
        for(std::size_t i=0;i<count;++i){
            const auto& c=cells[i];if(c.domain<Exterior||c.domain>Backing||!c.surface||c.surface>MaxSurface){undo();return Status::BadContact;}
            if(!c.heat&&!c.dose)continue;
            if(!entries)entries=std::make_unique<Directory>();
            Key q=key(owner->tag,c.x,c.y,c.z,c.domain,c.surface);
            unsigned slot=locate(q,false);
            if(slot==Slots || !((*entries)[slot].mask&(U64(1)<<lane(c.x,c.y,c.z)))){
                const Key single=key(owner->tag,c.x,c.y,c.z,c.domain,c.surface,true);
                const unsigned existingSingle=locate(single,false);
                if(existingSingle!=Slots)slot=existingSingle;
                if(slot==Slots){
                    if(!remainingFine)q=single;
                    else --remainingFine;
                    slot=locate(q,true);
                }
            }
            if(slot==Slots){undo();return Status::Capacity;}
            auto& e=(*entries)[slot];
            if(!e.k.tag||e.k.tag==Dead){
                if(occupied>=maxEntries){undo();return Status::Capacity;}
                created.emplace_back(slot,e.k.tag);e=Entry{};e.k=q;++occupied;
            }
            work.push_back({slot,lane(c.x,c.y,c.z),c});
        }
        std::sort(work.begin(),work.end(),[](const Pending& a,const Pending& b){return a.slot<b.slot||(a.slot==b.slot&&a.lane<b.lane);});
        if(std::adjacent_find(work.begin(),work.end(),[](const Pending& a,const Pending& b){return a.slot==b.slot&&a.lane==b.lane;})!=work.end()){
            undo();return Status::BadContact;
        }
        for(const auto& it:created){auto& e=(*entries)[it.first];if(!(e.k.tag&SingleCell))e.fine=allocateFine();dirDirty[it.first]=true;}
        for(std::size_t i=0;i<work.size();){
            const unsigned slot=work[i].slot;auto& e=(*entries)[slot];
            const Sample old=e.coarse;const U64 oldMask=e.mask;
            do {const auto& p=work[i];const auto bit=U64(1)<<p.lane;
                if(e.fine){
                    auto& s=pages[e.fine-1]->cell[p.lane];
                    if(!(oldMask&bit)||U32(tick-s.tick)<0x80000000u)pages[e.fine-1]->witness[p.lane]=p.c.witness;
                    s=add(s,tick,p.c.dose,p.c.heat,p.c.domain);fineDirty[e.fine-1]=true;
                }else{
                    const Sample existing=(oldMask&bit)?old:Sample{};
                    if(!(oldMask&bit)||U32(tick-existing.tick)<0x80000000u)e.witness=p.c.witness;
                    e.coarse=add(existing,tick,p.c.dose,p.c.heat,p.c.domain);
                    dirDirty[slot]=true;
                }
                e.mask|=bit;++i;
            }while(i<work.size()&&work[i].slot==slot);
        }
        owner->record(sequence);if(work.empty())return Status::NoChange;
        ++version;return Status::Applied;
    }
    Sample sample(OwnerKey k,double x,double y,double z,double nx,double ny,double nz,unsigned domain,U32 surface) const {
        const unsigned tag=ownerTag(k);ContactCell c;
        if(!tag||!entries||!makeContact(x,y,z,nx,ny,nz,surface,domain,0,0,c))return {};
        unsigned l=lane(c.x,c.y,c.z);const double len=std::sqrt(nx*nx+ny*ny+nz*nz);
        for(bool single:{false,true}){
            const unsigned slot=locate(key(tag,c.x,c.y,c.z,domain,surface,single),false);
            if(slot==Slots)continue;
            const auto& e=(*entries)[slot];if(!(e.mask&(U64(1)<<l)))continue;
            const U32 w=e.fine?pages[e.fine-1]->witness[l]:e.witness;
            if(matchesWitness(w,x,y,z,nx/len,ny/len,nz/len))return e.fine?pages[e.fine-1]->cell[l]:e.coarse;
        }
        return {};
    }
    // One transaction replaces the native parent's receiver with its surviving
    // and detached receivers, after physical ownership is known. The callback
    // checks the previously exposed face/witness against each exact piece seed.
    template<class Owns> Status fork(OwnerKey parent,const OwnerKey* children,std::size_t n,Owns&& owns) {
        auto* old=getOwner(parent);if(!old)return Status::StaleOwner;
        if(!children||!n||n>MaxOwners||owners.size()-1+n>MaxOwners)return Status::Capacity;
        const unsigned oldTag=old->tag;const U64 latest=old->latest,seen=old->seen;
        Family next(maxFine,maxEntries);next.occupied=occupied;next.version=version;
        next.rig=rig;next.rigDirty=rigDirty;next.grids=grids;next.spreadDirty=spreadDirty;next.owners=owners;next.dirDirty=dirDirty;next.fineDirty=fineDirty;next.frontDirty=frontDirty;
        if(entries)next.entries=std::make_unique<Directory>(*entries);
        for(unsigned i=0;i<MaxFine;++i)if(pages[i])next.pages[i]=std::make_unique<Page>(*pages[i]);
        next.cancel(parent);
        for(std::size_t i=0;i<n;++i){
            if(!children[i].spawn||(children[i].spawn==parent.spawn&&children[i].generation<=parent.generation)||
               next.ownerTag(children[i])||!next.birth(children[i]))return Status::BadContact;
            auto* target=next.getOwner(children[i]);target->latest=latest;target->seen=seen;
            target->spreadSlot=old->spreadSlot;target->seedAfterFork=old->spreadSlot!=0;target->carriedDose=old->carriedDose;target->recession=old->recession;if(target->recession.count||target->spreadSlot)next.frontDirty[target->tag-1]=true;
        }
        if(entries)for(const auto& e:*entries){
            if(!e.k.tag||e.k.tag==Dead||((e.k.tag>>2)&1023u)!=oldTag)continue;
            const bool single=(e.k.tag&SingleCell)!=0;
            for(unsigned j=0;j<64;++j)if(e.mask&(U64(1)<<j)){
                const Sample s=e.fine?pages[e.fine-1]->cell[j]:e.coarse;
                ContactCell c{};
                c.x=single?e.k.x:4*e.k.x+int(j&3u);c.y=single?e.k.y:4*e.k.y+int((j>>2)&3u);c.z=single?e.k.z:4*e.k.z+int(j>>4);
                c.domain=std::uint8_t(e.k.tag&3u);c.surface=(e.k.tag&~SingleCell)>>12;c.dose=s.dose;c.heat=s.heat;
                c.witness=e.fine?pages[e.fine-1]->witness[j]:e.witness;
                const auto point=witnessPoint(c);
                for(std::size_t i=0;i<n;++i)if(owns(children[i],c,point)){
                    auto* target=next.getOwner(children[i]);
                    if(!next.copyKnown(target->tag,c,s,single))return Status::Capacity;
                }
            }
        }
        ++next.version;*this=std::move(next);return Status::Applied;
    }
    U64 revision()const{return version;}
    const std::vector<Owner>& activeOwners()const{return owners;}
    void dirtyAll(){
        rigDirty=bool(rig);for(const auto& o:owners){if(o.recession.count||o.spreadSlot)frontDirty[o.tag-1]=true;if(o.spreadSlot)spreadDirty[o.spreadSlot-1]=~U64(0);}
        if(!entries){++version;return;}
        for(unsigned i=0;i<Slots;++i)if((*entries)[i].k.tag&&(*entries)[i].k.tag!=Dead)dirDirty[i]=true;
        for(unsigned i=0;i<MaxFine;++i)if(pages[i])fineDirty[i]=true;
        ++version;
    }
    // A native change destroys this exact once-exposed surface patch.
    // Used when a source fragment or prior cut face itself disappeared.
    bool forget(OwnerKey owner,const ContactCell& c){
        const auto tag=ownerTag(owner);if(!entries||!tag||!c.surface||c.surface>MaxSurface||c.domain<1||c.domain>3)return false;
        const unsigned l=lane(c.x,c.y,c.z);const auto bit=U64(1)<<l;
        for(bool single:{false,true}){
            const auto slot=locate(key(tag,c.x,c.y,c.z,c.domain,c.surface,single),false);
            if(slot==Slots)continue;auto& e=(*entries)[slot];if(!(e.mask&bit))continue;
            e.mask&=~bit;
            if(e.fine){pages[e.fine-1]->cell[l]={};pages[e.fine-1]->witness[l]=0;fineDirty[e.fine-1]=true;}
            if(!e.mask){if(e.fine)pages[e.fine-1].reset();e=Entry{};e.k.tag=Dead;--occupied;dirDirty[slot]=true;}
            ++version;return true;
        }
        return false;
    }
    // Compact save chunks contain live witnesses, not hash buckets or GPU handles.
    // The native host binds its own lineage cookie before accepting any reload.
    std::vector<U32> save(U64 lineage,U32 tick)const{
        std::vector<U32> out={0x324e5242u,U32(lineage),U32(lineage>>32),U32(owners.size()),0};
        for(const auto& o:owners)for(auto w:{o.key.spawn,o.key.generation,o.latest,o.seen}){
            out.push_back(U32(w));out.push_back(U32(w>>32));
        }
        if(entries)for(const auto& e:*entries){
            if(!e.k.tag||e.k.tag==Dead)continue;
            const U32 tag=(e.k.tag>>2)&1023u;auto owner=std::find_if(owners.begin(),owners.end(),[=](const Owner& o){return o.tag==tag;});
            const U32 idx=U32(owner-owners.begin());
            const bool single=(e.k.tag&SingleCell)!=0;
            for(unsigned j=0;j<64;++j)if(e.mask&(U64(1)<<j)){
                const auto s=e.fine?pages[e.fine-1]->cell[j]:e.coarse;
                const U32 w=e.fine?pages[e.fine-1]->witness[j]:e.witness;
                out.insert(out.end(),{idx,((e.k.tag&~SingleCell)>>12)|(single?SingleCell:0),e.k.tag&3u,U32(single?e.k.x:e.k.x*4+int(j&3u)),
                    U32(single?e.k.y:e.k.y*4+int((j>>2)&3u)),U32(single?e.k.z:e.k.z*4+int(j>>4)),
                    packed(s),w,U32(tick-s.tick)});
                ++out[4];
            }
        }
        unsigned active=0;for(const auto& o:owners)active+=o.recession.count!=0;
        if(active){
            out.push_back(0x31524642u);out.push_back(active);
            for(U32 i=0;i<owners.size();++i){const auto& field=owners[i].recession;if(!field.count)continue;
                out.push_back(i);out.push_back(field.count);
                for(unsigned j=0;j<field.count;++j){const auto& f=field.fronts[j];
                    for(float v:f.centre)out.push_back(burnBits(v));
                    out.push_back(burnBits(f.radius));out.push_back(U32(f.charMm)|(U32(f.maturity)<<16));
                }
            }
        }
        saveSpread(out,tick);return out;
    }
    bool restore(U64 lineage,U32 now,const std::vector<U32>& data){
        if(!lineage||data.size()<5||data[0]!=0x324e5242u||data[1]!=U32(lineage)||data[2]!=U32(lineage>>32)||
           data[3]>MaxOwners||data[4]>64u*MaxFine+MaxEntries||data.size()<5ull+8ull*data[3]+9ull*data[4])return false;
        Family next(maxFine,maxEntries);std::size_t i=5;auto word=[&]{U64 n=data[i++];return n|(U64(data[i++])<<32);};
        for(U32 j=0;j<data[3];++j){
            OwnerKey o{word(),word()};U64 seq=word(),seen=word();
            if((seen&&(!seq||!(seen&1)))||next.ownerTag(o)||!next.birth(o))return false;
            next.owners.back().latest=seq;next.owners.back().seen=seen;
        }
        if(data[4])next.entries=std::make_unique<Directory>();
        for(U32 j=0;j<data[4];++j){
            U32 idx=data[i++],encoded=data[i++],domain=data[i++];const U32 surface=encoded&~SingleCell;
            int x=std::int32_t(data[i++]),y=std::int32_t(data[i++]),z=std::int32_t(data[i++]);
            U32 v=data[i++],w=data[i++],age=data[i++];
            if(idx>=next.owners.size()||!surface||surface>MaxSurface||domain<1||domain>3||!v||v>0xffffffu||
                std::abs(double(x))>125000000||std::abs(double(y))>125000000||std::abs(double(z))>125000000)return false;
            const ContactCell c{x,y,z,std::uint16_t(v),std::uint8_t(v>>16),std::uint8_t(domain),surface,w};
            next.owners[idx].carriedDose=true;unsigned tag=next.owners[idx].tag,l=lane(x,y,z);
            for(bool single:{false,true}){unsigned si=next.locate(key(tag,x,y,z,domain,surface,single),false);
                if(si!=Slots&&((*next.entries)[si].mask&(U64(1)<<l)))return false;}
            if(!next.copyKnown(tag,c,{c.dose,c.heat,0,now-age},(encoded&SingleCell)!=0))return false;
        }
        if(i<data.size()&&data[i]==0x31524642u){
            if(data.size()-i<2||data[i++]!=0x31524642u)return false;
            const auto records=data[i++];if(!records||records>next.owners.size())return false;
            for(U32 r=0;r<records;++r){
                if(data.size()-i<2)return false;const U32 index=data[i++],count=data[i++];
                if(index>=next.owners.size()||!count||count>Recession::Limit||data.size()-i<5ull*count)return false;
                auto& o=next.owners[index];if(o.recession.count)return false;
                for(unsigned j=0;j<count;++j){
                    RecessionFront f;for(auto& v:f.centre)v=burnFloat(data[i++]);f.radius=burnFloat(data[i++]);
                    const auto metadata=data[i++];if(metadata>>24)return false;
                    f.charMm=std::uint16_t(metadata);f.maturity=std::uint8_t(metadata>>16);
                    if(!Recession::valid(f))return false;o.recession.fronts[o.recession.count++]=f;
                }
                next.frontDirty[o.tag-1]=true;
            }
        }
        if(i<data.size()&&!next.loadSpread(data,i,now))return false;
        if(i!=data.size())return false;
        // A restore into an existing atlas must clear tags whose old owner had a front.
        for(const auto& o:owners)if(o.recession.count||o.spreadSlot)next.frontDirty[o.tag-1]=true;
        next.version=version+1;*this=std::move(next);return true;
    }
    std::size_t fineCount()const{std::size_t n=0;for(auto& p:pages)n+=p!=nullptr;return n;}
    static constexpr unsigned MaxCells=16384,MaxProbe=64;
    static constexpr unsigned FrontBase=2*Slots+64*MaxFine;
    static constexpr unsigned SpreadBase=FrontBase+Recession::Words*MaxOwners;
    // Six consecutive 11-bit authored exposure codes cover all 3-tap row reads in a group of four.
    // Each texel shares its own owner's transaction, dirty interval and device-generation swap.
    static constexpr unsigned OpticalBase=((SpreadBase+painter_fire::Slots*painter_fire::Words+12+painter_fire::Slots+TextureWidth-1)/TextureWidth)*TextureWidth;
    static constexpr unsigned OpticalSlotTexels=4096/4;
    static Texel opticalRow(const std::uint16_t* row,unsigned group){
        const int lo=int(group*4)-1;
        const auto code=[&](int x)->U32 {return U32(row[x<0?0:x>15?15:x])&2047u;};
        return {code(lo)|(code(lo+1)<<11u),code(lo+2)|(code(lo+3)<<11u),
                code(lo+4)|(code(lo+5)<<11u),0u};
    }
    static constexpr unsigned GpuTexels=OpticalBase+painter_fire::Slots*OpticalSlotTexels;
    static_assert(OpticalBase==61696u && OpticalSlotTexels==1024u,"burn optical field layout");
    static constexpr unsigned GpuBytes=GpuTexels*sizeof(Texel);
    bool hasExposure()const{return occupied!=0||std::any_of(owners.begin(),owners.end(),[](const Owner& o){return o.recession.count!=0||o.spreadSlot!=0;});}
    static constexpr std::size_t directoryBytes(){return Slots*sizeof(Entry);}
    template<class Write> void publishDirty(Write&& write) {
        publishSpread(write);
        for(unsigned i=0;i<MaxOwners;++i)if(frontDirty[i]){
            const auto found=std::find_if(owners.begin(),owners.end(),[&](const Owner& o){return o.tag==i+1;});
            auto row=(found==owners.end()?Recession{}:found->recession).encode<Texel>();
            row[7].z=found==owners.end()?0:found->spreadSlot;row[7].w=(found!=owners.end()&&(found->carriedDose||found->recession.count))?1u:0u;
            write(FrontBase+i*Recession::Words,row.data(),Recession::Words);frontDirty[i]=false;
        }
        if(!entries)return;
        // Data upload precedes every new directory link. A renderer publishes the
        // whole transaction only after the final write on that same D3D context.
        for(unsigned i=0;i<MaxFine;++i)if(fineDirty[i]){
            std::array<Texel,64> data{};
            if(pages[i])for(unsigned j=0;j<64;++j){auto a=pages[i]->cell[j];data[j]={packed(a),a.tick,pages[i]->witness[j],0};}
            write(2*Slots+64*i,data.data(),64);
        }
        for(unsigned i=0;i<Slots;++i)if(dirDirty[i]){
            const auto& e=(*entries)[i];const Key q=e.k;
            Texel v[2]={{U32(q.x),U32(q.y),U32(q.z),q.tag},{}};
            v[1]=e.fine?Texel{e.fine,0,0,0}:Texel{packed(e.coarse),e.coarse.tick,e.witness,0};
            write(2*i,v,2);
        }
        dirDirty.fill(false);fineDirty.fill(false);
    }
private:
    using Directory=std::array<Entry,Slots>;
    unsigned maxFine{},maxEntries{},occupied{};
    U64 version{};
    std::unique_ptr<Directory> entries;
    std::vector<Owner> owners;
    std::array<std::unique_ptr<Page>,MaxFine> pages{};
    std::shared_ptr<const painter_fire::Rig> rig;bool rigDirty{};
    std::array<std::shared_ptr<painter_fire::State>,painter_fire::Slots> grids{};
    std::array<U64,painter_fire::Slots> spreadDirty{};
    std::array<bool,MaxOwners> frontDirty{};
    std::array<bool,Slots> dirDirty{};std::array<bool,MaxFine> fineDirty{};
    Owner* getOwner(OwnerKey k){for(auto& o:owners)if(o.key==k)return &o;return nullptr;}
    unsigned allocateFine(){for(unsigned i=0;i<maxFine;++i)if(!pages[i]){pages[i]=std::make_unique<Page>();return i+1;}return 0;}
    bool copyKnown(unsigned tag,const ContactCell& c,Sample s,bool preferSingle){
        Key q=key(tag,c.x,c.y,c.z,c.domain,c.surface);
        unsigned slot=locate(q,false);
        if(slot==Slots){
            if(occupied>=maxEntries)return false;
            bool fine=!preferSingle&&fineCount()<maxFine;if(!fine)q=key(tag,c.x,c.y,c.z,c.domain,c.surface,true);
            slot=locate(q,true);if(slot==Slots)return false;
            auto& e=(*entries)[slot];e=Entry{};e.k=q;e.fine=fine?allocateFine():0;++occupied;dirDirty[slot]=true;
        }
        const auto j=lane(c.x,c.y,c.z);auto& e=(*entries)[slot];e.mask|=U64(1)<<j;
        if(e.fine){pages[e.fine-1]->cell[j]=s;pages[e.fine-1]->witness[j]=c.witness;fineDirty[e.fine-1]=true;}
        else{e.coarse=s;e.witness=c.witness;dirDirty[slot]=true;}
        return true;
    }
    unsigned locate(Key k,bool insert) const {
        if(!entries)return Slots;
        unsigned firstDeleted=Slots;
        for(unsigned n=0;n<MaxProbe;++n){
            const unsigned i=(hash(k)+n)&(Slots-1);const auto& e=(*entries)[i];
            if(e.k==k)return i;
            if(!e.k.tag)return insert?(firstDeleted==Slots?i:firstDeleted):Slots;
            if(e.k.tag==Dead&&firstDeleted==Slots)firstDeleted=i;
        }
        return insert?firstDeleted:Slots;
    }
};
}
