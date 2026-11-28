#pragma once
// Immutable source anatomy, shared by all wounds/owners of one model asset.
// Exact tissue slots at the source volume resolution. Uniform bricks consume
// one page entry; only mixed bricks use R8 storage. No damage or per-shot work.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

namespace painter_detail {
struct TissueField {
    static constexpr unsigned brickShift=3,brickSize=1u<<brickShift,brickCells=brickSize*brickSize*brickSize;
    static constexpr std::uint32_t uniformBit=0x80000000u,headerWords=16,pageWidth=1024,dataWidth=4096;
    std::array<float,3> origin{};float pitch{};
    std::array<std::uint32_t,3> size{},pages{};
    std::vector<std::uint32_t> page; // first 16 words encode shader parameters
    std::vector<std::uint8_t> data; // mixed bricks: 1/2/4-bit palette, or direct R8
    std::uint32_t mixed{},uniform{},empty{};
    bool available{};
    std::size_t bytes()const{return page.size()*4+data.size();}
    unsigned at(int x,int y,int z)const noexcept{
        if(!available||x<0||y<0||z<0||unsigned(x)>=size[0]||unsigned(y)>=size[1]||unsigned(z)>=size[2])return 0;
        std::size_t p=(std::size_t(z>>brickShift)*pages[1]+unsigned(y>>brickShift))*pages[0]+unsigned(x>>brickShift)+headerWords;
        unsigned v=page[p];if(v&uniformBit)return v&127u; // R72: bit 64 = clothing padding (markClosurePadding); readers mask with 63
        unsigned local=((unsigned(z)&7u)*8+(unsigned(y)&7u))*8+(unsigned(x)&7u);
        unsigned bits=1u<<((v>>28)&3u),offset=v&0x0fffffffu;
        if(bits==8)return data[offset+local];
        unsigned palette=1u<<bits,bit=local*bits;
        unsigned index=(data[offset+palette+(bit>>3)]>>(bit&7))&((1u<<bits)-1);
        return data[offset+index];
    }
    unsigned sample(double x,double y,double z)const noexcept{
        return at(int(std::floor((x-origin[0])/pitch)),int(std::floor((y-origin[1])/pitch)),int(std::floor((z-origin[2])/pitch)));
    }
};
// Sparse must expose the existing material_volume::Sparse layout. Work is
// proportional to occupied voxels and run count; scratch is one 8x8 xy column.
// The limit includes GPU row padding. A refusal preserves the existing material
// path, never refuses a wound or reduces geometry/anatomy simulation.
template<class Sparse> std::shared_ptr<const TissueField> buildTissueField(const Sparse&s,const std::vector<std::uint16_t>&slots,std::size_t limit=32u<<20){
    auto out=std::make_shared<TissueField>();auto&v=*out;v.pitch=float(s.pitch);
    for(int a=0;a<3;++a){v.origin[a]=float(s.origin[a]);v.size[a]=s.size[a];v.pages[a]=(s.size[a]+7)/8;}
    const std::size_t pages=std::size_t(v.pages[0])*v.pages[1]*v.pages[2];
    const std::size_t paddedPages=((pages+TissueField::headerWords+TissueField::pageWidth-1)/TissueField::pageWidth)*TissueField::pageWidth;
    if(paddedPages*4>=limit)return out;
    v.page.assign(paddedPages,TissueField::uniformBit);
    std::vector<std::uint8_t> slab(std::size_t(v.pages[2])*TissueField::brickCells);
    for(unsigned by=0;by<v.pages[1];++by)for(unsigned bx=0;bx<v.pages[0];++bx){
        std::fill(slab.begin(),slab.end(),0);
        for(unsigned ly=0;ly<8;++ly)for(unsigned lx=0;lx<8;++lx){unsigned x=bx*8+lx,y=by*8+ly;if(x>=s.size[0]||y>=s.size[1])continue;
            std::size_t col=std::size_t(y)*s.size[0]+x;
            for(auto r=s.offsets[col];r<s.offsets[col+1];++r){const auto&run=s.runs[r];std::uint8_t slot=run.label<slots.size()?std::uint8_t(slots[run.label]&63u):0;
                for(unsigned z=run.lo;z<run.hi;++z)slab[std::size_t(z>>3)*512+((z&7)*8+ly)*8+lx]=slot;}}
        for(unsigned bz=0;bz<v.pages[2];++bz){auto first=slab.begin()+std::size_t(bz)*512,last=first+512;std::uint8_t value=*first;
            std::size_t p=(std::size_t(bz)*v.pages[1]+by)*v.pages[0]+bx+TissueField::headerWords;
            if(std::all_of(first,last,[value](std::uint8_t x){return x==value;})){v.page[p]=TissueField::uniformBit|value;value?++v.uniform:++v.empty;}
            else{
                std::array<std::uint8_t,64> present{},index{};unsigned count=0;
                for(auto it=first;it!=last;++it)present[*it]=1;
                for(unsigned slot=0;slot<64;++slot)if(present[slot])index[slot]=std::uint8_t(count++);
                unsigned mode=count<=2?0:count<=4?1:count<=16?2:3,bits=1u<<mode,palette=bits==8?0:1u<<bits;
                std::size_t bytes=palette+512*bits/8,offset=v.data.size();
                if(v.page.size()*4+offset+bytes+TissueField::dataWidth>limit){std::vector<std::uint32_t>().swap(v.page);std::vector<std::uint8_t>().swap(v.data);return out;}
                v.page[p]=(mode<<28)|std::uint32_t(offset);++v.mixed;v.data.resize(offset+bytes,0);
                if(bits==8)std::copy(first,last,v.data.begin()+offset);
                else{
                    for(unsigned slot=0;slot<64;++slot)if(present[slot])v.data[offset+index[slot]]=std::uint8_t(slot);
                    for(unsigned i=0;i<512;++i){unsigned bit=i*bits;v.data[offset+palette+(bit>>3)]|=std::uint8_t(index[first[i]]<<(bit&7));}
                }
            }
        }
    }
    v.data.resize(std::max<std::size_t>(TissueField::dataWidth,((v.data.size()+TissueField::dataWidth-1)/TissueField::dataWidth)*TissueField::dataWidth),0);
    auto fbits=[](float f){std::uint32_t u;std::memcpy(&u,&f,4);return u;};
    for(int a=0;a<3;++a){v.page[a]=fbits(v.origin[a]);v.page[4+a]=v.size[a];v.page[8+a]=v.pages[a];}
    v.page[3]=fbits(1.f/v.pitch);v.page[7]=TissueField::brickShift;v.page[11]=TissueField::dataWidth;v.page[12]=TissueField::pageWidth;v.page[13]=TissueField::headerWords;
    v.available=true;return out;
}
// R72 (2026-10-22): the closure field with its clothing padding marked. A cell with a closure label (inside the closed garment
// envelope) where the anatomy has no tissue - the stretched labels of R55/R56, the garment air slot 16 - gets label bit 64, so the
// wound-wall shader can tell padding from flesh (painter_channel_surface.h: a tunnel wall standing in padding with no flesh near it
// is not drawn - the floating wound channels of a shot-up, blown-open body). Bits 0-5 stay the label; every reader masks with 63.
// Lattice: the anatomy brick (b - k) under closure brick b when the origins differ by whole bricks (the baked and the built
// closure), else the anatomy sampled at the cell centre. Page/data layout as buildTissueField (a uniform page: bit 31 | the label;
// palettes of up to 128 values). The bricks go in page order through windows of 4096: pass 1 (parallel(n, body(first, last)))
// fills the window's fixed buffer by brick index - no merge, no sort, the same bytes for every executor - and pass 2 (serial)
// encodes the window in page order, so the scratch is that buffer (2 MiB) whatever the field, and the output grows only through
// encode, which admits each brick against the limit; the padded page table plus the smallest data row is admitted before any
// allocation and the padded result at the end. Not available when the limit is exceeded (the caller keeps the unmarked closure).
// crevice (2026-11-17): `drop` names the cells that get NO padding - drop.brick(bx, by, bz): the closure brick can hold one;
// drop.cell(x, y, z): this closure cell (asked only where the anatomy has no tissue) - such a cell becomes label 0, as outside the
// envelope. xrGame's crevice_padding::Drop: the cells in front of the outfit's own triangles (the padding is built behind them).
template<class Parallel,class Drop> std::shared_ptr<const TissueField> markClosurePadding(const TissueField&closure,const TissueField&anatomy,Parallel parallel,const Drop&drop,std::size_t limit){
    auto out=std::make_shared<TissueField>();auto&v=*out;
    if(!closure.available||!anatomy.available||!(closure.pitch>0)||closure.page.size()<TissueField::headerWords)return out;
    if(closure.page.size()*4+TissueField::dataWidth>limit)return out; // the all-uniform result already exceeds the limit
    const double pitch=closure.pitch;int k[3]={0,0,0};bool aligned=std::abs(double(closure.pitch)-double(anatomy.pitch))<1e-9*double(closure.pitch);
    for(int a=0;a<3&&aligned;++a){const double shift=(double(anatomy.origin[a])-double(closure.origin[a]))/(pitch*8);const double r=std::round(shift);aligned=std::abs(shift-r)<1e-3;k[a]=int(r);}
    v.origin=closure.origin;v.pitch=closure.pitch;v.size=closure.size;v.pages=closure.pages;v.mixed=v.uniform=v.empty=0;
    const unsigned px=v.pages[0],py=v.pages[1],pz=v.pages[2];const std::size_t bricks=std::size_t(px)*py*pz;
    if(!px||!py||!pz||bricks+TissueField::headerWords>closure.page.size())return out;
    auto anatomyAt=[&](int x,int y,int z)->unsigned{
        if(aligned)return anatomy.at(x-k[0]*8,y-k[1]*8,z-k[2]*8);
        return anatomy.sample(double(closure.origin[0])+(x+.5)*pitch,double(closure.origin[1])+(y+.5)*pitch,double(closure.origin[2])+(z+.5)*pitch);};
    // the anatomy brick under closure brick (bx,by,bz): a uniform value (true) or mixed (false)
    auto anatomyUniform=[&](unsigned bx,unsigned by,unsigned bz,unsigned&value)->bool{
        if(!aligned)return false;const int ax=int(bx)-k[0],ay=int(by)-k[1],az=int(bz)-k[2];
        if(ax<0||ay<0||az<0||unsigned(ax)>=anatomy.pages[0]||unsigned(ay)>=anatomy.pages[1]||unsigned(az)>=anatomy.pages[2]){value=0;return true;}
        const std::uint32_t p=anatomy.page[(std::size_t(az)*anatomy.pages[1]+unsigned(ay))*anatomy.pages[0]+unsigned(ax)+TissueField::headerWords];
        if(p&TissueField::uniformBit){value=p&127u;return true;}return false;};
    v.page.assign(closure.page.size(),TissueField::uniformBit);v.data.clear();
    // the same encoding as buildTissueField with 128-value palettes; false = the limit
    auto encode=[&](const std::uint8_t*first,std::size_t p)->bool{const std::uint8_t value=first[0];
        if(std::all_of(first,first+512,[value](std::uint8_t x){return x==value;})){v.page[p]=TissueField::uniformBit|value;value?++v.uniform:++v.empty;return true;}
        std::array<std::uint8_t,128> present{},index{};unsigned count=0;for(unsigned q=0;q<512;++q)present[first[q]&127u]=1;
        for(unsigned s=0;s<128;++s)if(present[s])index[s]=std::uint8_t(count++);
        const unsigned mode=count<=2?0:count<=4?1:count<=16?2:3,bits=1u<<mode,palette=bits==8?0:1u<<bits;
        const std::size_t bytes=palette+512*bits/8,offset=v.data.size();
        // admission against the final padded size: the data rows rounded up (at least one row) plus the page table
        const std::size_t rounded=std::max<std::size_t>(TissueField::dataWidth,((offset+bytes+TissueField::dataWidth-1)/TissueField::dataWidth)*TissueField::dataWidth);
        if(rounded>limit||v.page.size()*4>limit-rounded)return false;
        v.page[p]=(mode<<28)|std::uint32_t(offset);++v.mixed;v.data.resize(offset+bytes,0);
        if(bits==8)std::copy(first,first+512,v.data.begin()+std::ptrdiff_t(offset));
        else{for(unsigned s=0;s<128;++s)if(present[s])v.data[offset+index[s]]=std::uint8_t(s);
            for(unsigned q=0;q<512;++q){const unsigned bit=q*bits;v.data[offset+palette+(bit>>3)]|=std::uint8_t(index[first[q]&127u]<<(bit&7));}}
        return true;};
    constexpr std::size_t window=4096;const std::size_t span=std::min(window,bricks);
    std::vector<std::array<std::uint8_t,512>> cells(span);std::vector<std::uint32_t> word(span); // 0 = per-cell content in cells[]
    for(std::size_t w0=0;w0<bricks;w0+=window){const std::size_t n=std::min(window,bricks-w0);
        parallel(n,[&](std::size_t first,std::size_t last){
            for(std::size_t j=first;j<last;++j){const std::size_t i=w0+j,p=i+TissueField::headerWords;
                const unsigned bx=unsigned(i%px),by=unsigned((i/px)%py),bz=unsigned(i/(std::size_t(px)*py));
                const std::uint32_t cp=closure.page[p];unsigned av=0;const bool au=anatomyUniform(bx,by,bz,av);word[j]=0;
                if(cp&TissueField::uniformBit){const unsigned cv=cp&63u;
                    if(!cv){word[j]=TissueField::uniformBit;continue;}
                    if(au&&(av||!drop.brick(bx,by,bz))){word[j]=TissueField::uniformBit|(av?cv:(cv|64u));continue;}}
                auto&c=cells[j];
                for(unsigned lz=0;lz<8;++lz)for(unsigned ly=0;ly<8;++ly)for(unsigned lx=0;lx<8;++lx){const int x=int(bx*8+lx),y=int(by*8+ly),z=int(bz*8+lz);
                    const unsigned l=closure.at(x,y,z)&63u;c[(lz*8+ly)*8+lx]=std::uint8_t(!l?0u:(au?av:anatomyAt(x,y,z))?l:drop.cell(x,y,z)?0u:(l|64u));}}});
        for(std::size_t j=0;j<n;++j){const std::size_t p=w0+j+TissueField::headerWords;
            if(word[j]){v.page[p]=word[j];(word[j]&127u)?++v.uniform:++v.empty;}
            else if(!encode(cells[j].data(),p))return std::make_shared<TissueField>();}}
    v.data.resize(std::max<std::size_t>(TissueField::dataWidth,((v.data.size()+TissueField::dataWidth-1)/TissueField::dataWidth)*TissueField::dataWidth),0);
    if(v.page.size()*4+v.data.size()>limit)return std::make_shared<TissueField>();
    for(unsigned w=0;w<TissueField::headerWords;++w)v.page[w]=closure.page[w];
    v.available=true;return out;
}
// every cell of the envelope without anatomy is padding (the rule before crevice_padding)
struct AllPadding {bool brick(unsigned,unsigned,unsigned)const{return false;}bool cell(int,int,int)const{return false;}};
template<class Parallel> std::shared_ptr<const TissueField> markClosurePadding(const TissueField&closure,const TissueField&anatomy,Parallel parallel,std::size_t limit=128u<<20){
    return markClosurePadding(closure,anatomy,parallel,AllPadding{},limit);}
inline bool closureAlignment(const TissueField&field,const TissueField&anatomy,const double lo[3],const double hi[3],std::array<unsigned,3>&d){
    if(!(field.pitch>0)||!(std::abs(double(field.pitch)-double(anatomy.pitch))<1e-9))return false;
    for(int a=0;a<3;++a){const double p=double(field.pitch),o=double(field.origin[a]),e=o+double(field.size[a])*p;
        const double shift=(double(anatomy.origin[a])-o)/p;
        if(!(std::abs(shift-std::round(shift))<1e-3&&shift>-.5&&
             e>=double(anatomy.origin[a])+double(anatomy.size[a])*double(anatomy.pitch)-1e-6&&o<=lo[a]&&e>=hi[a]))return false;
        const long s=std::lround(shift);d[a]=unsigned((8-s%8)%8);}
    return true;}
// The field written on that grid: the start moves back d cells per axis (those cells are label 0, as outside the field), the far
// end stays covered, every old cell keeps its label at the same place. Page/data layout as markClosurePadding (128-value
// palettes, bricks in page order through windows of 4096: pass 1 parallel fills the window by brick index, pass 2 encodes it in
// page order - the same bytes for every executor); header words as buildTissueField, 14-15 copied. A new brick is uniform
// without a cell pass when every old cell it covers lies outside the field, or inside old bricks that are uniform with one
// label. Not available on a malformed input or past the limit (the caller then refuses the body as before).
template<class Parallel> std::shared_ptr<TissueField> realignClosureField(const TissueField&f,const std::array<unsigned,3>&d,Parallel parallel,std::size_t limit){
    auto out=std::make_shared<TissueField>();auto&v=*out;
    if(!f.available||!(f.pitch>0)||f.page.size()<TissueField::headerWords)return out;
    const std::size_t oldBricks=std::size_t(f.pages[0])*f.pages[1]*f.pages[2];
    if(!oldBricks||oldBricks+TissueField::headerWords>f.page.size())return out;
    for(int a=0;a<3;++a)if(f.pages[a]!=(f.size[a]+7)/8)return out;
    v.pitch=f.pitch;v.mixed=v.uniform=v.empty=0;
    for(int a=0;a<3;++a){if(d[a]>7)return out;
        v.origin[a]=float(double(f.origin[a])-double(d[a])*double(f.pitch));v.size[a]=f.size[a]+d[a];
        // the float start may round a fraction of a cell forward: one more cell keeps the old far end covered
        if(double(v.origin[a])+double(v.size[a])*double(v.pitch)<double(f.origin[a])+double(f.size[a])*double(f.pitch))++v.size[a];
        if(v.size[a]>=65536)return out;
        v.pages[a]=(v.size[a]+7)/8;}
    const unsigned px=v.pages[0],py=v.pages[1],pz=v.pages[2];const std::size_t bricks=std::size_t(px)*py*pz;
    const std::size_t paddedPages=((bricks+TissueField::headerWords+TissueField::pageWidth-1)/TissueField::pageWidth)*TissueField::pageWidth;
    if(paddedPages*4+TissueField::dataWidth>limit)return out;
    v.page.assign(paddedPages,TissueField::uniformBit);v.data.clear();
    const int dx=int(d[0]),dy=int(d[1]),dz=int(d[2]);const int sx=int(f.size[0]),sy=int(f.size[1]),sz=int(f.size[2]);
    auto oldUniform=[&](unsigned bx,unsigned by,unsigned bz,unsigned&value)->bool{
        const int lo[3]={int(bx*8)-dx,int(by*8)-dy,int(bz*8)-dz},n[3]={sx,sy,sz};
        for(int a=0;a<3;++a)if(lo[a]+7<0||lo[a]>=n[a]){value=0;return true;} // wholly outside the old field
        for(int a=0;a<3;++a)if(lo[a]<0||lo[a]+7>=n[a])return false;     // partly outside: per cell
        bool first=true;
        for(int z=lo[2]>>3;z<=(lo[2]+7)>>3;++z)for(int y=lo[1]>>3;y<=(lo[1]+7)>>3;++y)for(int x=lo[0]>>3;x<=(lo[0]+7)>>3;++x){
            const std::uint32_t p=f.page[(std::size_t(z)*f.pages[1]+unsigned(y))*f.pages[0]+unsigned(x)+TissueField::headerWords];
            if(!(p&TissueField::uniformBit))return false;
            const unsigned l=p&127u;if(first){value=l;first=false;}else if(l!=value)return false;}
        return true;};
    // the same encoding as markClosurePadding; false = the limit
    auto encode=[&](const std::uint8_t*first,std::size_t p)->bool{const std::uint8_t value=first[0];
        if(std::all_of(first,first+512,[value](std::uint8_t x){return x==value;})){v.page[p]=TissueField::uniformBit|value;value?++v.uniform:++v.empty;return true;}
        std::array<std::uint8_t,128> present{},index{};unsigned count=0;for(unsigned q=0;q<512;++q)present[first[q]&127u]=1;
        for(unsigned s=0;s<128;++s)if(present[s])index[s]=std::uint8_t(count++);
        const unsigned mode=count<=2?0:count<=4?1:count<=16?2:3,bits=1u<<mode,palette=bits==8?0:1u<<bits;
        const std::size_t bytes=palette+512*bits/8,offset=v.data.size();
        const std::size_t rounded=std::max<std::size_t>(TissueField::dataWidth,((offset+bytes+TissueField::dataWidth-1)/TissueField::dataWidth)*TissueField::dataWidth);
        if(rounded>limit||v.page.size()*4>limit-rounded)return false;
        v.page[p]=(mode<<28)|std::uint32_t(offset);++v.mixed;v.data.resize(offset+bytes,0);
        if(bits==8)std::copy(first,first+512,v.data.begin()+std::ptrdiff_t(offset));
        else{for(unsigned s=0;s<128;++s)if(present[s])v.data[offset+index[s]]=std::uint8_t(s);
            for(unsigned q=0;q<512;++q){const unsigned bit=q*bits;v.data[offset+palette+(bit>>3)]|=std::uint8_t(index[first[q]&127u]<<(bit&7));}}
        return true;};
    constexpr std::size_t window=4096;const std::size_t span=std::min(window,bricks);
    std::vector<std::array<std::uint8_t,512>> cells(span);std::vector<std::uint32_t> word(span); // 0 = per-cell content in cells[]
    for(std::size_t w0=0;w0<bricks;w0+=window){const std::size_t n=std::min(window,bricks-w0);
        parallel(n,[&](std::size_t first,std::size_t last){
            for(std::size_t j=first;j<last;++j){const std::size_t i=w0+j;
                const unsigned bx=unsigned(i%px),by=unsigned((i/px)%py),bz=unsigned(i/(std::size_t(px)*py));
                unsigned value=0;if(oldUniform(bx,by,bz,value)){word[j]=TissueField::uniformBit|value;continue;}
                word[j]=0;auto&c=cells[j];
                for(unsigned lz=0;lz<8;++lz)for(unsigned ly=0;ly<8;++ly)for(unsigned lx=0;lx<8;++lx)
                    c[(lz*8+ly)*8+lx]=std::uint8_t(f.at(int(bx*8+lx)-dx,int(by*8+ly)-dy,int(bz*8+lz)-dz)&127u);}});
        for(std::size_t j=0;j<n;++j){const std::size_t p=w0+j+TissueField::headerWords;
            if(word[j]){v.page[p]=word[j];(word[j]&127u)?++v.uniform:++v.empty;}
            else if(!encode(cells[j].data(),p))return std::make_shared<TissueField>();}}
    v.data.resize(std::max<std::size_t>(TissueField::dataWidth,((v.data.size()+TissueField::dataWidth-1)/TissueField::dataWidth)*TissueField::dataWidth),0);
    if(v.page.size()*4+v.data.size()>limit)return std::make_shared<TissueField>();
    auto fbits=[](float x){std::uint32_t u;std::memcpy(&u,&x,4);return u;};
    for(int a=0;a<3;++a){v.page[a]=fbits(v.origin[a]);v.page[4+a]=v.size[a];v.page[8+a]=v.pages[a];}
    v.page[3]=fbits(1.f/v.pitch);v.page[7]=TissueField::brickShift;v.page[11]=TissueField::dataWidth;v.page[12]=TissueField::pageWidth;v.page[13]=TissueField::headerWords;
    v.page[14]=f.page[14];v.page[15]=f.page[15];
    v.available=true;return out;}
} // namespace painter_detail
