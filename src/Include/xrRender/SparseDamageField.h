#pragma once
// Immutable per-publication removal mask. This is separate from source anatomy:
// zero keeps source skin, one removes it. The bit layout matches Grid::Tile.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>

namespace painter_detail {
struct DamageField {
    static constexpr std::uint32_t headerWords=16,pageWidth=1024,dataWidth=4096;
    static constexpr std::uint32_t keptPage=0,removedPage=0xffffffffu;
    std::array<float,3> origin{};
    float pitch{};
    std::array<std::uint32_t,3> size{},pages{};
    std::vector<std::uint32_t> page,data;
    std::uint32_t mixed{},uniform{};
    bool available{};
    std::size_t bytes()const noexcept{return 4*(page.size()+data.size());}
    bool at(int x,int y,int z)const noexcept {
        if(!available||x<0||y<0||z<0||unsigned(x)>=size[0]||unsigned(y)>=size[1]||unsigned(z)>=size[2])return false;
        const std::size_t i=(std::size_t(z>>3)*pages[1]+unsigned(y>>3))*pages[0]+unsigned(x>>3)+headerWords;
        const auto p=page[i];
        if(p==keptPage)return false;
        if(p==removedPage)return true;
        const unsigned bit=((unsigned(x)&7u)*8u+(unsigned(y)&7u))*8u+(unsigned(z)&7u);
        return ((data[std::size_t(p-1)+(bit>>5)]>>(bit&31u))&1u)!=0;
    }
    bool sample(double x,double y,double z)const noexcept {
        if(!available||!size[0]||!size[1]||!size[2]||!(pitch>0))return false;
        const double a=std::floor((x-origin[0])/pitch),b=std::floor((y-origin[1])/pitch),c=std::floor((z-origin[2])/pitch);
        if(!(a>=0&&b>=0&&c>=0&&a<size[0]&&b<size[1]&&c<size[2]))return false;
        return at(int(a),int(b),int(c));
    }
};

// The existing occupancy page and its data words retain their old addressing.
// Each occupied group of 32 tiles has one leaf; a mixed tag tile has 16 words.
// Missing metadata always selects the full material. No changes to gameplay data.
template<class Grid> std::shared_ptr<const DamageField> publishBlowoutTags(
    const Grid& grid,std::shared_ptr<const DamageField> original,std::size_t limit=16u<<20){
    if(!original||!original->available||!grid.lodAttached||!grid.blowoutTags||
       grid.blowoutTags->empty()||!original->size[0]||original->page[15]!=0)return original;
    const auto& f=*original;
    std::array<int,3> corner{};
    for(unsigned a=0;a<3;++a)
        corner[a]=int(std::llround((double(f.origin[a])-grid.origin[a])/(8.*grid.pitch)));
    struct TagTile{std::uint32_t index;std::array<std::uint32_t,16> bits;bool full;};
    std::vector<TagTile> tags;tags.reserve(grid.blowoutTags->size());
    for(const auto& item:*grid.blowoutTags){
        const auto& c=item.first;const int x=c.x-corner[0],y=c.y-corner[1],z=c.z-corner[2];
        if(x<0||y<0||z<0||unsigned(x)>=f.pages[0]||unsigned(y)>=f.pages[1]||unsigned(z)>=f.pages[2])continue;
        TagTile tile{};tile.index=(unsigned(z)*f.pages[1]+unsigned(y))*f.pages[0]+unsigned(x);
        const auto p=f.page[DamageField::headerWords+tile.index];
        if(p==DamageField::keptPage)continue;
        std::uint32_t any=0,all=~0u;
        for(unsigned w=0;w<16;++w){
            const auto raw=std::uint32_t(item.second.removed[w/2]>>(32*(w&1)));
            tile.bits[w]=p==DamageField::removedPage?raw:raw&f.data[p-1+w];
            any|=tile.bits[w];all&=tile.bits[w];
        }
        if(any){tile.full=all==~0u;tags.push_back(tile);}
    }
    if(tags.empty())return original;
    std::sort(tags.begin(),tags.end(),[](const TagTile& a,const TagTile& b){return a.index<b.index;});
    const std::size_t pages=std::size_t(f.pages[0])*f.pages[1]*f.pages[2];
    const std::size_t dirs=(pages+31)/32;
    const std::size_t start=std::size_t(f.mixed)*16;
    std::size_t end=start+dirs;std::uint32_t group=~0u;
    for(const auto& t:tags){
        if(group!=(t.index>>5)){group=t.index>>5;end+=32;}
        if(!t.full)end+=16;
    }
    const auto padded=(end+DamageField::dataWidth-1)/DamageField::dataWidth*DamageField::dataWidth;
    if(padded>limit/4||f.page.size()>limit/4-padded)return original;
    // A freshly built field has one owner. A shared field may still be drawn by a
    // previous publication; never write to it.
    auto out=original.use_count()==1?std::const_pointer_cast<DamageField>(original):std::make_shared<DamageField>(f);
    out->data.resize(std::max(out->data.size(),padded),0);
    std::fill(out->data.begin()+start,out->data.begin()+end,0);
    std::uint32_t cursor=std::uint32_t(start+dirs),leaf=0;group=~0u;
    for(const auto& t:tags){
        if(group!=(t.index>>5)){
            group=t.index>>5;leaf=cursor;cursor+=32;
            out->data[start+group]=leaf+1;
        }
        if(t.full)out->data[leaf+(t.index&31)]=~0u;
        else{
            out->data[leaf+(t.index&31)]=cursor+1;
            std::copy(t.bits.begin(),t.bits.end(),out->data.begin()+cursor);cursor+=16;
        }
    }
    out->page[14]=std::uint32_t(start+1);
    out->page[15]=0x424c4431u;
    return out;
}

// Grid-compatible template keeps this header usable by the worker and renderer.
// Owned severed components require their complete ownership predicate, not just
// the journal bits. Until that format is implemented they keep the geometry path.
template<class Grid> std::shared_ptr<const DamageField> buildDamageField(const Grid& grid,std::size_t limit=16u<<20) {
    auto out=std::make_shared<DamageField>();
    if(grid.ownership||!(grid.pitch>0)||!std::isfinite(grid.pitch))return out;
    std::array<int,3> lo{INT32_MAX,INT32_MAX,INT32_MAX},hi{INT32_MIN,INT32_MIN,INT32_MIN};
    for(const auto& item:grid.tiles){
        bool any=false;for(auto w:item.second.removed)any|=w!=0;
        if(!any)continue;
        const int c[3]={item.first.x,item.first.y,item.first.z};
        for(int a=0;a<3;++a){lo[a]=std::min(lo[a],c[a]);hi[a]=std::max(hi[a],c[a]);}
    }
    out->pitch=float(grid.pitch);
    if(lo[0]==INT32_MAX){
        if(limit<(DamageField::pageWidth+DamageField::dataWidth)*4)return out;
        out->page.assign(DamageField::pageWidth,0);out->data.assign(DamageField::dataWidth,0);
        out->available=true;return out;
    }
    std::size_t count=1;
    const int dimensions[3]={grid.size.x,grid.size.y,grid.size.z};
    for(int a=0;a<3;++a){
        if(lo[a]<0||hi[a]>=((dimensions[a]+7)/8))return out;
        const std::uint64_t n=std::uint64_t(hi[a])-lo[a]+1;
        if(n>UINT32_MAX||count>limit/4/n)return out;
        count*=std::size_t(n);out->pages[a]=std::uint32_t(n);
        out->size[a]=std::uint32_t(std::min<std::int64_t>(std::int64_t(n)*8,std::int64_t(dimensions[a])-std::int64_t(lo[a])*8));
        out->origin[a]=float(grid.origin[a]+double(lo[a])*8*grid.pitch);
    }
    const std::size_t padded=((count+DamageField::headerWords+DamageField::pageWidth-1)/DamageField::pageWidth)*DamageField::pageWidth;
    if(padded>limit/4||DamageField::dataWidth>limit/4-padded)return out;
    out->page.assign(padded,0);
    // First pass records only which pages are mixed/full. Prefix allocation in
    // page order makes bytes deterministic regardless of hash-map traversal.
    for(const auto& item:grid.tiles){
        const int c[3]={item.first.x,item.first.y,item.first.z};bool inside=true;
        for(int a=0;a<3;++a)inside&=c[a]>=lo[a]&&c[a]<=hi[a];
        if(!inside)continue;
        std::uint64_t any=0,all=~std::uint64_t(0);for(auto w:item.second.removed){any|=w;all&=w;}
        if(!any)continue;
        const std::size_t i=(std::size_t(c[2]-lo[2])*out->pages[1]+unsigned(c[1]-lo[1]))*out->pages[0]+unsigned(c[0]-lo[0])+DamageField::headerWords;
        if(all==~std::uint64_t(0)){out->page[i]=DamageField::removedPage;++out->uniform;}
        else{out->page[i]=1;++out->mixed;}
    }
    const std::size_t words=std::size_t(out->mixed)*16;
    const std::size_t paddedData=std::max<std::size_t>(DamageField::dataWidth,((words+DamageField::dataWidth-1)/DamageField::dataWidth)*DamageField::dataWidth);
    if(words>=DamageField::removedPage||paddedData>limit/4-padded){out->page.clear();return out;}
    out->data.assign(paddedData,0);std::uint32_t cursor=0;
    for(std::size_t i=DamageField::headerWords;i<DamageField::headerWords+count;++i)
        if(out->page[i]==1){out->page[i]=cursor+1;cursor+=16;}
    for(const auto& item:grid.tiles){
        const int c[3]={item.first.x,item.first.y,item.first.z};bool inside=true;
        for(int a=0;a<3;++a)inside&=c[a]>=lo[a]&&c[a]<=hi[a];
        if(!inside)continue;
        const std::size_t i=(std::size_t(c[2]-lo[2])*out->pages[1]+unsigned(c[1]-lo[1]))*out->pages[0]+unsigned(c[0]-lo[0])+DamageField::headerWords;
        const auto p=out->page[i];if(p==DamageField::keptPage||p==DamageField::removedPage)continue;
        for(unsigned x=0;x<8;++x){const auto w=item.second.removed[x];out->data[p-1+2*x]=std::uint32_t(w);out->data[p+2*x]=std::uint32_t(w>>32);}
    }
    auto fbits=[](float f){std::uint32_t u;std::memcpy(&u,&f,4);return u;};
    for(int a=0;a<3;++a){out->page[a]=fbits(out->origin[a]);out->page[4+a]=out->size[a];out->page[8+a]=out->pages[a];}
    out->page[3]=fbits(1.f/out->pitch);out->page[7]=3;out->page[11]=DamageField::dataWidth;out->page[12]=DamageField::pageWidth;out->page[13]=DamageField::headerWords;
    out->available=true;return out;
}
} // namespace painter_detail
