#pragma once

// An immutable tile-address table is shared by the two stock/relief passes.
// Each stock worker writes only its own tiles. Fully classified regions
// need no storage. The fallback is exact for oversized/sparse cuts.
struct CraterMargin {
    struct Page{std::uint64_t valid[8]{};double value[512];};
    Cell lo{},shape{};
    std::vector<int> ids;
    std::vector<std::unique_ptr<Page>> pages;
    CraterPlanes planes;
    bool nearFirst=true;
    double boundaryGuard=1e30;
    CraterMargin(const head_burst_record::Function& fn,const std::vector<Cell>& tiles,const int low[3],const int high[3]){
        bool far=false;double minLen=1e30,scale=1.;
        for(unsigned i=0;i<fn.S;++i){
            if(!fn.group[i])far=true;
            else if(far||fn.group[i]>=253)nearFirst=false;
            for(double x:fn.site[i])scale=std::max(scale,std::abs(x));
        }
        for(unsigned i=0;i<fn.S;++i)if(fn.group[i]){
            CraterPlanes::Group group;
            for(unsigned j=0;j<fn.S;++j)if(!fn.group[j]){
                const Vec a{fn.site[i][0],fn.site[i][1],fn.site[i][2]},b{fn.site[j][0],fn.site[j][1],fn.site[j][2]};
                const Vec delta=a-b;const double l=length(delta);
                if(l<=1e-9){nearFirst=false;continue;}
                minLen=std::min(minLen,l);const auto n=delta*(1./l);
                group.add(n,-dot(n,(a+b)*.5),0);
            }
            if(group.size)planes.groups.push_back(group);
        }
        nearFirst=nearFirst&&!planes.groups.empty()&&far;
        // In exact arithmetic the sign of the best bisector is the nearest
        // group. Near a tie (including nonordinary inputs), run the native
        // squared-distance test. This bound covers roundoff in both forms.
        for(double x:fn.B)scale=std::max(scale,std::abs(x));
        scale+=fn.Rcut+std::abs(fn.A1)+std::abs(fn.A2);
        boundaryGuard=1024.*std::numeric_limits<double>::epsilon()*(scale+scale*scale/minLen);
        lo={low[0],low[1],low[2]};shape={high[0]-low[0]+1,high[1]-low[1]+1,high[2]-low[2]+1};
        const std::size_t limit=16u<<20;
        if(tiles.size()>limit/sizeof(Page)||std::size_t(shape.x)>limit/sizeof(int)/shape.y||
           std::size_t(shape.x)*shape.y>limit/sizeof(int)/shape.z)return;
        ids.assign(std::size_t(shape.x)*shape.y*shape.z,-1);pages.resize(tiles.size());
        for(std::size_t i=0;i<tiles.size();++i)ids[index(tiles[i])]=int(i);
    }
    std::size_t index(Cell k)const{return (std::size_t(k.x-lo.x)*shape.y+unsigned(k.y-lo.y))*shape.z+unsigned(k.z-lo.z);}
    const Page* get(Cell k)const{
        if(ids.empty()||unsigned(k.x-lo.x)>=unsigned(shape.x)||unsigned(k.y-lo.y)>=unsigned(shape.y)||unsigned(k.z-lo.z)>=unsigned(shape.z))return nullptr;
        const int i=ids[index(k)];return i<0?nullptr:pages[i].get();
    }
    unsigned label(const head_burst_record::Function& fn,CraterNoiseRow& row,const Vec& p,Cell c,std::size_t tile){
        const Vec d=p-Vec{fn.B[0],fn.B[1],fn.B[2]};if(dot(d,d)>fn.Rcut*fn.Rcut)return 0;
        double q[3];row.warp(fn,p,q);
        const double margin=planes.at({q[0],q[1],q[2]});
        if(!pages.empty()){
            auto& page=pages[tile];if(!page)page.reset(new Page);
            const auto bit=Grid::local(c);
            page->valid[bit>>6]|=std::uint64_t(1)<<(bit&63);page->value[bit]=margin;
        }
        if(nearFirst&&std::abs(margin)>boundaryGuard)return margin>0?1:0;
        double best=1e30;unsigned result=0;
        for(unsigned i=0;i<fn.S;++i){
            const double a=q[0]-fn.site[i][0],b=q[1]-fn.site[i][1],c_=q[2]-fn.site[i][2],square=a*a+b*b+c_*c_;
            if(square<best){best=square;result=fn.group[i];}
        }
        return result;
    }
};
