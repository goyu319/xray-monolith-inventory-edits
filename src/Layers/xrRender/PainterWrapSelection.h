#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace PainterWrapSelection
{
struct Row
{
    float x,y,z, rx,rz, low, height, fade;
    unsigned id;
};
class Selection
{
    struct Entry{int x,y,z,ref,next;};
    std::array<int,521> heads;
    std::vector<Entry> entries;
    std::vector<unsigned> order;
    static unsigned hash(int x,int y,int z)
    {
        return (uint32_t(x)*73856093u^uint32_t(y)*19349663u^uint32_t(z)*83492791u)%521u;
    }
    static bool sameSupport(const Row& a,const Row& b)
    {
        const float minRadius=std::min(std::min(a.rx,a.rz),std::min(b.rx,b.rz));
        const float horizontal=.035f+.32f*minRadius;
        const float dx=a.x-b.x, dz=a.z-b.z;
        if(dx*dx+dz*dz>horizontal*horizontal ||
           std::abs(a.rx-b.rx)+std::abs(a.rz-b.rz)>.058f+.58f*minRadius) return false;
        const float overlap=std::min(a.low+a.height,b.low+b.height)-std::max(a.low,b.low);
        // Keep an independent row when the retained flame reaches the other contact
        // only at its ragged foot or tip.
        const float contactHeight=(a.y-b.low)/std::max(b.height,.001f);
        return contactHeight>=.09f && contactHeight<=.68f &&
               std::abs(a.y-b.y)<=.97f && overlap>.64f*std::min(a.height,b.height);
    }
public:
    // Reuse scratch. Rank from stable field state so camera order cannot choose a different cell.
    void filter(const std::vector<Row>& rows,std::vector<unsigned char>& keep)
    {
        keep.assign(rows.size(),1);order.clear();entries.clear();heads.fill(-1);
        for(unsigned i=0;i<rows.size();++i) if(rows[i].height>0.f)order.push_back(i);
        std::sort(order.begin(),order.end(),[&](unsigned i,unsigned j){
            const float a=rows[i].fade*rows[i].height,b=rows[j].fade*rows[j].height;
            return a!=b ? a>b : rows[i].id<rows[j].id;
        });
        for(unsigned i:order){
            const Row& r=rows[i];
            const int x=int(std::floor(r.x*2.f)),y=int(std::floor(r.y)),z=int(std::floor(r.z*2.f));
            bool duplicate=false;
            for(int dz=-1;dz<=1&&!duplicate;++dz)
            for(int dy=-1;dy<=1&&!duplicate;++dy)
            for(int dx=-1;dx<=1&&!duplicate;++dx){
                const int ax=x+dx,ay=y+dy,az=z+dz;
                for(int k=heads[hash(ax,ay,az)];k>=0;k=entries[k].next){
                    const auto& e=entries[k];if(e.x==ax&&e.y==ay&&e.z==az&&sameSupport(r,rows[e.ref])){duplicate=true;break;}
                }
            }
            if(duplicate){keep[i]=0;continue;}
            const unsigned h=hash(x,y,z);entries.push_back({x,y,z,int(i),heads[h]});heads[h]=int(entries.size()-1);
        }
    }
};
}
