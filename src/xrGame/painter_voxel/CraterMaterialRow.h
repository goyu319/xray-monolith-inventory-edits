#pragma once

// Damage cells arrive in X/Y/Z order. Step the ORIGINAL sparse Z runs for
// each original anatomy column, including matches through gaps, instead of
// binary searching the column for every one of its successive damage cells.
struct CraterMaterialRow {
    int oldX=INT_MIN,oldY=INT_MIN,oldZ=INT_MIN;
    std::size_t run{},end{};
    unsigned get(const Grid& g,Cell c){
        if(!g.anatomy||g.ownership)return g.materialBase(c);
        const auto& a=*g.anatomy;const Vec p=g.center(c);
        const int x=int(std::floor((p.x-a.origin[0])/a.pitch)),y=int(std::floor((p.y-a.origin[1])/a.pitch)),
                  z=int(std::floor((p.z-a.origin[2])/a.pitch));
        if(x<0||y<0||z<0||unsigned(x)>=a.size[0]||unsigned(y)>=a.size[1]||unsigned(z)>=a.size[2])return 0;
        if(x!=oldX||y!=oldY||z<oldZ){
            const auto key=std::size_t(y)*a.size[0]+unsigned(x);
            run=a.offsets[key];end=a.offsets[key+1];
            if(run<end&&a.runs[run].hi<=z){
                run=std::size_t(std::upper_bound(a.runs.begin()+run,a.runs.begin()+end,z,
                    [](int value,const auto& r){return value<r.hi;})-a.runs.begin());
            }
            oldX=x;oldY=y;
        }
        oldZ=z;
        while(run<end&&a.runs[run].hi<=z)++run;
        return run<end&&a.runs[run].lo<=z?a.runs[run].label:0;
    }
};
