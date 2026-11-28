#pragma once

// A shell cell is removed only if one of the seven original inward samples
// hits fresh tissue. Reject four-cell blocks that cannot reach ANY fresh cell
// even when allowing a surface point anywhere in the full .92-pitch radius.
struct CraterBackedBlocks {
    Cell lo{},hi{},shape{};
    std::vector<unsigned char> mask;
    static Cell block(Cell c){return {c.x>>2,c.y>>2,c.z>>2};}
    bool inside(Cell c)const{return unsigned(c.x-lo.x)<unsigned(shape.x)&&unsigned(c.y-lo.y)<unsigned(shape.y)&&unsigned(c.z-lo.z)<unsigned(shape.z);}
    std::size_t index(Cell c)const{return (std::size_t(c.x-lo.x)*shape.y+unsigned(c.y-lo.y))*shape.z+unsigned(c.z-lo.z);}
    bool possible(Cell blockKey)const{return mask.empty()||(inside(blockKey)&&(mask[index(blockKey)]&2)!=0);}
    void initialize(const std::vector<Cell>& fresh,const LocalCraterCells& bounds,const Vec& dir){
        if(fresh.empty())return;
        lo=block(bounds.lo);hi=block(bounds.hi);
        // Six pitch steps at most, plus a one-pitch surface tolerance.
        for(int a=0;a<3;++a){(&lo.x)[a]-=3;(&hi.x)[a]+=3;(&shape.x)[a]=hi[a]-lo[a]+1;}
        const std::size_t cap=512u<<10;
        if(std::size_t(shape.x)>cap/std::size_t(shape.y)||
           std::size_t(shape.x)*shape.y>cap/std::size_t(shape.z))return;
        mask.assign(std::size_t(shape.x)*shape.y*shape.z,0);
        std::vector<Cell> support;
        for(Cell c:fresh){Cell b=block(c);auto& f=mask[index(b)];if(!f){f=1;support.push_back(b);}}
        std::set<Cell> offsets;
        for(int j=0;j<=6;++j){
            Cell minimum,maximum;
            for(int a=0;a<3;++a){
                (&minimum.x)[a]=int(std::floor((0.5-.920001+j*dir[a])*.25));
                (&maximum.x)[a]=int(std::floor((3.5+.920001+j*dir[a])*.25));
            }
            for(int x=minimum.x;x<=maximum.x;++x)for(int y=minimum.y;y<=maximum.y;++y)
                for(int z=minimum.z;z<=maximum.z;++z)offsets.insert({x,y,z});
        }
        for(Cell b:support)for(Cell a:offsets){
            const Cell k{b.x-a.x,b.y-a.y,b.z-a.z};
            if(inside(k))mask[index(k)]|=2;
        }
    }
};
