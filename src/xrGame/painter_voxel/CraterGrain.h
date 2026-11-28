#pragma once

// The three V5 relief fields revisit the same lattice values for thousands of
// nearby points. Store only those exact corners covering this hit. Queries
// outside the small window still evaluate the original integer corner hash.
struct CraterGrain {
    int x0{}, y0{}, nx{}, ny{};
    std::uint64_t seed{};
    std::vector<double> values;

    static double corner(std::uint64_t seed,int x,int y){
        return double(core::quantMix(seed^(std::uint64_t(x)*0xc2b2ae3d27d4eb4full)^
                                          (std::uint64_t(y)*0x9e3779b97f4a7c15ull))>>11)*(2./9007199254740992.)-1.;
    }
    void initialize(double xMin,double yMin,double xMax,double yMax,std::uint64_t s){
        seed=s;
        x0=int(std::floor(xMin))-1;y0=int(std::floor(yMin))-1;
        nx=int(std::floor(xMax))-x0+3;ny=int(std::floor(yMax))-y0+3;
        if(nx<=0||ny<=0||nx>256||ny>256)return;
        values.resize(std::size_t(nx)*ny);
        for(int j=0;j<ny;++j)for(int i=0;i<nx;++i)values[std::size_t(j)*nx+i]=corner(s,x0+i,y0+j);
    }
    double at(double x,double y)const{
        const int ix=int(std::floor(x)),iy=int(std::floor(y));
        const double fx=x-ix,fy=y-iy,s=fx*fx*(3.-2.*fx),t=fy*fy*(3.-2.*fy);
        const int i=ix-x0,j=iy-y0;
        double c00,c10,c01,c11;
        if(!values.empty()&&i>=0&&j>=0&&i+1<nx&&j+1<ny){
            const double* p=values.data()+std::size_t(j)*nx+i;
            c00=p[0];c10=p[1];c01=p[nx];c11=p[nx+1];
        }else{
            c00=corner(seed,ix,iy);c10=corner(seed,ix+1,iy);c01=corner(seed,ix,iy+1);c11=corner(seed,ix+1,iy+1);
        }
        return (c00*(1.-s)+c10*s)*(1.-t)+(c01*(1.-s)+c11*s)*t;
    }
};

// Stock enumeration is unique by tile and lane; the V5 final selection is
// unique by ranked cell and its connectivity flag. Enumerate each occupied
// Z bit, in exactly Cell::operator< order. Retain the general sort for sparse
// bounds and as a defensive fallback if an input contains duplicates.
inline void orderUniqueCraterCells(std::vector<Cell>& cells){
    if(cells.size()<256){sortCraterCells(cells);return;}
    Cell lo=cells.front(),hi=lo;
    for(Cell c:cells)for(int a=0;a<3;++a){(&lo.x)[a]=std::min(lo[a],c[a]);(&hi.x)[a]=std::max(hi[a],c[a]);}
    const auto nx=std::size_t(hi.x-lo.x+1),ny=std::size_t(hi.y-lo.y+1),nz=std::size_t(hi.z-lo.z+64)/64;
    const std::size_t budget=std::min<std::size_t>(cells.size(),262144);
    if(nx>budget/ny||nx*ny>budget/nz){sortCraterCells(cells);return;}
    std::vector<std::uint64_t> bits(nx*ny*nz,0);
    for(Cell c:cells){
        const unsigned z=c.z-lo.z;
        auto& lane=bits[(std::size_t(c.x-lo.x)*ny+std::size_t(c.y-lo.y))*nz+(z>>6)];
        const auto flag=std::uint64_t(1)<<(z&63);
        if(lane&flag){sortCraterCells(cells);return;}
        lane|=flag;
    }
    std::size_t j=0,at=0;
    for(int x=lo.x;x<=hi.x;++x)for(int y=lo.y;y<=hi.y;++y)for(std::size_t w=0;w<nz;++w)
        for(auto z=bits[at++];z;z&=z-1)cells[j++]={x,y,lo.z+int(w*64)+int(head_burst::ctz64(z))};
}
