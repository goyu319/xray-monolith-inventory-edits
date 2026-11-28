#pragma once
#ifdef __AVX__
// Four consecutive Z cells share an XY lattice row. When their Z corners
// straddle a lattice boundary, the scalar path supplies those exact four
// octave samples. Both routes keep the native interpolation operation order.
inline void craterOctave4(CraterNoiseRow::Slice& slice,const head_burst_record::Octave* o,
                         double x,double y,const double z[4],__m256d r[3]){
    using Oct=head_burst_record::Octave;
    const int a=Oct::ffloor(z[0]*o->k),b=Oct::ffloor(z[3]*o->k);
    double single[3];slice.at(o,{x,y,z[0]},single);
    if(a==b&&slice.cached&&slice.px==x&&slice.py==y&&slice.iz==a){
        const auto coordinate=_mm256_mul_pd(_mm256_loadu_pd(z),_mm256_set1_pd(o->k));
        const auto f=_mm256_sub_pd(coordinate,_mm256_set1_pd(double(a)));
        const auto t=_mm256_mul_pd(_mm256_mul_pd(f,f),_mm256_sub_pd(_mm256_set1_pd(3.),_mm256_mul_pd(_mm256_set1_pd(2.),f)));
        for(int j=0;j<3;++j)r[j]=_mm256_add_pd(_mm256_set1_pd(slice.bottom[j]),_mm256_mul_pd(_mm256_set1_pd(slice.top[j]-slice.bottom[j]),t));
    }else{
        double side[3][4];
        for(int j=0;j<3;++j)side[j][0]=single[j];
        for(int k=1;k<4;++k){
            slice.at(o,{x,y,z[k]},single);
            for(int j=0;j<3;++j)side[j][k]=single[j];
        }
        for(int j=0;j<3;++j)r[j]=_mm256_loadu_pd(side[j]);
    }
}
#endif

inline unsigned stockingFour(CraterMargin& field,const Grid& g,const head_burst_record::Function& f,CraterNoiseRow& row,
                            Cell start,std::size_t tile,unsigned active){
#ifdef __AVX__
    if(field.nearFirst&&(active&(active-1))){
        const Vec base=g.center(start);double z[4];
        for(int j=0;j<4;++j)z[j]=g.origin.z+(start.z+j+.5)*g.pitch;
        const auto zp=_mm256_loadu_pd(z);
        const double dx=base.x-f.B[0],dy=base.y-f.B[1];
        const auto dz=_mm256_sub_pd(zp,_mm256_set1_pd(f.B[2]));
        const auto r2=_mm256_add_pd(_mm256_set1_pd(dx*dx+dy*dy),_mm256_mul_pd(dz,dz));
        active&=_mm256_movemask_pd(_mm256_cmp_pd(r2,_mm256_set1_pd(f.Rcut*f.Rcut),_CMP_LE_OQ));
        if(!active)return 0;
        __m256d lo[3],hi[3],p[3]={_mm256_set1_pd(base.x),_mm256_set1_pd(base.y),zp},w[3];
        craterOctave4(row.octave[0],f.o,base.x,base.y,z,lo);craterOctave4(row.octave[1],f.o+3,base.x,base.y,z,hi);
        for(int j=0;j<3;++j)w[j]=_mm256_add_pd(_mm256_add_pd(p[j],_mm256_mul_pd(_mm256_set1_pd(f.A1),lo[j])),_mm256_mul_pd(_mm256_set1_pd(f.A2),hi[j]));
        auto margin=_mm256_set1_pd(-1e8);
        for(const auto& group:field.planes.groups){
            auto nearest=_mm256_set1_pd(1e8);
            for(unsigned i=0;i<group.size;++i){
                const auto xy=_mm256_add_pd(_mm256_mul_pd(w[0],_mm256_set1_pd(group.x[i])),_mm256_mul_pd(w[1],_mm256_set1_pd(group.y[i])));
                const auto m=_mm256_add_pd(_mm256_add_pd(xy,_mm256_mul_pd(w[2],_mm256_set1_pd(group.z[i]))),_mm256_set1_pd(group.offset[i]));
                nearest=_mm256_min_pd(nearest,m);
            }
            margin=_mm256_max_pd(margin,nearest);
        }
        double values[4];_mm256_storeu_pd(values,margin);
        if(!field.pages.empty()){
            auto& page=field.pages[tile];if(!page)page.reset(new CraterMargin::Page);
            const unsigned bit=Grid::local(start);page->valid[bit>>6]|=std::uint64_t(active)<<(bit&63);
            _mm256_storeu_pd(page->value+bit,margin);
        }
        unsigned result=active&_mm256_movemask_pd(_mm256_cmp_pd(margin,_mm256_set1_pd(field.boundaryGuard),_CMP_GT_OQ));
        unsigned examine=active&~result&unsigned(_mm256_movemask_pd(_mm256_cmp_pd(margin,_mm256_set1_pd(-field.boundaryGuard),_CMP_GE_OQ)));
        while(examine){const unsigned j=head_burst::ctz64(examine);examine&=examine-1;
            const unsigned value=f.label(base.x,base.y,z[j]);if(value&&value<253)result|=1u<<j;}
        return result;
    }
#endif
    unsigned result=0;
    while(active){
        const unsigned j=head_burst::ctz64(active);active&=active-1;const Cell c{start.x,start.y,start.z+int(j)};
        const auto value=field.label(f,row,g.center(c),c,tile);if(value&&value<253)result|=1u<<j;
    }
    return result;
}
