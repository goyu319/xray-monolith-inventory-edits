#pragma once

// Native double half-planes, four sites per vector. The sums are still
// ((x*nx+y*ny)+z*nz)+offset. Empty lanes cannot change any reduction.
struct CraterPlanes {
    static constexpr unsigned capacity=head_burst_record::maxSites;
    struct Group {
        double x[16]{},y[16]{},z[16]{},offset[16]{};
        double ax[16]{},ay[16]{},az[16]{},pad[16]{};
        unsigned size{};
        Group(){for(auto& k:offset)k=1e30;}
        void add(Vec n,double d,double error){
            const unsigned i=size++;x[i]=n.x;y[i]=n.y;z[i]=n.z;offset[i]=d;
            ax[i]=std::abs(n.x);ay[i]=std::abs(n.y);az[i]=std::abs(n.z);pad[i]=error;
        }
#ifdef __AVX__
        static __m256d linear(const double* x,const double* y,const double* z,const double* d,
                              __m256d px,__m256d py,__m256d pz){
            return _mm256_add_pd(_mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(px,_mm256_loadu_pd(x)),
                _mm256_mul_pd(py,_mm256_loadu_pd(y))),_mm256_mul_pd(pz,_mm256_loadu_pd(z))),_mm256_loadu_pd(d));
        }
#endif
        double margin(Vec w)const{
#ifdef __AVX__
            auto result=_mm256_set1_pd(1e8);
            const auto px=_mm256_set1_pd(w.x),py=_mm256_set1_pd(w.y),pz=_mm256_set1_pd(w.z);
            for(unsigned i=0;i<size;i+=4)
                result=_mm256_min_pd(result,linear(x+i,y+i,z+i,offset+i,px,py,pz));
            double v[4];_mm256_storeu_pd(v,result);
            return std::min(std::min(v[0],v[1]),std::min(v[2],v[3]));
#else
            double r=1e8;for(unsigned i=0;i<size;++i)r=std::min(r,x[i]*w.x+y[i]*w.y+z[i]*w.z+offset[i]);return r;
#endif
        }
        // +1 proves a fully included region, -1 a disjoint one, 0 is uncertain.
        int box(Vec p,Vec h)const{
            bool inside=true;
#ifdef __AVX__
            const auto px=_mm256_set1_pd(p.x),py=_mm256_set1_pd(p.y),pz=_mm256_set1_pd(p.z);
            const auto hx=_mm256_set1_pd(h.x),hy=_mm256_set1_pd(h.y),hz=_mm256_set1_pd(h.z);
            const auto zero=_mm256_setzero_pd();
            for(unsigned i=0;i<size;i+=4){
                const auto m=linear(x+i,y+i,z+i,offset+i,px,py,pz);
                const auto e=linear(ax+i,ay+i,az+i,pad+i,hx,hy,hz);
                if(_mm256_movemask_pd(_mm256_cmp_pd(_mm256_add_pd(m,e),zero,_CMP_LT_OQ)))return -1;
                if(_mm256_movemask_pd(_mm256_cmp_pd(_mm256_sub_pd(m,e),zero,_CMP_LE_OQ)))inside=false;
            }
#else
            for(unsigned i=0;i<size;++i){
                const double m=x[i]*p.x+y[i]*p.y+z[i]*p.z+offset[i];
                const double e=pad[i]+(ax[i]*h.x+ay[i]*h.y+az[i]*h.z);
                if(m+e<0)return -1;inside=inside&&(m-e>0);
            }
#endif
            return inside?1:0;
        }
    };
    std::vector<Group> groups;
    double at(Vec w)const{
        double r=-1e8;
        for(const auto& group:groups)r=std::max(r,group.margin(w));
        return r;
    }
};
