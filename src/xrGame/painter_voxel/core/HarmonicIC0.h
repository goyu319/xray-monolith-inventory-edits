#pragma once
// Included inside the isolated kernel namespace. No engine dependencies.
// A bounded IC(0) factor of the harmonic free-vertex graph Laplacian.
// A itself, the Dirichlet data and convergence tests are never modified.
class HarmonicIC0 {
    std::vector<std::size_t> first_,column_;
    std::vector<double> inverseDiagonal_,lower_;
    inline static std::atomic<std::size_t> reservedTotal_{0};
    static constexpr std::size_t globalLimit_=16u*1024u*1024u;
    std::size_t reserved_{};
public:
    HarmonicIC0()=default;
    ~HarmonicIC0(){clear();}
    HarmonicIC0(const HarmonicIC0&)=delete;
    HarmonicIC0& operator=(const HarmonicIC0&)=delete;
    HarmonicIC0(HarmonicIC0&&)=delete;
    HarmonicIC0& operator=(HarmonicIC0&&)=delete;
    static std::size_t reserved_bytes() noexcept {return reservedTotal_.load(std::memory_order_relaxed);}
    void clear() noexcept {
        std::vector<std::size_t>().swap(first_);std::vector<std::size_t>().swap(column_);
        std::vector<double>().swap(inverseDiagonal_);std::vector<double>().swap(lower_);
        // Free storage before making its reservation available to another job.
        if(reserved_){reservedTotal_.fetch_sub(reserved_,std::memory_order_relaxed);reserved_=0;}
    }
    std::size_t bytes() const noexcept {return (first_.capacity()+column_.capacity())*sizeof(std::size_t)+(inverseDiagonal_.capacity()+lower_.capacity())*sizeof(double);}
    bool build(const std::vector<double>&degree,const std::vector<std::size_t>&freeFirst,
               const std::vector<std::size_t>&freeColumn,std::size_t cap=globalLimit_) {
        clear();const auto n=degree.size();
        if(!n||cap<sizeof(std::size_t)||freeFirst.size()!=n+1||freeFirst.front()!=0||freeFirst.back()!=freeColumn.size())return false;
        if(n>(cap-sizeof(std::size_t))/(sizeof(std::size_t)+sizeof(double)))return false;
        const auto base=(n+1)*sizeof(std::size_t)+n*sizeof(double);
        std::size_t entries=0;
        for(std::size_t i=0;i<n;++i){
            if(!(degree[i]>0)||!std::isfinite(degree[i])||freeFirst[i]>freeFirst[i+1]||freeFirst[i+1]>freeColumn.size()||freeFirst[i+1]-freeFirst[i]>256)return false;
            for(std::size_t e=freeFirst[i];e<freeFirst[i+1];++e){const auto j=freeColumn[e];if(j>=n||j==i)return false;entries+=j<i;}
        }
        if(entries>(cap-base)/(sizeof(std::size_t)+sizeof(double)))return false;
        const auto need=base+entries*(sizeof(std::size_t)+sizeof(double));
        if(need>globalLimit_)return false;
        auto used=reservedTotal_.load(std::memory_order_relaxed);
        do{if(used>globalLimit_-need)return false;}
        while(!reservedTotal_.compare_exchange_weak(used,used+need,std::memory_order_relaxed));
        reserved_=need;
        try {
            first_.resize(n+1);column_.resize(entries);lower_.resize(entries);inverseDiagonal_.resize(n);
            // These vectors start empty; the target STL allocates exactly the
            // requested count. Refuse an unexpected over-allocation as well.
            if(bytes()>reserved_){clear();return false;}
            std::size_t at=0;
            for(std::size_t i=0;i<n;++i){first_[i]=at;
                for(std::size_t e=freeFirst[i];e<freeFirst[i+1];++e)if(freeColumn[e]<i)column_[at++]=freeColumn[e];
                std::sort(column_.begin()+first_[i],column_.begin()+at);
            }
            first_[n]=at;
            std::size_t work=0;
            for(std::size_t i=0;i<n;++i){double pivot=degree[i];
                for(std::size_t e=first_[i];e<first_[i+1];++e){const auto j=column_[e];double value=-1.;
                    // Dot product on the intersection of the two existing
                    // lower patterns. Fill outside the original pattern is 0.
                    std::size_t a=first_[i],b=first_[j];
                    while(a<e&&b<first_[j+1]){
                        if(++work>32u*1024u*1024u){clear();return false;}
                        if(column_[a]<column_[b])++a;
                        else if(column_[b]<column_[a])++b;
                        else{value-=lower_[a]*lower_[b];++a;++b;}
                    }
                    value*=inverseDiagonal_[j];
                    if(!std::isfinite(value)){clear();return false;}
                    lower_[e]=value;pivot-=value*value;
                }
                // A failed/unsafe factor is optional: the caller uses its
                // unchanged Jacobi-PCG path. No diagonal shift alters A.
                if(!(pivot>32*std::numeric_limits<double>::epsilon()*degree[i])||!std::isfinite(pivot)){clear();return false;}
                inverseDiagonal_[i]=1./std::sqrt(pivot);
            }
        } catch(const std::bad_alloc&) {clear();return false;}
        return true;
    }
    void apply(const std::vector<double>&r,std::vector<double>&z) const {
        // L y = r, then L^T z = y. In-place transpose scatter avoids a second
        // CSR copy and additional per-bone scratch. The shared factor is const.
        const auto n=inverseDiagonal_.size();
        for(std::size_t i=0;i<n;++i){double v=r[i];for(std::size_t e=first_[i];e<first_[i+1];++e)v-=lower_[e]*z[column_[e]];z[i]=v*inverseDiagonal_[i];}
        for(std::size_t i=n;i-->0;){z[i]*=inverseDiagonal_[i];for(std::size_t e=first_[i];e<first_[i+1];++e)z[column_[e]]-=lower_[e]*z[i];}
    }
};
