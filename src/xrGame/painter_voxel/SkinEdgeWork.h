#pragma once
// Pure packed-data auxiliary work. Never owns the Mesh or reads an engine object.
#include "SkinEdge.h"
#include <atomic>
#include <utility>
namespace skin_edge {
struct WorkKey {
    std::uint16_t root_bone{},cut_bone{};
    std::uint32_t at{};
    std::array<float,3> point{},axis{};
    bool operator==(const WorkKey& b)const noexcept{
        return root_bone==b.root_bone&&cut_bone==b.cut_bone&&at==b.at&&
            std::memcmp(point.data(),b.point.data(),sizeof(point))==0&&
            std::memcmp(axis.data(),b.axis.data(),sizeof(axis))==0;
    }
};
class Work final:public gfy_packed::MeshPostWork {
    const gfy_packed::Mesh* expected_; // Identity only outside Run/Find; the renderer/stage owns its lifetime.
    const std::vector<WorkKey> keys_;
    std::vector<Result> results_;
    std::atomic<unsigned> state_{0}; // 0 unrun;1 running;2 published;3 failed
public:
    Work(const gfy_packed::Mesh* mesh,std::vector<WorkKey> keys):expected_(mesh),keys_(std::move(keys)){}
    void Run(const gfy_packed::Mesh& mesh)noexcept override{
        if(&mesh!=expected_||mesh.post_work.get()!=this)return;
        unsigned idle=0;if(!state_.compare_exchange_strong(idle,1,std::memory_order_acq_rel))return;
        try{
            std::vector<Result> result;result.reserve(keys_.size());
            for(const auto& key:keys_){
                const double c[3]={key.point[0],key.point[1],key.point[2]},a[3]={key.axis[0],key.axis[1],key.axis[2]};
                result.push_back(measure(mesh,key.root_bone,c,a)); // The unchanged original function.
            }
            results_=std::move(result);state_.store(2,std::memory_order_release);
        }catch(...){state_.store(3,std::memory_order_release);} // Auxiliary failure never invalidates geometry.
    }
    const Result* Find(const gfy_packed::Mesh& mesh,const WorkKey& key)const noexcept{
        if(&mesh!=expected_||mesh.post_work.get()!=this||state_.load(std::memory_order_acquire)!=2)return nullptr;
        for(std::size_t i=0;i<keys_.size();++i)if(keys_[i]==key)return &results_[i];
        return nullptr;
    }
};
}
