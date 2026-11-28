#pragma once
#include <atomic>
#include "LiveDamageWorker.h"
#include "PartChildOwnership.h"
namespace painter_flight {
struct ProxyPostWork final:public gfy_packed::MeshPostWork {
    std::shared_ptr<const gfy_packed::Mesh> witness;
    explicit ProxyPostWork(std::shared_ptr<const gfy_packed::Mesh> mesh):witness(std::move(mesh)){}
    void Run(const gfy_packed::Mesh&)noexcept override{witness->post_work->Run(*witness);}
};
inline std::shared_ptr<const gfy_packed::Mesh> proxy_packet(const gfy_packed::Mesh& source,size_t originals,
    u64 cookie,u64 revision,std::shared_ptr<const gfy_packed::Mesh> measured={}){
    auto mesh=std::make_shared<gfy_packed::Mesh>(source);
    if(measured&&measured->post_work)mesh->post_work=std::make_shared<ProxyPostWork>(std::move(measured));
    for(size_t index=0;index<originals;++index){
        const auto owned=std::find_if(mesh->children.begin(),mesh->children.end(),
            [&](const gfy_packed::Child& child){return child.child_index==index;});
        if(owned!=mesh->children.end())continue;
        gfy_packed::Child removed;
        removed.child_index=removed.material_source_child=static_cast<u16>(index);
        mesh->children.push_back(std::move(removed));
    }
    if(mesh->analytic_channels){
        auto fields=std::make_shared<painter_detail::AnalyticChannelField>(*mesh->analytic_channels);
        fields->identity.cookie=cookie;fields->identity.generation=0;fields->identity.revision=revision;
        mesh->analytic_channels=std::move(fields);
    }
    if(mesh->skin_ownership){
        auto fields=std::make_shared<painter_detail::SkinOwnershipField>(*mesh->skin_ownership);
        fields->identity.cookie=cookie;fields->identity.generation=0;fields->identity.revision=revision;
        mesh->skin_ownership=std::move(fields);
    }
    return mesh;
}
struct Group {
    std::shared_ptr<gfy_live::SeparationPreparation> job;
    u32 retryAfter=0;
    unsigned failures=0,admissionFailures=0;
    bool reported=false,exhausted=false;
    std::shared_ptr<const gfy_live::SeparationOutput> exact(u32 now) {
        if(!job||exhausted)return {};
        auto result=job->completed();
        if(!result)return {};
        if(result->error.empty()&&!result->no_separation)return result;
        if(!reported){
            Msg("[painter_flight] phase=exact_retry revision=%llu why=%s",job->stamp().revision,result->error.empty()?"no_separation":result->error.c_str());
            reported=true;++failures;
        }
        if(failures>=3){exhausted=true;return {};}
        if(retryAfter&&s32(now-retryAfter)<0)return {};
        retryAfter=now+1000;
        if(auto retry=job->retry_exact()){job=std::move(retry);reported=false;admissionFailures=0;}
        else if(++admissionFailures>=3){exhausted=true;}
        return {};
    }
};
struct Member {
    std::shared_ptr<Group> group;
    u32 component=0;
    std::vector<part_child_ownership::Role> roles;
    std::vector<part_child_ownership::Decision> plan;
};
}
