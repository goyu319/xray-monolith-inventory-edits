#pragma once
// Immutable triangle-local keep regions on authored skin. A plane keeps
// dot(plane.xyz, original interpolated rest position) + plane.w >= 0.
// These are convex intersections; they never alter anatomy or damage voxels.
#include "AnalyticChannelField.h"
#include <algorithm>
#include <map>
namespace painter_detail {
struct SkinOwnershipChild {
    static constexpr unsigned width=4096;
    unsigned child{},count{};
    std::vector<std::array<std::uint32_t,2>> data;
};
struct SkinOwnershipField {
    static constexpr unsigned planeWidth=4096;
    ChannelIdentity identity;
    std::vector<std::array<float,4>> planes; // Logical entries, GPU padding is separate.
    std::vector<SkinOwnershipChild> children;
    const SkinOwnershipChild* child(unsigned slot)const noexcept {
        for(const auto&c:children)if(c.child==slot)return &c;return nullptr;
    }
    std::size_t bytes()const noexcept {
        std::size_t n=planes.size()*sizeof(planes[0]);for(const auto&c:children)n+=c.data.size()*sizeof(c.data[0]);return n;
    }
    void validate()const {
        if(!identity.snapshot||planes.empty()||children.empty())throw std::invalid_argument("skin_owner_empty_field");
        for(const auto&p:planes){for(float v:p)if(!std::isfinite(v))throw std::invalid_argument("skin_owner_nonfinite_plane");
            if(!(double(p[0])*p[0]+double(p[1])*p[1]+double(p[2])*p[2]>1e-20))throw std::invalid_argument("skin_owner_zero_plane");}
        std::vector<unsigned>slots;
        for(const auto&c:children){if(c.child>=1024||!c.count||c.count>65535||c.data.size()!=((std::size_t(c.count)+c.width-1)/c.width)*c.width||std::find(slots.begin(),slots.end(),c.child)!=slots.end())throw std::invalid_argument("skin_owner_child_manifest");
            slots.push_back(c.child);bool any=false;
            for(unsigned t=0;t<c.count;++t){const auto&r=c.data[t];if(r[0]>planes.size()||r[1]>planes.size()-r[0])throw std::invalid_argument("skin_owner_plane_range");any|=r[1]!=0;}
            if(!any)throw std::invalid_argument("skin_owner_unrestricted_child");
            for(std::size_t t=c.count;t<c.data.size();++t)if(c.data[t]!=std::array<std::uint32_t,2>{})throw std::invalid_argument("skin_owner_padding");}
    }
};
class SkinOwnershipBuilder {
    SkinOwnershipField value;
    std::map<unsigned,std::vector<std::array<std::uint32_t,2>>> rows;
public:
    explicit SkinOwnershipBuilder(ChannelIdentity identity){value.identity=std::move(identity);}
    void append(unsigned dstChild,const SkinOwnershipField*source,unsigned srcChild,std::size_t srcTriangle,
                const std::vector<std::array<float,4>>&extra={}){
        if(dstChild>=1024)throw std::invalid_argument("skin_owner_destination");
        auto&dst=rows[dstChild];if(dst.size()==65535)throw std::length_error("skin_owner_triangle_budget");
        std::vector<std::array<float,4>>keep;
        if(source){const auto*c=source->child(srcChild);if(c){if(srcTriangle>=c->count||srcTriangle>=c->data.size())throw std::invalid_argument("skin_owner_source_triangle");
            const auto&r=c->data[srcTriangle];if(r[0]>source->planes.size()||r[1]>source->planes.size()-r[0])throw std::invalid_argument("skin_owner_source_range");
            keep.insert(keep.end(),source->planes.begin()+r[0],source->planes.begin()+r[0]+r[1]);}}
        for(const auto&p:extra)if(std::find(keep.begin(),keep.end(),p)==keep.end())keep.push_back(p);
        if(value.planes.size()>std::numeric_limits<std::uint32_t>::max()-keep.size())throw std::length_error("skin_owner_plane_budget");
        const auto offset=std::uint32_t(value.planes.size());value.planes.insert(value.planes.end(),keep.begin(),keep.end());
        dst.push_back({offset,std::uint32_t(keep.size())});
    }
    std::shared_ptr<const SkinOwnershipField> finish(){
        if(value.planes.empty())return {};
        for(auto&item:rows){bool any=false;for(const auto&r:item.second)any|=r[1]!=0;if(!any)continue;
            SkinOwnershipChild c;c.child=item.first;c.count=unsigned(item.second.size());c.data=std::move(item.second);
            c.data.resize(((c.data.size()+c.width-1)/c.width)*c.width);value.children.push_back(std::move(c));}
        value.validate();return std::make_shared<const SkinOwnershipField>(std::move(value));
    }
};
} // namespace painter_detail
