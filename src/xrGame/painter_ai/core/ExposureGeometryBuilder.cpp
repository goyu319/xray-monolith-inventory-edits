#include "ExposureGeometryBuilder.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <tuple>

namespace painter { namespace combat { namespace core {
namespace {
bool same(const ExposureGeometryStamp&a,const ExposureGeometryStamp&b){return a.level_epoch==b.level_epoch&&a.map_hash==b.map_hash&&a.geometry_revision==b.geometry_revision&&a.data_revision==b.data_revision&&a.source_basis==b.source_basis;}
bool eq(Vec3 a,Vec3 b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
void mix(std::uint64_t&h,std::uint64_t v){for(unsigned i=0;i<8;++i){h^=(v>>(i*8))&255;h*=1099511628211ull;}}
void mix_float(std::uint64_t&h,float v){if(v==0)v=0;std::uint32_t bits;std::memcpy(&bits,&v,sizeof bits);mix(h,bits);}
void mix_point(std::uint64_t&h,Vec3 v){mix_float(h,v.x);mix_float(h,v.y);mix_float(h,v.z);}
std::uint32_t opening_id(std::uint32_t b,std::uint32_t p,bool reverse){std::uint64_t h=1469598103934665603ull;mix(h,b);mix(h,p);mix(h,reverse);auto id=std::uint32_t(h^(h>>32));return id?id:1;}
bool valid_policy(const ExposureGeometryPolicy&p){const float positive[]={p.grid_m,p.source_range_m,p.source_floor_m,p.pair_m};for(float f:positive)if(!std::isfinite(f)||f<=0)return false;
    const float nonnegative[]={p.inset_m,p.source_height_m,p.project_horizontal_m,p.project_vertical_m};for(float f:nonnegative)if(!std::isfinite(f)||f<0)return false;
    return p.memory_bytes&&p.room_capacity&&p.opening_capacity&&p.source_capacity&&p.link_capacity&&p.grid_cell_limit&&p.source_capacity<(std::numeric_limits<std::size_t>::max)()/2;}
bool add_size(std::size_t&sum,std::size_t n,std::size_t width){if(!width)return true;if(n>((std::numeric_limits<std::size_t>::max)()-sum)/width)return false;sum+=n*width;return true;}
struct Interrupted{};void poll(const std::function<bool()>&stop){if(stop&&stop())throw Interrupted{};}
}
struct ExposureGeometryBuilder::Impl {
    struct RoomRow {std::uint32_t building=0,room=0;Vec3 min,max;float floor=0;std::size_t owner_index=0;};
    struct OpeningRow {ExposureOpening opening;ExposurePortalOrientation origin;};
    struct SourceSlot {ExposureSource source;};
    ExposureGeometryInput geometry;ExposureGeometryStamp input;ExposureGeometryProgress progress;
    std::vector<RoomRow> rooms;std::vector<OpeningRow> openings;std::vector<ExposurePortalOrientation> origins;std::vector<SourceSlot> table;
    std::size_t building=0,item=0,sort_end=0,room=0,probe=0,source=0,opening=0,second=0,hash_stage=0,hash_index=0;
    std::uint64_t gx=0,gz=0,nx=0,nz=0;unsigned grid_stage=0,link_stage=0;Vec3 request,snapped;std::uint32_t node=invalid_node;std::uint64_t hash=1469598103934665603ull;
    static bool room_less(const RoomRow&a,const RoomRow&b){return std::tie(a.building,a.room)<std::tie(b.building,b.room);}
    static bool opening_less(const OpeningRow&a,const OpeningRow&b){return a.opening.id<b.opening.id;}
    static bool source_less(const ExposureSource&a,const ExposureSource&b){return a.id<b.id;}
    const RoomRow* find_room(std::uint32_t b,std::uint32_t r)const{RoomRow key;key.building=b;key.room=r;auto i=std::lower_bound(rooms.begin(),rooms.end(),key,room_less);return i!=rooms.end()&&i->building==b&&i->room==r?&*i:nullptr;}
    void next_grid(){grid_stage=0;if(++gz>=nz){gz=0;if(++gx>=nx){gx=0;++room;nx=nz=0;}}}
};
ExposureGeometryBuilder::ExposureGeometryBuilder(ExposureGeometryPolicy p):policy_(p){}
ExposureGeometryBuilder::~ExposureGeometryBuilder()=default;
void ExposureGeometryBuilder::reset(){impl_.reset();}
ExposureGeometryProgress ExposureGeometryBuilder::state()const{return impl_?impl_->progress:ExposureGeometryProgress{};}
const ExposureGeometryInput* ExposureGeometryBuilder::geometry()const{return impl_&&impl_->progress.complete?&impl_->geometry:nullptr;}
const std::vector<ExposurePortalOrientation>* ExposureGeometryBuilder::orientations()const{return impl_&&impl_->progress.complete?&impl_->origins:nullptr;}
ExposureGeometryProgress ExposureGeometryBuilder::advance(const ExposureGeometryStamp&stamp,const EngineServices&svc,ExposureWorkQuota&q,const std::function<bool()>&stop){
    const auto work0=q.work_used,queries0=q.queries_used;auto out=[&](){auto r=state();r.work=q.work_used-work0;r.queries=q.queries_used-queries0;return r;};
    auto query=[&](){poll(stop);if(!q.queries_left)return false;--q.queries_left;++q.queries_used;return true;};
    try{
        if(impl_&&!same(impl_->input,stamp))reset();poll(stop);
        const auto*cache=svc.buildings;
        if(!valid_policy(policy_)||!stamp.level_epoch||!stamp.geometry_revision||!stamp.source_basis||!cache||!cache->locate||!svc.project||
            cache->level_epoch!=stamp.level_epoch||cache->map_hash!=stamp.map_hash||cache->data_version!=stamp.data_revision){reset();return out();}
        if(!impl_){
            std::size_t bytes=sizeof(*this)+sizeof(Impl);const auto table_count=policy_.source_capacity*2+1;
            if(!add_size(bytes,policy_.room_capacity,sizeof(Impl::RoomRow))||!add_size(bytes,policy_.opening_capacity,sizeof(Impl::OpeningRow)+sizeof(ExposureOpening)+sizeof(ExposurePortalOrientation))||
                !add_size(bytes,policy_.source_capacity,sizeof(ExposureSource))||!add_size(bytes,policy_.link_capacity,sizeof(ExposureSourceOpening))||!add_size(bytes,table_count,sizeof(Impl::SourceSlot))||
                !add_size(bytes,8,policy_.allocation_overhead_bytes)||bytes>policy_.memory_bytes){auto r=out();r.status=ExposureGeometryStatus::memory_limit;return r;}
            auto n=std::make_unique<Impl>();n->input=stamp;n->geometry.stamp=stamp;n->geometry.pair_max_m=policy_.pair_m;
            poll(stop);n->rooms.reserve(policy_.room_capacity);n->openings.reserve(policy_.opening_capacity);n->origins.reserve(policy_.opening_capacity);
            n->geometry.openings.reserve(policy_.opening_capacity);n->geometry.sources.reserve(policy_.source_capacity);n->geometry.links.reserve(policy_.link_capacity);n->table.resize(table_count);
            bytes=sizeof(*this)+sizeof(Impl);add_size(bytes,n->rooms.capacity(),sizeof(Impl::RoomRow));add_size(bytes,n->openings.capacity(),sizeof(Impl::OpeningRow));
            add_size(bytes,n->origins.capacity(),sizeof(ExposurePortalOrientation));add_size(bytes,n->geometry.openings.capacity(),sizeof(ExposureOpening));
            add_size(bytes,n->geometry.sources.capacity(),sizeof(ExposureSource));add_size(bytes,n->geometry.links.capacity(),sizeof(ExposureSourceOpening));add_size(bytes,n->table.capacity(),sizeof(Impl::SourceSlot));add_size(bytes,8,policy_.allocation_overhead_bytes);
            if(bytes>policy_.memory_bytes){auto r=out();r.status=ExposureGeometryStatus::memory_limit;return r;}
            n->progress.charged_bytes=bytes;n->progress.status=ExposureGeometryStatus::pending;
            mix(n->hash,stamp.source_basis);mix(n->hash,stamp.level_epoch);mix(n->hash,stamp.map_hash);mix(n->hash,stamp.geometry_revision);mix(n->hash,stamp.data_revision);
            for(float f:{policy_.grid_m,policy_.inset_m,policy_.source_height_m,policy_.source_range_m,policy_.source_floor_m,policy_.project_horizontal_m,policy_.project_vertical_m,policy_.pair_m})mix_float(n->hash,f);
            poll(stop);impl_=std::move(n);
        }
        auto&n=*impl_;auto&pr=n.progress;auto&g=n.geometry;
        if(pr.complete||pr.status==ExposureGeometryStatus::invalid||pr.status==ExposureGeometryStatus::capacity_limit||pr.status==ExposureGeometryStatus::memory_limit)return out();
        pr.status=ExposureGeometryStatus::pending;
        auto capacity=[&](){pr.status=ExposureGeometryStatus::capacity_limit;};auto invalid=[&](){pr.status=ExposureGeometryStatus::invalid;};
        while(q.work_left){poll(stop);--q.work_left;++q.work_used;
            using Phase=ExposureGeometryPhase;
            if(pr.phase==Phase::rooms){
                if(n.building>=cache->buildings.size()){pr.phase=Phase::room_sort;n.sort_end=n.rooms.size();continue;}
                const auto&b=cache->buildings[n.building];if(b.id==invalid_room){invalid();break;}
                if(n.item>=b.rooms.size()){++n.building;n.item=0;continue;}const auto&r=b.rooms[n.item];
                if(r.id==invalid_room||!finite(r.min)||!finite(r.max)||!std::isfinite(r.floor_y)||r.min.x>r.max.x||r.min.z>r.max.z){invalid();break;}
                if(n.rooms.size()>=policy_.room_capacity){capacity();break;}
                n.rooms.push_back({b.id,r.id,r.min,r.max,r.floor_y,n.building});std::push_heap(n.rooms.begin(),n.rooms.end(),Impl::room_less);++n.item;pr.rooms=n.rooms.size();
            }else if(pr.phase==Phase::room_sort){if(n.sort_end>1){std::pop_heap(n.rooms.begin(),n.rooms.begin()+n.sort_end,Impl::room_less);--n.sort_end;}else{pr.phase=Phase::room_check;n.item=0;}}
            else if(pr.phase==Phase::room_check){if(n.item>=n.rooms.size()){pr.phase=Phase::portals;n.building=n.item=0;continue;}
                if(n.item&&n.rooms[n.item].building==n.rooms[n.item-1].building&&(n.rooms[n.item].room==n.rooms[n.item-1].room||n.rooms[n.item].owner_index!=n.rooms[n.item-1].owner_index)){invalid();break;}++n.item;
            }else if(pr.phase==Phase::portals){
                if(n.building>=cache->buildings.size()){pr.phase=Phase::opening_sort;n.sort_end=n.openings.size();continue;}
                const auto&b=cache->buildings[n.building];if(n.item>=b.portals.size()){++n.building;n.item=0;continue;}const auto&p=b.portals[n.item];
                const auto*a=n.find_room(b.id,p.room_a);const auto*z=p.room_b==invalid_room?nullptr:n.find_room(b.id,p.room_b);
                const double length=std::hypot(double(p.normal.x),double(p.normal.z));
                if(!a||(p.room_b!=invalid_room&&!z)||p.room_a==p.room_b||!finite(p.centre)||!finite(p.normal)||!std::isfinite(length)||length<=0||
                    !std::isfinite(p.half_width)||p.half_width<=0||!std::isfinite(p.y0)||!std::isfinite(p.y1)||p.y1<=p.y0){invalid();break;}
                if(policy_.opening_capacity-n.openings.size()<(z?2u:1u)){capacity();break;}
                for(unsigned reverse=0;reverse<(z?2u:1u);++reverse){Impl::OpeningRow row;auto&o=row.opening;o.id=opening_id(b.id,p.id,reverse!=0);o.building=b.id;
                    o.far_room=reverse?p.room_b:p.room_a;o.near_room=reverse?p.room_a:p.room_b;o.centre=p.centre;
                    const float sign=reverse?-1.f:1.f;o.normal={float(p.normal.x/length)*sign,0,float(p.normal.z/length)*sign};o.half_width=p.half_width;o.y0=p.y0;o.y1=p.y1;
                    row.origin={o.id,b.id,p.id,o.far_room,o.near_room};
                    n.openings.push_back(row);std::push_heap(n.openings.begin(),n.openings.end(),Impl::opening_less);}
                ++n.item;pr.openings=n.openings.size();
            }else if(pr.phase==Phase::opening_sort){if(n.sort_end>1){std::pop_heap(n.openings.begin(),n.openings.begin()+n.sort_end,Impl::opening_less);--n.sort_end;}else{pr.phase=Phase::opening_copy;n.item=0;}}
            else if(pr.phase==Phase::opening_copy){if(n.item>=n.openings.size()){pr.phase=Phase::grid;continue;}
                const auto&row=n.openings[n.item];if(n.item&&row.opening.id==n.openings[n.item-1].opening.id){invalid();break;}
                g.openings.push_back(row.opening);n.origins.push_back(row.origin);++n.item;
            }else if(pr.phase==Phase::grid){
                if(n.room>=n.rooms.size()){pr.phase=Phase::source_sort;n.sort_end=g.sources.size();continue;}const auto&r=n.rooms[n.room];
                if(!n.nx){const double width=double(r.max.x)-r.min.x-2*policy_.inset_m,depth=double(r.max.z)-r.min.z-2*policy_.inset_m;
                    if(width<0||depth<0){++n.room;continue;}const double nx=std::floor(width/policy_.grid_m)+1,nz=std::floor(depth/policy_.grid_m)+1;
                    if(nx>double(policy_.grid_cell_limit)||nz>double(policy_.grid_cell_limit)||nx*nz>double(policy_.grid_cell_limit-pr.grid_candidates)){capacity();break;}
                    n.nx=std::uint64_t(nx);n.nz=std::uint64_t(nz);}
                if(n.grid_stage==0){if(!query())break;Vec3 request{float(double(r.min.x)+policy_.inset_m+double(n.gx)*policy_.grid_m),r.floor,float(double(r.min.z)+policy_.inset_m+double(n.gz)*policy_.grid_m)};
                    Vec3 snapped;const auto node=svc.project(request,policy_.project_horizontal_m,policy_.project_vertical_m,&snapped);poll(stop);
                    if(node==invalid_node){++pr.grid_candidates;n.next_grid();continue;}
                    const double dx=double(snapped.x)-request.x,dz=double(snapped.z)-request.z;
                    if(!finite(snapped)||dx*dx+dz*dz>double(policy_.project_horizontal_m)*policy_.project_horizontal_m||std::abs(double(snapped.y)-request.y)>policy_.project_vertical_m){invalid();break;}
                    if(snapped.x<r.min.x||snapped.x>r.max.x||snapped.z<r.min.z||snapped.z>r.max.z){++pr.grid_candidates;n.next_grid();continue;}
                    n.request=request;n.snapped=snapped;n.node=node;n.grid_stage=svc.node_position?1:2;
                }else if(n.grid_stage==1){if(!query())break;Vec3 actual;const bool found=svc.node_position(n.node,&actual);poll(stop);
                    if(!found){pr.status=ExposureGeometryStatus::query_failure;break;}if(!finite(actual)||!eq(actual,n.snapped)){invalid();break;}n.grid_stage=2;
                }else if(n.grid_stage==2){if(!query())break;const auto at=cache->locate(n.node);poll(stop);if(at.first!=r.building||at.second!=r.room){++pr.grid_candidates;n.next_grid();continue;}
                    n.probe=(std::uint64_t(n.node)+1)%n.table.size();n.grid_stage=3;
                }else{auto&slot=n.table[n.probe];ExposureSource s;s.id=std::uint64_t(n.node)+1;s.building=r.building;s.room=r.room;s.node=n.node;s.point=n.snapped;s.point.y+=policy_.source_height_m;
                    if(!finite(s.point)){invalid();break;}
                    if(!slot.source.id){if(g.sources.size()>=policy_.source_capacity){capacity();break;}slot.source=s;g.sources.push_back(s);std::push_heap(g.sources.begin(),g.sources.end(),Impl::source_less);pr.sources=g.sources.size();++pr.grid_candidates;n.next_grid();}
                    else if(slot.source.id==s.id){if(slot.source.building!=s.building||slot.source.room!=s.room||!eq(slot.source.point,s.point)){invalid();break;}++pr.duplicate_sources;++pr.grid_candidates;n.next_grid();}
                    else n.probe=(n.probe+1)%n.table.size();
                }
            }else if(pr.phase==Phase::source_sort){if(n.sort_end>1){std::pop_heap(g.sources.begin(),g.sources.begin()+n.sort_end,Impl::source_less);--n.sort_end;}else{pr.phase=Phase::links;}}
            else if(pr.phase==Phase::links){
                if(n.source>=g.sources.size()){pr.phase=Phase::fingerprint;continue;}if(n.opening>=n.openings.size()){++n.source;n.opening=0;n.link_stage=0;continue;}
                const auto&s=g.sources[n.source];const auto&first=n.openings[n.opening];const auto&o=first.opening;
                if(n.link_stage==0){const double dx=double(s.point.x)-o.centre.x,dz=double(s.point.z)-o.centre.z;
                    const double normal=dx*o.normal.x+dz*o.normal.z,tangent=std::max(0.0,std::abs(dx*-o.normal.z+dz*o.normal.x)-o.half_width);
                    if(s.building!=o.building||s.room!=o.far_room||normal*normal+tangent*tangent>double(policy_.source_range_m)*policy_.source_range_m){++n.opening;continue;}
                    n.second=0;n.link_stage=1;
                }else if(n.link_stage==1){
                    if(n.second>=n.openings.size()){n.link_stage=2;continue;}const auto index=n.second;const auto&row=n.openings[index];const auto&two=row.opening;
                    const double dx=double(o.centre.x)-two.centre.x,dz=double(o.centre.z)-two.centre.z;
                    const bool qualifies=index!=n.opening&&o.near_room!=invalid_room&&two.building==o.building&&two.far_room==o.near_room&&two.near_room!=s.room&&
                        dx*dx+dz*dz<=double(policy_.pair_m)*policy_.pair_m;
                    if(qualifies){if(g.links.size()>=policy_.link_capacity){capacity();break;}g.links.push_back({n.source,n.opening,index});}++n.second;
                }else{if(g.links.size()>=policy_.link_capacity){capacity();break;}g.links.push_back({n.source,n.opening,ExposureSourceOpening::no_second});
                    ++n.opening;n.link_stage=0;}
                pr.links=g.links.size();
            }else if(pr.phase==Phase::fingerprint){
                if(n.hash_stage==0){if(n.hash_index>=g.sources.size()){++n.hash_stage;n.hash_index=0;continue;}const auto&s=g.sources[n.hash_index++];mix(n.hash,s.id);mix(n.hash,s.building);mix(n.hash,s.room);mix(n.hash,s.node);mix_point(n.hash,s.point);}
                else if(n.hash_stage==1){if(n.hash_index>=g.openings.size()){++n.hash_stage;n.hash_index=0;continue;}const auto&o=g.openings[n.hash_index++];mix(n.hash,o.id);mix(n.hash,o.building);mix(n.hash,o.far_room);mix(n.hash,o.near_room);mix_point(n.hash,o.centre);mix_point(n.hash,o.normal);mix_float(n.hash,o.half_width);mix_float(n.hash,o.y0);mix_float(n.hash,o.y1);}
                else if(n.hash_stage==2){if(n.hash_index>=g.links.size()){++n.hash_stage;continue;}const auto&l=g.links[n.hash_index++];mix(n.hash,l.source);mix(n.hash,l.opening);mix(n.hash,l.second_opening);}
                else{g.stamp.source_basis=n.hash?n.hash:1;g.paired_links_complete=true;pr.phase=Phase::done;pr.complete=true;pr.status=ExposureGeometryStatus::complete;break;}
            }else break;
        }
        return out();
    }catch(const Interrupted&){auto r=out();r.status=ExposureGeometryStatus::cancelled;r.complete=false;return r;}
    catch(const std::bad_alloc&){auto r=out();r.status=ExposureGeometryStatus::memory_limit;r.complete=false;return r;}
    catch(...){auto r=out();r.status=ExposureGeometryStatus::query_failure;r.complete=false;return r;}
}
}}}
