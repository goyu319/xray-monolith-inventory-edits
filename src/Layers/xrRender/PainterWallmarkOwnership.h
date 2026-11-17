#pragma once
// Decal-local indirection. A decal range is a UNION of convex regions; each
// region is an INTERSECTION of rest-space planes. It is never a skin primitive ID.
#include "../../Include/xrRender/RuntimePackedMesh.h"
#include "../../Include/xrRender/SkinOwnershipField.h"
#include <array>
#include <atomic>
#include <cstring>
#include <map>
#include <set>
#include <stdexcept>
#include <type_traits>
namespace PainterWallmarkOwnership {
using Range=std::array<std::uint32_t,2>;
using Plane=std::array<float,4>;
using TriangleKey=std::array<std::uint32_t,33>;
constexpr Range unrestricted={0,0};
constexpr Range rejected={~0u,0};
struct Primitive { unsigned source_child{},source_face{};std::unique_ptr<TriangleKey> key;Range direct{}; };
struct Field {
    std::shared_ptr<const painter_detail::SkinOwnershipField> source;
    std::shared_ptr<void> accounting; // The CPU provenance may outlive its GPU holder.
    std::vector<Plane> planes;
    std::vector<Range> regions;
    std::map<unsigned,std::vector<Primitive>> primitives;
    std::map<std::pair<unsigned,unsigned>,Range> authored;
    std::map<std::pair<unsigned,TriangleKey>,Range> cavities;
    std::set<unsigned> replaced;
    const Primitive* primitive(unsigned child,unsigned triangle)const noexcept {
        const auto c=primitives.find(child);return c==primitives.end()||triangle>=c->second.size()?nullptr:&c->second[triangle];
    }

};
inline std::uint32_t bits(float f){std::uint32_t n;std::memcpy(&n,&f,4);return n;}
inline TriangleKey key(const gfy_packed::Child&c,unsigned triangle){
    std::array<std::array<std::uint32_t,11>,3> rows{};
    for(unsigned k=0;k<3;++k){const auto&v=c.vertices.at(c.indices.at(std::size_t(triangle)*3+k));
        for(unsigned q=0;q<3;++q)rows[k][q]=bits(v.surface[q]);
        for(unsigned q=0;q<4;++q)rows[k][3+q]=v.bone[q];
        for(unsigned q=0;q<3;++q)rows[k][7+q]=bits(v.weight[q]);rows[k][10]=4;}
    // Rotation/winding of an unchanged triangle does not change its surface.
    std::sort(rows.begin(),rows.end());TriangleKey out{};
    for(unsigned k=0;k<3;++k)std::copy(rows[k].begin(),rows[k].end(),out.begin()+k*11);return out;
}
inline std::array<Plane,3> edges(const gfy_packed::Child&c,unsigned triangle){
    std::array<std::array<double,3>,3> p{};for(unsigned k=0;k<3;++k){const auto&v=c.vertices.at(c.indices.at(std::size_t(triangle)*3+k));
        for(unsigned a=0;a<3;++a){p[k][a]=v.surface[a];if(!std::isfinite(p[k][a]))throw std::invalid_argument("decal_owner_nonfinite_position");}}
    const auto cross=[](const auto&a,const auto&b){return std::array<double,3>{a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};};
    std::array<double,3>a{},b{};for(unsigned q=0;q<3;++q){a[q]=p[1][q]-p[0][q];b[q]=p[2][q]-p[0][q];}
    auto n=cross(a,b);const double length=std::sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);
    // An authored zero-area primitive cannot carry visible decal coverage.
    // Keep its direct metadata; only its carry region is the empty set.
    if(!(length>0)||!std::isfinite(length))return {Plane{0,0,0,-1},Plane{},Plane{}};for(double&v:n)v/=length;
    std::array<Plane,3>out{};for(unsigned k=0;k<3;++k){for(unsigned q=0;q<3;++q)a[q]=p[(k+1)%3][q]-p[k][q];auto e=cross(n,a);
        double d=0;for(unsigned q=0;q<3;++q)d-=e[q]*p[k][q];Plane plane={float(e[0]),float(e[1]),float(e[2]),float(d)};
        for(float v:plane)if(!std::isfinite(v))throw std::invalid_argument("decal_owner_nonfinite_edge");out[k]=plane;}return out;
}
// Bounds are computed on the field worker with no table allocations. The
// existing native child/face limits remain unchanged; over-budget publication
// is refused intact rather than truncating triangles or ownership regions.
struct Plan {
    std::size_t triangles{},unknown{},child_count{},direct{},planes{},regions{};
    std::size_t cpu_retained{},cpu_transient{},gpu{},peak{},retained{};
};
inline std::size_t add(std::size_t a,std::size_t b){if(b>std::numeric_limits<std::size_t>::max()-a)throw std::length_error("decal_owner_size");return a+b;}
inline std::size_t mul(std::size_t a,std::size_t b){if(a&&b>std::numeric_limits<std::size_t>::max()/a)throw std::length_error("decal_owner_size");return a*b;}
inline std::size_t node(std::size_t value){return add((add(value,4*sizeof(void*))+15)&~std::size_t(15),64);}
inline std::size_t padded(std::size_t n,std::size_t stride){return (add(mul((add(n,4095)/4096)*4096,stride),65535)/65536)*65536;}
inline Plan plan(const gfy_packed::Mesh&mesh){
    Plan p;if(mesh.children.size()>gfy_packed::max_child_slots)throw std::invalid_argument("decal_owner_child_count");
    // Source validation uses at most1024 slot IDs, independent of mesh size.
    if(mesh.skin_ownership){mesh.skin_ownership->validate();p.planes=mesh.skin_ownership->planes.size();}
    std::array<bool,gfy_packed::max_child_slots>slots{};
    std::size_t own_planes=0;
    for(const auto&c:mesh.children){if(c.material)continue;
        if(c.child_index>=slots.size()||slots[c.child_index])throw std::invalid_argument("decal_owner_child_duplicate");slots[c.child_index]=true;
        if(c.vertices.size()>65535||c.indices.size()%3||c.indices.size()/3>65535||(!c.original_faces.empty()&&c.original_faces.size()!=c.indices.size()/3))throw std::invalid_argument("decal_owner_source_manifest");
        ++p.child_count;const auto count=c.indices.size()/3;p.triangles=add(p.triangles,count);
        const auto*owned=mesh.skin_ownership?mesh.skin_ownership->child(c.child_index):nullptr;
        if(owned&&owned->count!=count)throw std::invalid_argument("decal_owner_primitive_manifest");
        for(std::size_t t=0;t<count;++t){p.unknown+=c.original_faces.empty()||c.original_faces[t]==~0u;
            if(owned){own_planes=add(own_planes,owned->data[t][1]);p.direct+=owned->data[t][1]!=0;}}
    }
    p.planes=std::max<std::size_t>(1,add(p.planes,add(mul(p.triangles,3),own_planes)));
    p.regions=std::max<std::size_t>(1,add(p.triangles,p.direct));
    if(p.planes>std::size_t(4096)*16384||p.regions>0x1000000u)throw std::length_error("decal_owner_texture_extent");
    p.gpu=add(padded(p.planes,16),padded(p.regions,8));
    // Exact reserve bounds for planes, regions and primitive rows. Tree nodes
    // include four links, alignment and64B allocator overhead. Known authored
    // faces need no132B key; only unknown provenance retains exact native rows.
    using Auth=std::pair<unsigned,unsigned>;using Cave=std::pair<unsigned,TriangleKey>;
    using PendingA=std::pair<const Auth,std::vector<Range>>;using PendingC=std::pair<const Cave,std::vector<Range>>;
    const auto known=p.triangles-p.unknown;
    p.cpu_retained=add(sizeof(Field)+256,add(mul(p.planes,sizeof(Plane)),mul(p.regions,sizeof(Range))));
    p.cpu_retained=add(p.cpu_retained,mul(p.triangles,sizeof(Primitive)));
    p.cpu_retained=add(p.cpu_retained,mul(p.child_count,node(sizeof(std::pair<const unsigned,std::vector<Primitive>>))+node(sizeof(unsigned))+64));
    p.cpu_retained=add(p.cpu_retained,mul(known,node(sizeof(std::pair<const Auth,Range>))));
    p.cpu_retained=add(p.cpu_retained,mul(p.unknown,add(node(sizeof(std::pair<const Cave,Range>)),sizeof(TriangleKey)+64)));
    // Pending union vectors grow geometrically. Old+new buffers are bounded by
    // three range rows per triangle during growth; map counts are worst-case.
    p.cpu_transient=add(mul(known,node(sizeof(PendingA))),mul(p.unknown,node(sizeof(PendingC))));
    p.cpu_transient=add(p.cpu_transient,add(mul(p.triangles,3*sizeof(Range)),mul(p.triangles,64)));
    const auto upload=std::max(mul((p.planes+4095)/4096*4096,16),mul((p.regions+4095)/4096*4096,8));
    p.cpu_transient=add(p.cpu_transient,upload+128);
    p.retained=add(p.cpu_retained,p.gpu);p.peak=add(p.retained,p.cpu_transient);return p;
}
inline std::shared_ptr<const Field>build(const gfy_packed::Mesh&mesh,bool force=false,const Plan*prepared=nullptr,std::shared_ptr<void>accounting={}){
    if(!mesh.skin_ownership&&!force)return {};
    const auto bounds=prepared?*prepared:plan(mesh);
    auto f=std::make_shared<Field>();f->accounting=std::move(accounting);f->source=mesh.skin_ownership;
    f->planes.reserve(bounds.planes);f->regions.reserve(bounds.regions);
    if(f->source)f->planes.insert(f->planes.end(),f->source->planes.begin(),f->source->planes.end());
    std::map<std::pair<unsigned,unsigned>,std::vector<Range>>authored;
    std::map<std::pair<unsigned,TriangleKey>,std::vector<Range>>cavities;
    auto region=[&](Range p)->Range{if(!p[1])return unrestricted;const auto first=std::uint32_t(f->regions.size());f->regions.push_back(p);return {first,1};};
    for(const auto&c:mesh.children){if(c.material)continue; // material children do not receive wallmarks
        f->replaced.insert(c.material_source_child);if(c.indices.empty())continue;
        auto&rows=f->primitives[c.child_index];rows.reserve(c.indices.size()/3);
        const auto*owned=f->source?f->source->child(c.child_index):nullptr;
        for(unsigned t=0;t<c.indices.size()/3;++t){Primitive row;row.source_child=c.material_source_child;row.source_face=c.original_faces.empty()?~0u:c.original_faces[t];
            Range own{};if(owned)own=owned->data[t];row.direct=region(own);
            const auto edge=edges(c,t);const auto first=std::uint32_t(f->planes.size());f->planes.insert(f->planes.end(),edge.begin(),edge.end());
            if(own[1])f->planes.insert(f->planes.end(),f->source->planes.begin()+own[0],f->source->planes.begin()+own[0]+own[1]);
            const Range convex{first,3+own[1]};
            if(row.source_face!=~0u)authored[{row.source_child,row.source_face}].push_back(convex);
            else{row.key=std::make_unique<TriangleKey>(key(c,t));cavities[{row.source_child,*row.key}].push_back(convex);}rows.push_back(std::move(row));}}
    auto flatten=[&](const auto&src,auto&dst){for(const auto&v:src){const auto first=std::uint32_t(f->regions.size());f->regions.insert(f->regions.end(),v.second.begin(),v.second.end());dst.emplace(v.first,Range{first,std::uint32_t(v.second.size())});}};
    flatten(authored,f->authored);flatten(cavities,f->cavities);
    if(f->planes.empty())f->planes.push_back({0,0,0,-1});if(f->regions.empty())f->regions.push_back({0,1});
    if(f->planes.size()>bounds.planes||f->regions.size()>bounds.regions)throw std::length_error("decal_owner_plan_mismatch");return f;
}
inline bool keep(const Field&f,Range range,const std::array<float,3>&p){
    if(!range[1])return range[0]!=~0u;if(range[0]>f.regions.size()||range[1]>f.regions.size()-range[0])return false;
    for(unsigned g=0;g<range[1];++g){const auto&r=f.regions[range[0]+g];if(r[0]>f.planes.size()||r[1]>f.planes.size()-r[0])return false;bool yes=true;
        for(unsigned q=0;q<r[1];++q){const auto&n=f.planes[r[0]+q];const float d=n[0]*p[0]+n[1]*p[1]+n[2]*p[2]+n[3];if(!(d>=0)){yes=false;break;}}if(yes)return true;}return false;
}
} // namespace PainterWallmarkOwnership

#ifdef USE_DX11
#include "PainterFieldBudget.h"
namespace PainterChannelCache { inline void EvictAllUnpinned(); }
struct PainterWallmarkOwnershipCharge {
    PainterFieldBudget::Lease lease;
    void Reduce(std::size_t need)noexcept{if(need<lease.bytes){const auto delta=lease.bytes-need;lease.bytes=need;PainterFieldBudget::used().fetch_sub(delta,std::memory_order_acq_rel);}}
};
struct PainterWallmarkOwnershipGpu {
    std::shared_ptr<const PainterWallmarkOwnership::Field> field;
    // Only this pending immutable mesh is pinned. BuildNative consumes/releases
    // it after CPU construction; the weak identity never retains body history.
    std::shared_ptr<const gfy_packed::Mesh> pending_mesh;
    std::weak_ptr<const gfy_packed::Mesh> mesh_identity;
    bool force=false;
    std::shared_ptr<PainterWallmarkOwnershipCharge> charge;
    std::atomic<unsigned>native{0};std::uint64_t device_epoch{};ID3D11Device*device_identity=nullptr;
    ID3D11Texture2D*plane_surface=nullptr,*region_surface=nullptr;
    ref_texture plane_texture,region_texture;xr_string plane_name,region_name;
    bool registered=false,retired=false;u32 last_frame{};
    HRESULT native_result=E_PENDING;const char* failure_phase="unqueued";
    std::size_t requested_bytes{},budget_at_failure{};
    ~PainterWallmarkOwnershipGpu(){_RELEASE(plane_surface);_RELEASE(region_surface);}
    bool ReserveNative()noexcept{unsigned zero=0;return native.compare_exchange_strong(zero,1,std::memory_order_acq_rel);}
    void FailNative(const char* phase="queue",HRESULT result=E_ABORT)noexcept{pending_mesh.reset();field.reset();charge.reset();_RELEASE(plane_surface);_RELEASE(region_surface);failure_phase=phase;native_result=result;native.store(3,std::memory_order_release);}
    void BuildNative(ID3D11Device*device)noexcept{
        if(native.load(std::memory_order_acquire)!=1)return;device_identity=device;
        PainterWallmarkOwnership::Plan bounds;
        try{
            auto input=std::move(pending_mesh);if(!input){FailNative("input",E_POINTER);return;}
            failure_phase="plan";
            bounds=PainterWallmarkOwnership::plan(*input);
            auto reservation=std::make_shared<PainterWallmarkOwnershipCharge>();
            requested_bytes=bounds.peak;
            if(!reservation->lease.Acquire(bounds.peak)){budget_at_failure=PainterFieldBudget::used().load(std::memory_order_acquire);FailNative("budget",E_OUTOFMEMORY);return;}
            charge=std::move(reservation);
            failure_phase="cpu_build";field=PainterWallmarkOwnership::build(*input,force,&bounds,charge);
            if(!field||field->planes.empty()||field->regions.empty()){FailNative("cpu_shape",E_INVALIDARG);return;}
        }catch(const std::bad_alloc&){FailNative(failure_phase,E_OUTOFMEMORY);return;}catch(...){FailNative(failure_phase,E_INVALIDARG);return;}
        auto make=[&](ID3D11Texture2D**out,const auto&values,DXGI_FORMAT format)->HRESULT{
            using T=typename std::decay_t<decltype(values)>::value_type;constexpr unsigned width=4096;
            if(!device||values.empty()||values.size()>std::size_t(width)*D3D11_REQ_TEXTURE2D_U_OR_V_DIMENSION||field->regions.size()>0x1000000u)return E_INVALIDARG;
            try{std::vector<T> padded(((values.size()+width-1)/width)*width);std::copy(values.begin(),values.end(),padded.begin());
                D3D11_TEXTURE2D_DESC d={};d.Width=width;d.Height=unsigned(padded.size()/width);d.MipLevels=1;d.ArraySize=1;d.Format=format;
                d.SampleDesc.Count=1;d.Usage=D3D11_USAGE_IMMUTABLE;d.BindFlags=D3D11_BIND_SHADER_RESOURCE;
                D3D11_SUBRESOURCE_DATA s={};s.pSysMem=padded.data();s.SysMemPitch=width*sizeof(T);return device->CreateTexture2D(&d,&s,out);
            }catch(...){return E_OUTOFMEMORY;}};
        HRESULT hr=make(&plane_surface,field->planes,DXGI_FORMAT_R32G32B32A32_FLOAT);if(SUCCEEDED(hr))hr=make(&region_surface,field->regions,DXGI_FORMAT_R32G32_UINT);
        if(FAILED(hr)){FailNative("native_texture",hr);return;}charge->Reduce(bounds.retained);failure_phase="complete";native_result=hr;native.store(2,std::memory_order_release);
    }
    bool Register(ID3D11Device*device,std::uint64_t epoch){
        if(native.load(std::memory_order_acquire)!=2||retired||epoch!=device_epoch||device!=device_identity)return false;if(registered)return true;
        auto bind=[](ref_texture&t,const xr_string&name,ID3D11Texture2D*s){if(!s)return false;t.create(name.c_str());if(!t)return false;
            t->surface_set(s);t->flags.bLoaded=true;t->PostLoad();return true;};
        registered=bind(plane_texture,plane_name,plane_surface)&&bind(region_texture,region_name,region_surface);return registered;
    }
};
namespace PainterWallmarkOwnershipCache {
inline std::vector<std::shared_ptr<PainterWallmarkOwnershipGpu>>&entries(){static auto*p=new std::vector<std::shared_ptr<PainterWallmarkOwnershipGpu>>;return *p;}
inline void Collect(){auto&v=entries();v.erase(std::remove_if(v.begin(),v.end(),[](const auto&p){return p.use_count()==1;}),v.end());}
inline std::shared_ptr<PainterWallmarkOwnershipGpu>Prepare(const std::shared_ptr<const gfy_packed::Mesh>&mesh,std::uint64_t epoch,bool force=false){
    if(!mesh||(!mesh->skin_ownership&&!force)||!epoch)return {};
    // Exact immutable shared-owner identity: O(entries), no triangle walk on the owner.
    for(auto&p:entries())if(!p->retired&&p->native.load(std::memory_order_acquire)!=3&&p->device_epoch==epoch&&p->force==force&&
        !p->mesh_identity.owner_before(mesh)&&!mesh.owner_before(p->mesh_identity)&&p->mesh_identity.lock().get()==mesh.get()){p->last_frame=Device.dwFrame;return p;}
    Collect();
    // The new worker reserves from the shared field budget. Release only cache-owned
    // channel entries here, on the renderer owner, before it can refuse that reservation.
    // Current, queued and fenced-retired packets have another reference and survive.
    PainterChannelCache::EvictAllUnpinned();
    auto p=std::make_shared<PainterWallmarkOwnershipGpu>();p->pending_mesh=mesh;p->mesh_identity=mesh;p->force=force;
    p->device_epoch=epoch;p->last_frame=Device.dwFrame;static std::uint64_t serial=0;const auto id=++serial;char name[128];
    xr_sprintf(name,"$user$painter_decal_owner_%llu_planes",id);p->plane_name=name;xr_sprintf(name,"$user$painter_decal_owner_%llu_regions",id);p->region_name=name;
    entries().push_back(p);return p;
}
inline void Clear(ID3D11Device*keep=nullptr){for(auto&p:entries())if(!keep||(p->device_identity&&p->device_identity!=keep))p->retired=true;Collect();}
}
#endif
