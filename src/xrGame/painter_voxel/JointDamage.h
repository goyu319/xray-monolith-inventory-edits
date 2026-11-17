#pragma once
// Geometry measurement only: no health, ammo or severing policy. Regions use
// the immutable anatomy lattice, independent of wound-cell quality. Compressed
// spans avoid an eightfold memory increase at High. Only blocks overlapping
// allocated removal tiles need fine-grained overlap integration after a hit.
struct JointSpan {int x{},y{},lo{},hi{};bool bone{};};
struct JointBlock {Cell key;std::vector<JointSpan>spans;};
int mask_only_mode_value() noexcept; // gfy_live (LiveDamageWorker.cpp): speed step, value 3's joint sums
struct JointMeasure {
    std::uint16_t bone{};
    double tissue{},skeleton{},pitch{};Vec origin;
    std::vector<JointBlock> blocks;
    // speed step (2026-11-12): the same additions in the same order, faster. A grid without ownership (removed(c) = the tile bit of a
    // valid cell): the block's grid cells are read once into a bit box (a block without a removed cell adds nothing), the spans'
    // x / y conversions are made once per block column, and every cell of a span is looked up in the box (a cell outside it: the
    // former lookup). The cranial measures took 18 / 11 ms after a head burst.
    template<class Emit>
    void evaluateBlock(const Grid&grid,const JointBlock&block,const Emit&emit)const {
        const Vec blockLo=origin+Vec{double(block.key.x*8),double(block.key.y*8),double(block.key.z*8)}*pitch;
        int lo[3],hi[3];
        for(int a=0;a<3;++a){lo[a]=std::max(0,int(std::floor((blockLo[a]-grid.origin[a])/grid.pitch+1e-7)))/8;
            hi[a]=std::min(grid.size[a]-1,int(std::ceil((blockLo[a]+8*pitch-grid.origin[a])/grid.pitch-1e-7))-1)/8;}
        bool touched=false;
        for(int x=lo[0];x<=hi[0]&&!touched;++x)for(int y=lo[1];y<=hi[1]&&!touched;++y)for(int z=lo[2];z<=hi[2]&&!touched;++z)
            touched=grid.tiles.find({x,y,z})!=grid.tiles.end();
        if(!touched)return;
        core::TileCache removedBits(grid); // R68: Grid::removed through cached tile pointers
        auto spanCells=[&](const JointSpan&span,const Vec&lower,const Vec&upper,const int first[3],const int last[3],const auto&isRemoved){
            // volume = ((1 * ox) * oy) * oz as before: the x / y overlaps (cellLo.x = origin.x + x * pitch, one per x / y) made once per column
            for(int x=first[0];x<=last[0];++x){const double cx=grid.origin[0]+double(x)*grid.pitch,ox=std::max(0.,std::min(upper[0],cx+grid.pitch)-std::max(lower[0],cx));
                for(int y=first[1];y<=last[1];++y){const double cy=grid.origin[1]+double(y)*grid.pitch,oy=std::max(0.,std::min(upper[1],cy+grid.pitch)-std::max(lower[1],cy));
                    double wxy=1;wxy*=ox;wxy*=oy;
                    for(int z=first[2];z<=last[2];++z){
                        if(!isRemoved(x,y,z))continue;
                        const double cz=grid.origin[2]+double(z)*grid.pitch;double volume=wxy;volume*=std::max(0.,std::min(upper[2],cz+grid.pitch)-std::max(lower[2],cz));
                        emit(volume,span.bone);}}}};
        auto convert=[&](const JointSpan&span,Vec&lower,Vec&upper,int first[3],int last[3]){
            lower=origin+Vec{double(span.x),double(span.y),double(span.lo)}*pitch;
            upper=lower+Vec{pitch,pitch,(span.hi-span.lo)*pitch};
            for(int a=0;a<3;++a){first[a]=int(std::floor((lower[a]-grid.origin[a])/grid.pitch+1e-7));
                last[a]=int(std::ceil((upper[a]-grid.origin[a])/grid.pitch-1e-7))-1;}};
        if(!grid.ownership){
            int c0[3],c1[3];for(int a=0;a<3;++a){c0[a]=std::max(0,int(std::floor((blockLo[a]-grid.origin[a])/grid.pitch+1e-7)));
                c1[a]=std::min(grid.size[a]-1,int(std::ceil((blockLo[a]+8*pitch-grid.origin[a])/grid.pitch-1e-7))-1);}
            if(c0[0]>c1[0]||c0[1]>c1[1]||c0[2]>c1[2])return; // no valid cell under the block: nothing removed there
            const int nx=c1[0]-c0[0]+1,ny=c1[1]-c0[1]+1,nz=c1[2]-c0[2]+1;
            if(std::size_t(nx)*ny*nz<=4096){
                std::uint64_t box[64]={},column[64]={};bool any=false; // column: a grid column (x, y) of the box holds a removed cell
                for(int x=c0[0];x<=c1[0];++x)for(int y=c0[1];y<=c1[1];++y)for(int z=c0[2];z<=c1[2];){const int ze=std::min(c1[2],z|7);
                    if(const auto*t=removedBits.find(Grid::tileKey({x,y,z}))){const unsigned row=unsigned((t->removed[std::size_t(x&7)]>>((y&7)*8))&255u);
                        if(row&(0xFFu<<(z&7))&(0xFFu>>(7-(ze&7))))for(int k=z;k<=ze;++k)if((row>>(k&7))&1u){
                            const std::size_t c=std::size_t(x-c0[0])*ny+std::size_t(y-c0[1]),i=c*nz+std::size_t(k-c0[2]);
                            box[i>>6]|=std::uint64_t(1)<<(i&63);column[c>>6]|=std::uint64_t(1)<<(c&63);any=true;}}
                    z=ze+1;}
                if(!any)return;
                auto isRemoved=[&](int x,int y,int z){
                    if(x<c0[0]||x>c1[0]||y<c0[1]||y>c1[1]||z<c0[2]||z>c1[2])return removedBits.removed({x,y,z});
                    const std::size_t i=(std::size_t(x-c0[0])*ny+std::size_t(y-c0[1]))*nz+std::size_t(z-c0[2]);return ((box[i>>6]>>(i&63))&1u)!=0;};
                // a span's grid x / y range (the former conversion of its lower / upper x and y, once per block column): a span whose
                // grid columns all lie in the box and hold no removed cell adds nothing
                int fx[8],lx[8],fy[8],ly[8];bool hx[8]={},hy[8]={};
                for(const auto&span:block.spans){const int ix=span.x-block.key.x*8,iy=span.y-block.key.y*8;
                    if(ix>=0&&ix<8&&iy>=0&&iy<8){
                        if(!hx[ix]){hx[ix]=true;const double l=origin[0]+double(span.x)*pitch,u=l+pitch;
                            fx[ix]=int(std::floor((l-grid.origin[0])/grid.pitch+1e-7));lx[ix]=int(std::ceil((u-grid.origin[0])/grid.pitch-1e-7))-1;}
                        if(!hy[iy]){hy[iy]=true;const double l=origin[1]+double(span.y)*pitch,u=l+pitch;
                            fy[iy]=int(std::floor((l-grid.origin[1])/grid.pitch+1e-7));ly[iy]=int(std::ceil((u-grid.origin[1])/grid.pitch-1e-7))-1;}
                        if(fx[ix]>=c0[0]&&lx[ix]<=c1[0]&&fy[iy]>=c0[1]&&ly[iy]<=c1[1]){bool hit=false;
                            for(int x=fx[ix];x<=lx[ix]&&!hit;++x)for(int y=fy[iy];y<=ly[iy];++y){const std::size_t c=std::size_t(x-c0[0])*ny+std::size_t(y-c0[1]);if((column[c>>6]>>(c&63))&1u){hit=true;break;}}
                            if(!hit)continue;}}
                    Vec lower,upper;int first[3],last[3];convert(span,lower,upper,first,last);spanCells(span,lower,upper,first,last,isRemoved);}
                return;}
        }
        auto isRemoved=[&](int x,int y,int z){return removedBits.removed({x,y,z});};
        for(const auto&span:block.spans){Vec lower,upper;int first[3],last[3];convert(span,lower,upper,first,last);spanCells(span,lower,upper,first,last,isRemoved);}
    }
    JointDamage evaluate(const Grid&grid)const {
        JointDamage result;result.bone=bone;result.original_m3=tissue;result.original_bone_m3=skeleton;
        auto sum=[&](double volume,bool isBone){result.removed_m3+=volume;if(isBone)result.removed_bone_m3+=volume;};
        static const std::size_t sideBlocks=[]{const char*e=std::getenv("GFY_MS_JOINT_SIDE_BLOCKS");return e?std::size_t(std::max(0,std::atoi(e))):std::size_t(2048);}();
        if(!grid.ownership&&sideBlocks&&blocks.size()>=sideBlocks&&mask_only_mode_value()==3){
            std::vector<double> part(2*blocks.size(),0.);std::atomic<std::size_t> next{0};
            auto work=[&]{for(std::size_t b;(b=next.fetch_add(1,std::memory_order_relaxed))<blocks.size();){double m=0,bo=0;
                evaluateBlock(grid,blocks[b],[&](double v,bool isBone){m+=v;if(isBone)bo+=v;});part[2*b]=m;part[2*b+1]=bo;}};
            std::vector<std::thread> helpers;std::exception_ptr failure[3];
            try{for(int h=0;h<3;++h)helpers.emplace_back([&,h]{core::enterPainterThread();try{work();}catch(...){failure[h]=std::current_exception();}});}catch(...){}
            std::exception_ptr own;try{work();}catch(...){own=std::current_exception();}
            for(auto&t:helpers)t.join();
            if(own)std::rethrow_exception(own);for(const auto&f:failure)if(f)std::rethrow_exception(f);
            for(std::size_t b=0;b<blocks.size();++b){result.removed_m3+=part[2*b];result.removed_bone_m3+=part[2*b+1];}}
        else for(const auto&block:blocks)evaluateBlock(grid,block,sum);
        result.removed_m3=std::min(result.original_m3,result.removed_m3);
        result.removed_bone_m3=std::min(result.original_bone_m3,result.removed_bone_m3);
        return result;
    }
};
std::vector<JointMeasure> prepareJointMeasures(const Grid&grid,const std::vector<JointRegion>&regions,unsigned bones){
    need(regions.size()<=64,"joint_region_count");if(regions.empty())return {};
    need(bool(grid.anatomy)&&bool(grid.material_slots),"joint_regions_require_anatomy");
    const auto&anatomy=*grid.anatomy;std::set<unsigned>seen;std::vector<JointMeasure>out;
    std::size_t visited=0,spanCount=0;
    for(const auto&region:regions){
        need(region.bone<bones&&seen.insert(region.bone).second,"joint_region_bone");
        const Vec center=vec(region.center),axis=unit(vec(region.axis));
        for(int a=0;a<3;++a)need(std::isfinite(center[a])&&std::abs(center[a])<100,"joint_region_center");
        // 2026-10-17: trunk rows (waist, chest) need a wider cylinder than a limb; the work budgets below bound the cost.
        need(std::isfinite(region.radius)&&region.radius>=.002&&region.radius<=.22&&
            std::isfinite(region.half_width)&&region.half_width>=.001&&region.half_width<=.03,"joint_region_dimensions");
        int lo[3],hi[3];for(int a=0;a<3;++a){
            const double extent=std::abs(axis[a])*region.half_width+std::sqrt(std::max(0.,1-axis[a]*axis[a]))*region.radius;
            lo[a]=std::max(0,int(std::floor((center[a]-extent-anatomy.origin[a])/anatomy.pitch)));
            hi[a]=std::min(int(anatomy.size[a])-1,int(std::floor((center[a]+extent-anatomy.origin[a])/anatomy.pitch)));
        }
        JointMeasure joint;joint.bone=region.bone;joint.pitch=anatomy.pitch;joint.origin={anatomy.origin[0],anatomy.origin[1],anatomy.origin[2]};
        const double cellVolume=anatomy.pitch*anatomy.pitch*anatomy.pitch;
        std::map<Cell,std::vector<JointSpan>>blocks;
        for(int x=lo[0];x<=hi[0];++x)for(int y=lo[1];y<=hi[1];++y){
            const auto column=std::size_t(y)*anatomy.size[0]+x;
            for(auto r=anatomy.offsets[column];r<anatomy.offsets[column+1];++r){const auto&run=anatomy.runs[r];
                need(run.label<grid.material_slots->size(),"joint_material_label");const bool skeleton=(*grid.material_slots)[run.label]==2;
                for(int z=std::max(lo[2],int(run.lo));z<=std::min(hi[2],int(run.hi)-1);++z){
                    need(++visited<=48000000,"joint_preparation_work_budget");
                    Vec delta=joint.origin+Vec{x+.5,y+.5,z+.5}*anatomy.pitch-center;double axial=dot(delta,axis);
                    if(std::abs(axial)>region.half_width||dot(delta,delta)-axial*axial>region.radius*region.radius)continue;
                    auto&spans=blocks[{x/8,y/8,z/8}];
                    if(!spans.empty()&&spans.back().x==x&&spans.back().y==y&&spans.back().hi==z&&spans.back().bone==skeleton)++spans.back().hi;
                    else{need(++spanCount<=4000000,"joint_span_budget");spans.push_back({x,y,z,z+1,skeleton});}
                    joint.tissue+=cellVolume;if(skeleton)joint.skeleton+=cellVolume;
                }
            }
        }
        for(auto&block:blocks)joint.blocks.push_back({block.first,std::move(block.second)});
        out.push_back(std::move(joint));
    }
    return out;
}