// Build a closed temporary tissue surface from the admitted physical component.
namespace flight_silhouette {
inline std::shared_ptr<const gfy_packed::Mesh> make(const SeparationPiece& piece) {
    if(!piece.seed || !piece.seed->grid || !piece.seed->topology || !piece.seed->asset || piece.root_bone>=64) return {};
    const auto& grid=*piece.seed->grid;
    const auto& topology=*piece.seed->topology;
    if(topology.components.size()<2 || !(grid.pitch>0)) return {};
    const auto& component=topology.components[1];
    int axis=0;
    for(int a=1;a<3;++a) if(component.hi[a]-component.lo[a]>component.hi[axis]-component.lo[axis]) axis=a;
    const int u=(axis+1)%3,v=(axis+2)%3;
    constexpr int arcs=12,maxRings=12;
    const int length=component.hi[axis]-component.lo[axis];
    if(length<=0) return {};
    const int count=std::min(maxRings,std::max(1,(length+3)/4));
    const int step=(length+count-1)/count;
    std::array<double,arcs> dx{},dy{};
    for(int a=0;a<arcs;++a){double angle=6.2831853071795864769*a/arcs;dx[a]=std::cos(angle);dy[a]=std::sin(angle);}
    struct Section {
        double weight=0, along=0, x=0,y=0,support[arcs]{};
        Section(){for(double& value:support)value=-1.e30;}
    };
    std::array<Section,maxRings> rings;
    for(const auto& col:topology.columns) for(auto i=col.first;i<col.first+col.count;++i){
        const auto& span=topology.spans[i]; if(span.component!=1)continue;
        int z=span.lo;
        while(z<span.hi){
            const int c[3]={col.x,col.y,z};
            const int band=(c[axis]-component.lo[axis])/step;
            if(band<0||band>=count)return {};
            const int end=axis==2?std::min<int>(span.hi,component.lo[axis]+(band+1)*step):int(span.hi);
            if(end<=z)return {};
            const double weight=end-z,mean[3]={col.x+.5,col.y+.5,.5*(z+end)};
            auto& sec=rings[band];sec.weight+=weight;sec.along+=weight*mean[axis];sec.x+=weight*mean[u];sec.y+=weight*mean[v];
            for(int a=0;a<arcs;++a){
                const double halfU=u==2?weight*.5:.5,halfV=v==2?weight*.5:.5;
                const double bound=mean[u]*dx[a]+mean[v]*dy[a]+std::abs(dx[a])*halfU+std::abs(dy[a])*halfV;
                sec.support[a]=std::max(sec.support[a],bound);
            }
            z=end;
        }
    }
    struct Sample{std::array<float,3> position;std::uint16_t bone;};
    std::vector<Sample> samples;
    bool positive=true,foundWinding=false;
    std::size_t total=0,index=0;for(const auto& ch:piece.mesh.children)total+=ch.vertices.size();
    const auto stride=std::max<std::size_t>(1,(total+255)/256);
    for(const auto& ch:piece.mesh.children){
        for(const auto& vertex:ch.vertices){if((index++%stride)!=0||vertex.surface[12]<=-999.f)continue;
            unsigned w=0;for(unsigned j=1;j<4;++j)if(vertex.weight[j]>vertex.weight[w])w=j;
            const auto b=vertex.bone[w];
            if(b>=64)continue;
            const auto bone=piece.pose_anchors.size()>b&&piece.pose_anchors[b]<64?piece.pose_anchors[b]:b;
            if(std::none_of(piece.elements.begin(),piece.elements.end(),[&](const SeparationElement& e){return e.bone==bone;}))continue;
            samples.push_back({{vertex.surface[0],vertex.surface[1],vertex.surface[2]},bone});
        }
        if(!foundWinding)for(std::size_t i=0;i+2<ch.indices.size();i+=3){
            const auto& a=ch.vertices[ch.indices[i]].surface;const auto& b=ch.vertices[ch.indices[i+1]].surface;const auto& c=ch.vertices[ch.indices[i+2]].surface;
            double e[3],f[3];for(int j=0;j<3;++j){e[j]=b[j]-a[j];f[j]=c[j]-a[j];}
            double d=0;for(int j=0;j<3;++j)d+=(e[(j+1)%3]*f[(j+2)%3]-e[(j+2)%3]*f[(j+1)%3])*a[j+3];
            if(std::abs(d)>1.e-12){positive=d>0;foundWinding=true;break;}
        }
    }
    auto out=std::make_shared<gfy_packed::Mesh>();gfy_packed::Child ch;
    ch.child_index=piece.seed->asset->original_children;ch.material=5;ch.material_source_child=0;
    const double origin[3]={grid.origin.x,grid.origin.y,grid.origin.z};
    auto vertex=[&](double h,double cx,double cy,double nx,double ny,double nz,double tu,double tv){
        const double n[3]={nx,ny,nz};double pos[3]{};pos[axis]=h;pos[u]=cx;pos[v]=cy;
        gfy_packed::Vertex row;
        double t[3]={-ny,nx,0};if(std::abs(nz)>.99){t[0]=1;t[1]=t[2]=0;}
        double tl=std::sqrt(t[0]*t[0]+t[1]*t[1]+t[2]*t[2]);
        for(int a=0;a<3;++a){t[a]/=tl;row.surface[a]=float(origin[a]+grid.pitch*pos[a]);row.surface[a+3]=float(n[a]);row.surface[a+6]=float(t[a]);}
        for(int a=0;a<3;++a)row.surface[a+9]=float(n[(a+1)%3]*t[(a+2)%3]-n[(a+2)%3]*t[(a+1)%3]);
        row.surface[12]=float(tu);row.surface[13]=float(tv);
        auto bone=piece.root_bone;double best=1.e30;
        for(const auto& s:samples){double d=0;for(int a=0;a<3;++a){double diff=s.position[a]-row.surface[a];d+=diff*diff;}if(d<best){best=d;bone=s.bone;}}
        row.bone={bone,bone,bone,bone};row.weight={0.f,0.f,0.f,1.f};ch.vertices.push_back(row);
        return std::uint16_t(ch.vertices.size()-1);
    };
    auto tri=[&](std::uint16_t a,std::uint16_t b,std::uint16_t c){ch.indices.push_back(a);ch.indices.push_back(positive?b:c);ch.indices.push_back(positive?c:b);ch.original_faces.push_back(~std::uint32_t(0));};
    std::uint16_t previous=0,first=0;bool any=false;double lastX=0,lastY=0;
    for(int r=0;r<count;++r){
        const auto& sec=rings[r];if(!(sec.weight>0))continue;
        const double cx=sec.x/sec.weight,cy=sec.y/sec.weight,h=sec.along/sec.weight;
        const auto base=std::uint16_t(ch.vertices.size());
        for(int a=0;a<arcs;++a){
            double radius=1.e30;for(int j=0;j<arcs;++j){
                const double cosAngle=dx[a]*dx[j]+dy[a]*dy[j];if(cosAngle<=1.e-6)continue;
                radius=std::min(radius,(sec.support[j]-cx*dx[j]-cy*dy[j])/cosAngle);
            }
            double norm[3]{};norm[u]=dx[a];norm[v]=dy[a];
            vertex(h,cx+radius*dx[a],cy+radius*dy[a],norm[0],norm[1],norm[2],double(a)/arcs,h*grid.pitch*8);
        }
        if(any)for(int a=0;a<arcs;++a){const int b=(a+1)%arcs;tri(previous+a,previous+b,base+b);tri(previous+a,base+b,base+a);}
        else {first=base;double norm[3]{};norm[axis]=-1;const auto tip=vertex(component.lo[axis],cx,cy,norm[0],norm[1],norm[2],.5,0);
            for(int a=0;a<arcs;++a)tri(tip,first+(a+1)%arcs,first+a);}
        any=true;previous=base;lastX=cx;lastY=cy;
    }
    if(!any)return {};
    double norm[3]{};norm[axis]=1;const auto cap=vertex(component.hi[axis],lastX,lastY,norm[0],norm[1],norm[2],.5,1);
    for(int a=0;a<arcs;++a)tri(cap,previous+a,previous+(a+1)%arcs);
    out->children.push_back(std::move(ch));return out;
}
}
