#pragma once

// A row visits successive damage-cell Z coordinates. Both noise octaves share
// their X/Y smoothing between those cells, and share the two XY-interpolated
// faces for every Z in one lattice interval. Interpolation order stays XYZ.
struct CraterNoiseRow {
    using Function=head_burst_record::Function;
    using Octave=head_burst_record::Octave;
    struct Slice {
        double px{},py{},bottom[3]{},top[3]{};
        int iz{};
        bool cached=false;

        void at(const Octave* oct,const Vec& p,double out[3]){
            const double z=p.z*oct->k;const int Z=Octave::ffloor(z);
            const double fz=head_burst_record::smooth(z-Z);
            if(!cached||px!=p.x||py!=p.y||iz!=Z){
                const double x=p.x*oct->k,y=p.y*oct->k;
                const int X=Octave::ffloor(x),Y=Octave::ffloor(y);
                const int cx=X-oct->lo[0],cy=Y-oct->lo[1],cz=Z-oct->lo[2];
                const int* size=oct->n;
                if(oct->v.empty()||cx<0||cy<0||cz<0||cx+1>=size[0]||cy+1>=size[1]||cz+1>=size[2]){
                    Function::at3(oct,p.x,p.y,p.z,out);
                    return;
                }
                px=p.x;py=p.y;iz=Z;cached=true;
                const double fx=head_burst_record::smooth(x-X),fy=head_burst_record::smooth(y-Y);
                const std::size_t line=size[0],plane=line*size[1];
                const auto base=(std::size_t(cz)*size[1]+cy)*line+cx;
                for(int ch=0;ch<3;++ch){
                    const double* a=oct[ch].v.data()+base,*b=a+plane;
                    const double a0=a[0]+(a[1]-a[0])*fx,a1=a[line]+(a[line+1]-a[line])*fx;
                    const double b0=b[0]+(b[1]-b[0])*fx,b1=b[line]+(b[line+1]-b[line])*fx;
                    bottom[ch]=a0+(a1-a0)*fy;
                    top[ch]=b0+(b1-b0)*fy;
                }
            }
            for(int ch=0;ch<3;++ch)out[ch]=bottom[ch]+(top[ch]-bottom[ch])*fz;
        }
    } octave[2];

    void warp(const Function& f,const Vec& p,double out[3]){
        double a[3],b[3];octave[0].at(f.o,p,a);octave[1].at(f.o+3,p,b);
        for(int ch=0;ch<3;++ch)out[ch]=p[ch]+f.A1*a[ch]+f.A2*b[ch];
    }
    unsigned label(const Function& f,const Vec& p){
        const Vec d=p-Vec{f.B[0],f.B[1],f.B[2]};if(dot(d,d)>f.Rcut*f.Rcut)return 0;
        double q[3];warp(f,p,q);double best=1e30;unsigned result=0;
        for(unsigned i=0;i<f.S;++i){
            const double a=q[0]-f.site[i][0],b=q[1]-f.site[i][1],c=q[2]-f.site[i][2],dd=a*a+b*b+c*c;
            if(dd<best){best=dd;result=f.group[i];}
        }
        return result;
    }
};
