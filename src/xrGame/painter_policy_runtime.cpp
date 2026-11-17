// Portable, float32 policy inference and angle-target servo.
// No physics or neural framework dependencies; training/runtime must share the contract.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#define API extern "C"
namespace {
thread_local std::string error;
struct Policy {
    uint32_t obs, h1, h2, acts, flags;
    std::vector<float> mean, inv, w1, b1, w2, b2, w3, b3;
};
bool read(std::ifstream& f, void* p, size_t size) { return bool(f.read(static_cast<char*>(p),size)); }
bool weights(std::ifstream& f, std::vector<float>& v, size_t n) {
    v.resize(n); if(!read(f,v.data(),n*sizeof(float)))return false;
    for(float x:v)if(!std::isfinite(x))return false;
    return true;
}
void layer(const std::vector<float>& x,const std::vector<float>& w,const std::vector<float>& b,std::vector<float>& y) {
    y.resize(b.size());
    for(size_t i=0;i<b.size();i++) {
        float sum=b[i]; for(size_t j=0;j<x.size();j++)sum+=w[i*x.size()+j]*x[j];
        y[i]=std::tanh(sum);
    }
}
}
API const char* ar_policy_error() { return error.c_str(); }
API void* ar_policy_load(const char* path,const char* expected_contract,int allow_test_only) {
    error.clear();
    if(!path||!expected_contract||std::strlen(expected_contract)!=64) { error="Missing expected contract hash";return nullptr; }
    std::ifstream f(path,std::ios::binary); char magic[8],hash[64]; uint32_t header[6];
    if(!read(f,magic,8)||std::memcmp(magic,"ARMLP002",8)||!read(f,header,sizeof(header))||!read(f,hash,64)) {
        error="Invalid or truncated policy header";return nullptr;
    }
    if(std::memcmp(hash,expected_contract,64)) {error="Policy contract mismatch";return nullptr;}
    if(header[0]!=2||header[1]<1||header[1]>4096||header[2]<1||header[2]>1024||header[3]<1||header[3]>1024||header[4]<1||header[4]>256||(header[5]&~1u)) {
        error="Unsupported policy dimensions/version/flags";return nullptr;
    }
    if((header[5]&1u)&&!allow_test_only) {error="Numerical-test policy cannot drive a game shell";return nullptr;}
    auto p=std::make_unique<Policy>();p->obs=header[1];p->h1=header[2];p->h2=header[3];p->acts=header[4];p->flags=header[5];
    if(!weights(f,p->mean,p->obs)||!weights(f,p->inv,p->obs)||!weights(f,p->w1,p->h1*p->obs)||!weights(f,p->b1,p->h1)||!weights(f,p->w2,p->h2*p->h1)||!weights(f,p->b2,p->h2)||!weights(f,p->w3,p->acts*p->h2)||!weights(f,p->b3,p->acts)||f.peek()!=std::ifstream::traits_type::eof()) {
        error="Invalid policy payload";return nullptr;
    }
    for(float x:p->inv)if(x<=0){error="Invalid observation normalizer";return nullptr;}
    return p.release();
}
API void ar_policy_destroy(void* raw) {delete static_cast<Policy*>(raw);}
API int ar_policy_observations(void* raw) {return raw?static_cast<Policy*>(raw)->obs:-1;}
API int ar_policy_actions(void* raw) {return raw?static_cast<Policy*>(raw)->acts:-1;}
API int ar_policy_infer(void* raw,const float* input,int n,float* output,int capacity) {
    auto p=static_cast<Policy*>(raw);
    if(!p||!input||!output||n!=p->obs||capacity<p->acts){error="Invalid policy input/output dimensions";return -1;}
    std::vector<float> x(n),h1,h2,y;
    for(int i=0;i<n;i++) {
        if(!std::isfinite(input[i])){error="Nonfinite policy observation";return -1;}
        x[i]=std::clamp((input[i]-p->mean[i])*p->inv[i],-10.f,10.f);
    }
    layer(x,p->w1,p->b1,h1);layer(h1,p->w2,p->b2,h2);layer(h2,p->w3,p->b3,y);
    for(size_t i=0;i<y.size();i++){if(!std::isfinite(y[i])){error="Nonfinite policy output";return -1;}output[i]=y[i];}
    return 0;
}
// A source axis containing zero maps [-1,1] to [low,high]. Caller preserves
// target state and invokes this once per policy tick; velocity is per physics tick.
API int ar_policy_target(float action,float low,float high,float previous,float smoothing,float* target) {
    if(!target||!std::isfinite(action)||!std::isfinite(low)||!std::isfinite(high)||!std::isfinite(previous)||!std::isfinite(smoothing)||low>0||high<0||low>high||smoothing<0||smoothing>1)return -1;
    action=std::clamp(action,-1.f,1.f);
    float desired=action>=0?action*high:-action*low;
    *target=smoothing*previous+(1.f-smoothing)*desired;return 0;
}
// Measured-shell contract: signed residual from a fixed captured reference.
// This is distinct from the historical zero-centered ar_policy_target ABI.
API int ar_policy_target_reference(float action,float low,float reference,float high,
    float previous,float smoothing,float* target) {
    if(!target||!std::isfinite(action)||!std::isfinite(low)||!std::isfinite(reference)||
        !std::isfinite(high)||!std::isfinite(previous)||!std::isfinite(smoothing)||
        low>reference||reference>high||smoothing<0||smoothing>1)return -1;
    action=std::clamp(action,-1.f,1.f);
    const float desired=reference+action*(action>=0?high-reference:reference-low);
    *target=smoothing*previous+(1.f-smoothing)*desired;
    return 0;
}
API int ar_policy_velocity(float target,float angle,float rate,float kp,float kd,float cap,float* velocity) {
    if(!velocity||!std::isfinite(target)||!std::isfinite(angle)||!std::isfinite(rate)||!std::isfinite(kp)||!std::isfinite(kd)||!std::isfinite(cap)||kp<0||kd<0||cap<=0)return -1;
    *velocity=std::clamp(kp*(target-angle)-kd*rate,-cap,cap);return 0;
}

// xray-root-relative-observation-v2. States are COM xyz, quaternion wxyz,
// world linear velocity xyz, world angular velocity xyz, in stable body order.
// Contacts and Euler rates must come from the same completed physics tick.
API int ar_policy_observe(const float* states,int bodies,int root,
    const float* angles,const float* rates,const float* targets,int joints,
    const int* contacts,const float* previous,int actions,float* output,int capacity) {
    if(!states||!angles||!rates||!targets||!contacts||!previous||!output||
        bodies<1||bodies>512||joints<1||joints>512||actions<1||actions>256||root<0||root>=bodies)
        {error="Invalid observation inputs";return -1;}
    const int size=14*bodies+9*joints+4+actions;
    if(capacity<size){error="Observation output too small";return -1;}
    for(int i=0;i<13*bodies;i++)if(!std::isfinite(states[i])){error="Nonfinite body state";return -1;}
    for(int i=0;i<3*joints;i++)if(!std::isfinite(angles[i])||!std::isfinite(rates[i])||!std::isfinite(targets[i])){error="Nonfinite joint state";return -1;}
    for(int i=0;i<actions;i++)if(!std::isfinite(previous[i])){error="Nonfinite previous action";return -1;}
    for(int i=0;i<bodies;i++) {
        const float* q=states+13*i+3;
        const float norm=q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
        if(std::abs(norm-1)>0.001f){error="Body quaternion is not normalized";return -1;}
    }
    const float* r=states+13*root;
    const float w=r[3],x=-r[4],y=-r[5],z=-r[6];
    auto rotate=[&](float vx,float vy,float vz,float* out) {
        float tx=2*(y*vz-z*vy),ty=2*(z*vx-x*vz),tz=2*(x*vy-y*vx);
        out[0]=vx+w*tx+y*tz-z*ty;out[1]=vy+w*ty+z*tx-x*tz;out[2]=vz+w*tz+x*ty-y*tx;
    };
    int cursor=0;
    for(int i=0;i<bodies;i++){const float* s=states+13*i;rotate(s[0]-r[0],s[1]-r[1],s[2]-r[2],output+cursor);cursor+=3;}
    for(int i=0;i<bodies;i++) {
        const float* q=states+13*i+3;
        float v[4]={w*q[0]-x*q[1]-y*q[2]-z*q[3],w*q[1]+x*q[0]+y*q[3]-z*q[2],
            w*q[2]+y*q[0]+z*q[1]-x*q[3],w*q[3]+z*q[0]+x*q[2]-y*q[1]};
        float sign=v[0]<0?-1.f:1.f;for(float a:v)output[cursor++]=a*sign;
    }
    for(int i=0;i<bodies;i++){const float* s=states+13*i;rotate(s[7]/5,s[8]/5,s[9]/5,output+cursor);cursor+=3;}
    for(int i=0;i<bodies;i++){const float* s=states+13*i;rotate(s[10]/10,s[11]/10,s[12]/10,output+cursor);cursor+=3;}
    rotate(0,-1,0,output+cursor);cursor+=3;output[cursor++]=r[1];
    for(int i=0;i<3*joints;i++)output[cursor++]=angles[i];
    for(int i=0;i<3*joints;i++)output[cursor++]=rates[i]/10;
    for(int i=0;i<3*joints;i++)output[cursor++]=targets[i];
    for(int i=0;i<bodies;i++)output[cursor++]=contacts[i]>0?1.f:0.f;
    for(int i=0;i<actions;i++)output[cursor++]=previous[i];
    return cursor;
}
