#include "stdafx.h"
#include "painter_balance_test.h"
#include "Entity_Alive.h"
#include "Hit.h"
#include "level.h"
#include "../xrCore/LocatorAPI.h"
#include "../Include/xrRender/Kinematics.h"
#include "../xrPhysics/PhysicsShell.h"
#include "../include/xrapi/xrapi.h"
class CGameMtlLibrary;
IC CGameMtlLibrary& GMLibrary() { VERIFY(PGMLib); return *PGMLib; }
#include "../xrPhysics/PHElement.h"
#include "../xrPhysics/PHJoint.h"
#include "../xrPhysics/PHIsland.h"
#include "../xrPhysics/console_vars.h"
#include <cmath>
#include <cstdio>
#include <vector>
#include <string>
#include <algorithm>
#include <memory>

extern "C" {
unsigned char* SHA256(const unsigned char*,size_t,unsigned char*);
void* ar_policy_load(const char*,const char*,int);
void ar_policy_destroy(void*);
const char* ar_policy_error();
int ar_policy_observations(void*);int ar_policy_actions(void*);
int ar_policy_observe(const float*,int,int,const float*,const float*,const float*,int,const int*,const float*,int,float*,int);
int ar_policy_infer(void*,const float*,int,float*,int);
int ar_policy_target_reference(float,float,float,float,float,float,float*);
int ar_policy_velocity(float,float,float,float,float,float,float*);
}
int painter_balance_test_enabled=0,painter_balance_test_active=0,painter_balance_test_inferences=0;
namespace {
struct JointDef { std::string name; Fvector low,high,reference;float force; };
struct BodyDef {std::string name;float mass;};
struct Config {
    void* model=nullptr;std::string visual,source_hash;int root=0,obs=0,actions=0;
    float duration=8;Fvector gravity;std::vector<BodyDef>bodies;std::vector<JointDef>joints;
    std::vector<std::pair<int,int>> free_axes;
    ~Config(){if(model)ar_policy_destroy(model);}
};
Config* config=nullptr;
std::vector<PainterBalanceTest*> sessions;
void policy_step(CPHIsland* island,bool after,bool exact){for(auto* session:sessions)session->step(island,after,exact);}
bool close_numeric(float a,float b,float tolerance=.0002f){return std::isfinite(a)&&std::isfinite(b)&&std::fabs(a-b)<=tolerance;}
bool hash_source(LPCSTR visual,const std::string& expected){
    string_path path;xr_strconcat(path,visual,".ogf");IReader*r=FS.r_open("$game_meshes$",path);if(!r)return false;
    unsigned char digest[32];bool ok=SHA256(static_cast<const unsigned char*>(r->pointer()),r->length(),digest)!=nullptr;FS.r_close(r);
    char hex[65]{};for(int i=0;i<32;++i)sprintf(hex+i*2,"%02x",digest[i]);return ok&&expected==hex;
}
bool keys(CInifile& ini,LPCSTR section,std::initializer_list<LPCSTR> names){for(auto n:names)if(!ini.line_exist(section,n))return false;return true;}
}
struct PainterBalanceTest::Impl {
    CPhysicsShell* shell=nullptr;CEntityAlive* owner=nullptr;bool running=false,first=true;
    unsigned ticks=0;float ground=0;FILE* trace=nullptr;
    std::vector<CPHElement*> bodies;std::vector<CPHJoint*> joints;
    std::vector<float> states,angles,rates,targets,previous_angles,actions,obs;
    std::vector<int> contacts;
    void stop(LPCSTR reason){
        if(!running)return;
        for(auto [j,a]:config->free_axes){
            if(shell && j<shell->get_JointsNumber() && shell->get_JointByStoreOrder(u16(j))==joints[j])
                joints[j]->SetForceAndVelocity(0.f,0.f,a);
        }
        Msg("[painter_balance_test] id=%u phase=stopped reason=%s ticks=%u",owner->ID(),reason,ticks);
        if(trace){fprintf(trace,"# end reason=%s ticks=%u\n",reason,ticks);fclose(trace);trace=nullptr;}
        running=false;--painter_balance_test_active;
    }
    int index(dBodyID b)const {for(size_t i=0;i<bodies.size();++i)if(bodies[i]->get_body()==b)return int(i);return -1;}
};
PainterBalanceTest::PainterBalanceTest():p_(new Impl){sessions.push_back(this);}
PainterBalanceTest::~PainterBalanceTest(){stop();sessions.erase(std::remove(sessions.begin(),sessions.end(),this),sessions.end());delete p_;}
void PainterBalanceTest::stop(){p_->stop("owner_release");p_->shell=nullptr;p_->owner=nullptr;}

bool painter_balance_test_configure(bool enabled){
    if(!enabled){painter_balance_test_enabled=0;for(auto*s:sessions)s->stop();return true;}
    if(config){painter_balance_test_enabled=1;return true;}
    string_path path;FS.update_path(path,"$game_config$","painter\\balance_test.ltx");if(!FS.exist(path)){Msg("[painter_balance_test] missing configuration");return false;}
    CInifile ini(path,TRUE,TRUE,FALSE);
    if(!keys(ini,"policy",{"contract","visual","source_sha256","bodies","joints","actions","observations","root","gravity","duration"}))return false;
    auto c=std::make_unique<Config>();c->visual=ini.r_string("policy","visual");c->source_hash=ini.r_string("policy","source_sha256");
    const int nb=ini.r_s32("policy","bodies"),nj=ini.r_s32("policy","joints");
    c->actions=ini.r_s32("policy","actions");c->obs=ini.r_s32("policy","observations");c->root=ini.r_s32("policy","root");
    c->gravity=ini.r_fvector3("policy","gravity");c->duration=ini.r_float("policy","duration");
    if(nb<1||nb>32||nj<1||nj>32||c->root<0||c->root>=nb||c->actions<1||c->actions>96||
        c->obs!=14*nb+9*nj+4+c->actions||!std::isfinite(c->duration)||c->duration<=0||c->duration>30||
        !hash_source(c->visual.c_str(),c->source_hash))return false;
    for(int i=0;i<nb;++i){string64 s;xr_sprintf(s,"body_%d",i);if(!keys(ini,s,{"name","mass"}))return false;
        BodyDef b{ini.r_string(s,"name"),ini.r_float(s,"mass")};if(!std::isfinite(b.mass)||b.mass<=0)return false;c->bodies.push_back(b);}
    for(int j=0;j<nj;++j){string64 s;xr_sprintf(s,"joint_%d",j);if(!keys(ini,s,{"name","force","low","high","reference"}))return false;
        JointDef d{ini.r_string(s,"name"),ini.r_fvector3(s,"low"),ini.r_fvector3(s,"high"),ini.r_fvector3(s,"reference"),ini.r_float(s,"force")};
        if(!std::isfinite(d.force)||d.force<=0||d.force>300)return false;
        for(int a=0;a<3;++a){if(!std::isfinite(d.low[a])||!std::isfinite(d.high[a])||!std::isfinite(d.reference[a])||d.low[a]>d.reference[a]||d.reference[a]>d.high[a])return false;
            if(d.high[a]-d.low[a]>1e-5f)c->free_axes.push_back({j,a});}c->joints.push_back(d);}
    if(c->free_axes.size()!=c->actions)return false;
    FS.update_path(path,"$game_config$","painter\\balance_test.bin");
    // Explicit test command alone allows a test-only artifact. Never clear its
    // flag or call it a production/standing-validated policy.
    c->model=ar_policy_load(path,ini.r_string("policy","contract"),1);
    if(!c->model){Msg("[painter_balance_test] load rejected: %s",ar_policy_error());return false;}
    if(ar_policy_observations(c->model)!=c->obs||ar_policy_actions(c->model)!=c->actions)return false;
    config=c.release();g_ph_island_step_policy=policy_step;painter_balance_test_enabled=1;
    Msg("[painter_balance_test] loaded experimental=1 visual=%s observations=%d actions=%d trained_standing_verified=0",config->visual.c_str(),config->obs,config->actions);return true;
}
bool PainterBalanceTest::start(CPhysicsShell* shell,CEntityAlive* owner){
    stop();if(!painter_balance_test_enabled||!config||!shell||!owner||owner->g_Alive())return false;
    auto reject=[&](LPCSTR s){Msg("[painter_balance_test] id=%u phase=rejected reason=%s",owner->ID(),s);return false;};
    if(!owner->cast_stalker()||config->visual!=owner->cNameVisual().c_str())return reject("untrained_visual");
    if(shell->get_ElementsNumber()!=config->bodies.size()||shell->get_JointsNumber()!=config->joints.size()||!close_numeric(ph_console::ph_step_time,.01f,1e-6f))return reject("layout_or_clock");
    auto*k=smart_cast<IKinematics*>(owner->Visual());if(!k)return reject("kinematics");
    auto&p=*p_;p.bodies.clear();p.joints.clear();
    for(size_t i=0;i<config->bodies.size();++i){auto*b=static_cast<CPHElement*>(shell->get_ElementByStoreOrder(u16(i)));if(!b||!b->get_body()||config->bodies[i].name!=k->LL_BoneName_dbg(b->m_SelfID))return reject("body_order");
        dMass mass;dBodyGetMass(b->get_body(),&mass);if(!close_numeric(mass.mass,config->bodies[i].mass,.002f))return reject("body_mass");p.bodies.push_back(b);}
    for(size_t j=0;j<config->joints.size();++j){auto*q=static_cast<CPHJoint*>(shell->get_JointByStoreOrder(u16(j)));if(!q||q->eType!=CPhysicsJoint::full_control||!q->GetDJoint1()||config->joints[j].name!=k->LL_BoneName_dbg(q->BoneID()))return reject("joint_order");
        for(int a=0;a<3;++a)if(!close_numeric(dJointGetAMotorParam(q->GetDJoint1(),dParamLoStop+a*dParamGroup),config->joints[j].low[a])||!close_numeric(dJointGetAMotorParam(q->GetDJoint1(),dParamHiStop+a*dParamGroup),config->joints[j].high[a]))return reject("joint_limits");p.joints.push_back(q);}
    Fvector origin;const dReal*r=dBodyGetPosition(p.bodies[config->root]->get_body());origin.set(r[0],r[1],r[2]);Fvector down;down.set(0,-1,0);collide::rq_result result;
    if(!Level().ObjectSpace.RayPick(origin,down,4.f,collide::rqtStatic,result,owner))return reject("ground_reference");
    p.ground=origin.y-result.range;p.shell=shell;p.owner=owner;p.ticks=0;p.first=true;
    p.states.assign(config->bodies.size()*13,0);p.angles.assign(config->joints.size()*3,0);p.rates=p.angles;p.previous_angles=p.angles;p.targets=p.angles;
    for(size_t j=0;j<config->joints.size();++j)for(int a=0;a<3;++a)p.targets[j*3+a]=config->joints[j].reference[a];
    p.actions.assign(config->actions,0);p.obs.assign(config->obs,0);p.contacts.assign(config->bodies.size(),0);
    string_path path,name;xr_sprintf(name,"painter_balance_test_%u_%u.csv",Device.dwTimeGlobal,owner->ID());FS.update_path(path,"$logs$",name);p.trace=fopen(path,"wb");
    if(p.trace){fprintf(p.trace,"# experimental policy=%s ground_y=%.9g\nstep",config->visual.c_str(),p.ground);for(int i=0;i<config->obs;++i)fprintf(p.trace,",obs%d",i);for(int i=0;i<config->actions;++i)fprintf(p.trace,",action%d",i);fputc('\n',p.trace);}
    p.running=true;++painter_balance_test_active;
    Msg("[painter_balance_test] id=%u phase=started inference_hz=25 servo_hz=100 old_controller=off trace=%s",owner->ID(),path);return true;
}
void PainterBalanceTest::hit(const SHit& hit){
    if(!p_->running)return;auto*k=smart_cast<IKinematics*>(p_->owner->Visual());if(!k||hit.bone()>=k->LL_BoneCount())return;
    const char*n=k->LL_BoneName_dbg(hit.bone());if(strstr(n,"head")||strstr(n,"neck")||strstr(n,"jaw")||hit.type()==ALife::eHitTypeExplosion)p_->stop("head_or_explosion_hit");
}
void PainterBalanceTest::step(CPHIsland* island,bool after,bool exact){
    auto&p=*p_;if(after||!p.running)return;
    bool contains_root=false;for(dxBody*b=island->firstbody;b;b=static_cast<dxBody*>(b->next))if(b==p.bodies[config->root]->get_body()){contains_root=true;break;}if(!contains_root)return;
    if(!painter_balance_test_enabled){p.stop("test_disabled");return;}
    if(!exact||!close_numeric(fixed_step,.01f,1e-6f)){p.stop("solver_or_step_mismatch");return;}
    if(p.ticks>=unsigned(config->duration*100)){p.stop("test_duration");return;}
    if(p.owner->getDestroy()||p.owner->g_Alive()||p.shell->get_ElementsNumber()!=p.bodies.size()||p.shell->get_JointsNumber()!=p.joints.size()){p.stop("owner_changed");return;}
    dVector3 gravity;dWorldGetGravity(island->DWorld(),gravity);for(int a=0;a<3;++a)if(!close_numeric(gravity[a],config->gravity[a],.001f)){p.stop("gravity_mismatch");return;}
    auto*k=smart_cast<IKinematics*>(p.owner->Visual());
    for(size_t i=0;i<p.bodies.size();++i){auto*b=p.bodies[i];if(p.shell->get_ElementByStoreOrder(u16(i))!=b||!k||!k->LL_GetBoneVisible(b->m_SelfID)){p.stop("dismemberment_or_body_changed");return;}
        dBodyID raw=b->get_body();const dReal*data[]={dBodyGetPosition(raw),dBodyGetQuaternion(raw),dBodyGetLinearVel(raw),dBodyGetAngularVel(raw)};int offset=0;for(int group=0;group<4;++group)for(int x=0;x<(group==1?4:3);++x){float v=data[group][x];if(!std::isfinite(v)){p.stop("nonfinite_body");return;}p.states[i*13+offset++]=v;}p.states[i*13+1]-=p.ground;}
    for(size_t j=0;j<p.joints.size();++j){if(p.shell->get_JointByStoreOrder(u16(j))!=p.joints[j]){p.stop("joint_changed");return;}for(int a=0;a<3;++a){const auto slot=j*3+a;float v=p.joints[j]->GetAxisAngle(a);if(!std::isfinite(v)){p.stop("nonfinite_joint");return;}const float delta=v-p.previous_angles[slot];p.rates[slot]=p.first?0.f:std::atan2(std::sin(delta),std::cos(delta))/.01f;p.angles[slot]=p.previous_angles[slot]=v;}}
    p.first=false;
    if(p.ticks%4==0){
        if(ar_policy_observe(p.states.data(),int(p.bodies.size()),config->root,p.angles.data(),p.rates.data(),p.targets.data(),int(p.joints.size()),p.contacts.data(),p.actions.data(),config->actions,p.obs.data(),config->obs)!=config->obs||ar_policy_infer(config->model,p.obs.data(),config->obs,p.actions.data(),config->actions)){p.stop("invalid_inference");return;}
        for(size_t n=0;n<config->free_axes.size();++n){auto[j,a]=config->free_axes[n];auto&d=config->joints[j];auto slot=j*3+a;if(ar_policy_target_reference(p.actions[n],d.low[a],d.reference[a],d.high[a],p.targets[slot],.5f,&p.targets[slot])){p.stop("invalid_target");return;}}
        ++painter_balance_test_inferences;
        if(p.trace){fprintf(p.trace,"%u",p.ticks);for(float v:p.obs)fprintf(p.trace,",%.9g",v);for(float v:p.actions)fprintf(p.trace,",%.9g",v);fputc('\n',p.trace);fflush(p.trace);}
    }
    for(auto[j,a]:config->free_axes){float velocity=0;if(ar_policy_velocity(p.targets[j*3+a],p.angles[j*3+a],p.rates[j*3+a],10,.6f,4,&velocity)){p.stop("invalid_velocity");return;}p.joints[j]->SetForceAndVelocity(config->joints[j].force,velocity,a);}
    // Carry actual static-floor contacts from this step into the NEXT observation.
    std::fill(p.contacts.begin(),p.contacts.end(),0);
    for(dxJoint*j=island->firstjoint;j;j=static_cast<dxJoint*>(j->next))if(dJointGetType(j)==dJointTypeContact){const auto*c=static_cast<dxJointContact*>(j);const auto&g=c->contact.geom;
        if(std::fabs(g.normal[1])<.5f||std::fabs(g.pos[1]-p.ground)>.1f)continue;
        if(j->node[0].body&&j->node[1].body)continue;
        int i=p.index(j->node[0].body?j->node[0].body:j->node[1].body);if(i>=0)++p.contacts[i];}
    ++p.ticks;
}
