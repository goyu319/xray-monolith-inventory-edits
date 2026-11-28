#pragma once
#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>
namespace painter_burn {
// Canonical original MTL texture from the model's source manifest. Caller verifies source SHA and face mapping.
enum class MaterialRole : unsigned { Unknown, Cloth, Skin, Hard, ExposedTissue, Backing };
inline bool atlasEquals(std::string_view a, std::string_view b) {
    if(a.size() != b.size()) return false;
    for(std::size_t i=0;i<a.size();++i) {
        char c=a[i]; if(c>='A' && c<='Z') c+=32; if(c=='\\') c='/';
        if(c!=b[i])return false;
    }
    return true;
}
inline MaterialRole sourceMaterialRole(std::string_view atlas) {
    for(auto known:{"gscfokya/mil/mil_up_low_1","gscfokya/green_kapush","act/newloner/sunrise/spetsnaz_pants_a0_col",
                    "act/newloner/upper_d", "act/newloner/sunrise/rus_upper_d", "act/newloner/sunrise/sunrise_vest_col", "act/newmilitary/spetsnaz_tactical_vest_a0"})
        if(atlasEquals(atlas,known))return MaterialRole::Cloth;
    for(auto known:{"act/act_head_5443_14","act/newloner/hhl_m_wolf_light_co"})
        if(atlasEquals(atlas,known))return MaterialRole::Skin;
    for(auto known:{"gscfokya/mil/mil_helmet_1","nestandart/mil/glass_skat","act/newloner/sunrise/mp_kneepads_01_col"})
        if(atlasEquals(atlas,known))return MaterialRole::Hard;
    // Every other texture a body wears: by what it is called. (With only the textures above known, nearly every outfit
    // in the game was "unknown" - it could get a little darker in a fire, never burn through, blister or char.)
    std::string low(atlas);
    for(char& c:low){if(c>='A'&&c<='Z')c+=32;if(c=='\\')c='/';}
    const auto slash=low.rfind('/');
    const std::string_view leaf=slash==std::string::npos?std::string_view(low):std::string_view(low).substr(slash+1);
    if(leaf.empty())return MaterialRole::Unknown;
    const auto has=[&](const char* w){return leaf.find(w)!=std::string_view::npos;};
    // what sits on a head and is no skin
    for(auto w:{"mask","glass","helm","hair","beard","teeth","eye","lens","visor","goggle","nvg","phones","ochk"})
        if(has(w))return MaterialRole::Hard;
    for(auto w:{"head","face","hhl","skin","hand","t_fac_"})
        if(has(w))return MaterialRole::Skin;
    // gear with nothing under it: it soots over, it does not open
    for(auto w:{"exo","nvc","wpn","weapon","item","boot","shoe","pack","ryukza","rukz","podsum","grudak","gear","loadout",
                "obves","meloch","servo","armr","nosorog","bulat","glove","perchat","balon","ballon","tank","metal","kneepad",
                "plate","armor","armour","holster","kobur","kabur","belt","strap","knife","radio","pda","bags","atache"})
        if(has(w))return MaterialRole::Hard;
    return MaterialRole::Cloth;
}
struct MaterialUV {float x,y;};
inline MaterialUV burnChart(float x,float y,float z,float nx,float ny,float nz){
    if(std::abs(nz)>=std::abs(nx) && std::abs(nz)>=std::abs(ny))return {nz<0?-x:x,y};
    if(std::abs(nx)>=std::abs(ny))return {nx<0?z:-z,y};
    return {x,ny<0?-z:z};
}
namespace cloth_math {
inline float frac(float x){return x-std::floor(x);}
inline float sat(float x){return std::clamp(x,0.f,1.f);}
inline float smooth(float a,float b,float x){float t=sat((x-a)/(b-a));return t*t*(3.f-2.f*t);}
inline float mix(float a,float b,float t){return a+(b-a)*t;}
inline float hash(float x,float y) {
    float qx=frac(x*.1031f),qy=frac(y*.1030f),qz=frac(x*.0973f);
    float a=qx*(qy+33.33f)+qy*(qz+33.33f)+qz*(qx+33.33f);
    qx+=a;qy+=a;qz+=a;
    return frac((qx+qy)*qz);
}
inline float yarnWave(float a){
    float t=frac(a*.3183098861837907f);
    float x=std::min(t,1.f-t)*3.141592653589793f,x2=x*x;
    return x*(1.f+x2*(-.1666666667f+x2*(.0083333333f+x2*(-.0001984127f+x2*.0000027557f))));
}
inline float noise(float x,float y){
    float ix=std::floor(x),iy=std::floor(y),a=frac(x),b=frac(y);
    a=a*a*(3.f-2.f*a);b=b*b*(3.f-2.f*b);
    return mix(mix(hash(ix,iy),hash(ix+1,iy),a),mix(hash(ix,iy+1),hash(ix+1,iy+1),a),b);
}
}
// Exact same coverage computation as pb_surface(PB_CLOTH) wherever a certified exterior is directly sampled.
// Other domains and unknown apparel retain their authored geometry.
inline float clothEdge(MaterialUV uv,float dose) {
    using namespace cloth_math;
    const float x=uv.x,y=uv.y,low=noise(x*33.f,y*33.f),mid=noise(x*107.f+low,y*107.f+low);
    const float grain=noise(x*720.f+mid,y*720.f+mid);
    const float wx=x+.003f*(low-.5f),wy=y+.003f*(mid-.5f);
    const float sx=yarnWave(wx*2600.f+mid*2.f);
    const float sy=yarnWave(wy*3040.f+low*2.f);
    const float thread=std::max(1.f-smooth(.12f,.36f,sx),1.f-smooth(.13f,.39f,sy));
    return sat(dose)-(.625f+.120f*(low-.5f)+.073f*(mid-.5f)+.035f*(grain-.5f))
           -thread*(.010f+.060f*noise(x*190.f+7.f,y*190.f+7.f));
}
inline bool clothOpen(MaterialRole role,MaterialUV uv,float priorDose){
    return role==MaterialRole::Cloth && clothEdge(uv,priorDose)>0.f;
}
}
