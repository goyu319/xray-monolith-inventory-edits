#pragma once
#include "PainterBurnState.h"
#include <map>
namespace painter_burn {
// One registry per original source lineage. The host must pass the authenticated
// original source-face ordinal or the actual birth identity of a generated wall.
// A triangle index in the current blended draw packet is insufficient.
class SurfaceIds {
    using Interface=std::pair<U64,U32>;
    std::map<Interface,U32> ids;
    U32 next=262144;
public:
    static constexpr unsigned Capacity=4096;
    static U32 exterior(U32 originalFace) {return originalFace<262143?originalFace+1:0;}
    U32 exposed(U64 birthSequence,U32 nativeInterface,bool create=false) {
        if(!birthSequence)return 0;
        auto k=Interface{birthSequence,nativeInterface};auto i=ids.find(k);
        if(i!=ids.end())return i->second;
        if(!create||ids.size()>=Capacity||next>MaxSurface)return 0;
        U32 token=next++;ids.emplace(k,token);return token;
    }
    std::vector<U32> save(U64 lineage)const{
        std::vector<U32> words={0x32444942u,U32(lineage),U32(lineage>>32),U32(ids.size())};
        for(auto& p:ids)words.insert(words.end(),{U32(p.first.first),U32(p.first.first>>32),p.first.second,p.second});
        return words;
    }
    bool restore(U64 lineage,const std::vector<U32>& words){
        if(!lineage||words.size()<4||words[0]!=0x32444942u||words[1]!=U32(lineage)||words[2]!=U32(lineage>>32)||
           words[3]>Capacity||words.size()!=4+4ull*words[3])return false;
        SurfaceIds v;std::array<bool,Capacity> used{};
        for(std::size_t i=4;i<words.size();i+=4){
            U64 birth=words[i]|(U64(words[i+1])<<32);U32 token=words[i+3];
            if(!birth||token<262144||token-262144>=words[3]||used[token-262144]||
               !v.ids.emplace(Interface{birth,words[i+2]},token).second)return false;
            used[token-262144]=true;
        }
        v.next+=words[3];*this=std::move(v);return true;
    }
};
}
