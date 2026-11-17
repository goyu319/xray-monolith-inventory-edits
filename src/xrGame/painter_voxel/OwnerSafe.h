#pragma once
// ownersafe (2026-11-15): test-only fault switches for the gear (accessory) fallbacks and the owner's failure handling. Probe builds
// that define GFY_OWNERSAFE_FAULT read GFY_OWNERSAFE_FAULT=name[@n],... and fail the named check once, the n-th time it is reached
// (default: the first). The game build does not define it: every switch is the constant false and compiles to nothing.
#ifdef GFY_OWNERSAFE_FAULT
#include <cstdlib>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
inline bool painter_ownersafe_fault(const char*name){
 static std::mutex m;static std::map<std::string,long> left;static bool parsed=false;std::lock_guard<std::mutex> lock(m);
 if(!parsed){parsed=true;if(const char*e=std::getenv("GFY_OWNERSAFE_FAULT")){const std::string all(e);std::size_t at=0;
   while(at<all.size()){std::size_t c=all.find(',',at);if(c==std::string::npos)c=all.size();const std::string one=all.substr(at,c-at);at=c+1;
    if(one.empty())continue;const auto q=one.find('@');left[one.substr(0,q)]=q==std::string::npos?1:std::atol(one.c_str()+q+1);}}}
 const auto it=left.find(name);if(it==left.end()||it->second<=0)return false;
 if(--it->second)return false;
 std::cout<<"OWNERSAFE_FAULT fired="<<name<<std::endl;return true;}
#else
constexpr bool painter_ownersafe_fault(const char*){return false;}
#endif
