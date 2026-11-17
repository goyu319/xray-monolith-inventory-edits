#pragma once
// Pure translation units only; engine no-EH headers use NodeRoute.h.
#include "NodeRoute.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
namespace painter { namespace combat { namespace engine {
// Graph's coordinate/link queries are bounded by shared work, including stale
// heap pops and reconstruction. Storage is O(expansion_limit), never level-size.
// Callers own quota across multiple routes; cancellation does not refund work.
template<class Graph>StaticRouteResult build_static_route(const Graph&g,const StaticRouteRequest&q,StaticRouteQuota&budget,const std::function<bool()>&cancelled={}){
    StaticRouteResult r;r.source=q;const auto before=budget.work_used;
    struct Cancel{};struct Limit{};struct Bad{};
    const auto poll=[&](){if(cancelled&&cancelled())throw Cancel{};};
    const auto use=[&](){poll();if(!budget.work_left)throw Limit{};--budget.work_left;++budget.work_used;};
    const auto point=[&](std::uint32_t id){float p[3];g.centre(id,p);if(!std::isfinite(p[0])||!std::isfinite(p[1])||!std::isfinite(p[2]))throw Bad{};return Vec3{p[0],p[1],p[2]};};
    const auto dist=[](Vec3 a,Vec3 b){const double x=double(a.x)-b.x,y=double(a.y)-b.y,z=double(a.z)-b.z;const double v=std::sqrt(x*x+y*y+z*z);if(!std::isfinite(v)||v>3.4e38)throw Bad{};return float(v);};
    try{
        poll();
        if(!q.level_epoch||!q.snapshot_seq||!q.geometry_revision||!q.ownership_generation||!q.restrictions_revision||q.actor.id==invalid_id||!q.actor.generation||
            !q.expansion_limit||q.expansion_limit>65536||!q.output_limit||q.output_limit>65536||!g.valid(q.origin_node)||!g.valid(q.goal_node))return r;
        struct Node {float cost;std::uint32_t parent;bool closed=false;};
        struct Pending {float score;std::uint32_t id;};
        struct Later {bool operator()(const Pending&a,const Pending&b)const{return a.score==b.score?a.id>b.id:a.score>b.score;}};
        std::map<std::uint32_t,Node> seen;std::priority_queue<Pending,std::vector<Pending>,Later> open;
        use();const auto goal=point(q.goal_node),start=point(q.origin_node);
        seen.emplace(q.origin_node,Node{0,invalid_node});open.push({dist(start,goal),q.origin_node});
        bool found=false;
        while(!open.empty()){
            use();const auto id=open.top().id;open.pop();auto&at=seen.at(id);if(at.closed)continue;
            if(r.expansions>=q.expansion_limit)throw Limit{};++r.expansions;at.closed=true;
            if(id==q.goal_node){r.length=at.cost;found=true;break;}
            const auto here=point(id);
            for(int k=0;k<4;++k){
                use();const auto next=g.link(id,k);if(!g.valid(next))continue;
                const auto there=point(next);const float cost=at.cost+dist(here,there);if(!std::isfinite(cost))throw Bad{};
                auto it=seen.find(next);
                if(it==seen.end())it=seen.emplace(next,Node{cost,id}).first;
                else if(it->second.closed||cost>=it->second.cost)continue;
                else{it->second.cost=cost;it->second.parent=id;}
                const float score=cost+dist(there,goal);if(!std::isfinite(score))throw Bad{};
                open.push({score,next});
            }
        }
        if(!found){r.status=StaticRouteStatus::unreachable;}
        else{
            auto id=q.goal_node;
            for(;;){use();if(r.nodes.size()>=q.output_limit)throw Limit{};r.nodes.push_back(id);r.centres.push_back(point(id));if(id==q.origin_node)break;id=seen.at(id).parent;if(id==invalid_node)throw Bad{};}
            std::reverse(r.nodes.begin(),r.nodes.end());std::reverse(r.centres.begin(),r.centres.end());poll();r.status=StaticRouteStatus::complete;
        }
    }catch(const Cancel&){r.status=StaticRouteStatus::cancelled;}
    catch(const Limit&){r.status=StaticRouteStatus::exhausted;}
    catch(const Bad&){r.status=StaticRouteStatus::invalid;}
    catch(...){r.status=StaticRouteStatus::failed;}
    if(r.status!=StaticRouteStatus::complete){r.nodes.clear();r.centres.clear();r.length=0;}
    r.work=budget.work_used-before;return r;
}

}}}
