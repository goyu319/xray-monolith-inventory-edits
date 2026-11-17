// Pure value/graph adapter; PCH disabled, /EHsc. Engine no-EH sources only
// supply synchronous graph accessors. Exceptions never cross this boundary.
#include "NodeRouteAlgorithm.h"
namespace painter { namespace combat { namespace engine {
StaticRouteResult build_static_route_query(const StaticRouteGraph&g,const StaticRouteRequest&q,StaticRouteQuota&budget,const std::function<bool()>&cancelled){
    StaticRouteResult out;out.source=q;
    if(!g.context||!g.valid||!g.centre||!g.link)return out;
    struct View {
        const StaticRouteGraph&g;
        bool valid(std::uint32_t id)const{return g.valid(g.context,id);}
        void centre(std::uint32_t id,float(&p)[3])const{g.centre(g.context,id,p);}
        std::uint32_t link(std::uint32_t id,int k)const{return g.link(g.context,id,k);}
    };
    return build_static_route(View{g},q,budget,cancelled);
}
}}}
