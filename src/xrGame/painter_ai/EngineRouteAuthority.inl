// Included inside painter::combat::engine after the private registry helpers.
// Authority thread only. No pointer or engine object crosses the value queue.
namespace {
RecordPtr route_record(ActorKey key) {
    if(key.id>=u32(u16(-1)))return {};
    auto* object=Level().Objects.net_Find(static_cast<u16>(key.id));
    auto* stalker=smart_cast<CAI_Stalker*>(object);
    auto found=records.find(stalker);
    if(found==records.end()||found->second->key!=key||!registered(*found->second))return {};
    return found->second;
}
bool route_identity(void*,const core::RouteProofSource& requested,const core::RouteProofSource& current) {
    auto record=route_record(requested.actor);if(!record||!service_enabled)return false;
    auto& r=*record;const auto& a=r.snapshot;
    if(!r.owned||!r.object->g_Alive()||!(a.flags&flag_admitted)||a.yield_reasons||a.wounded
       ||r.key!=requested.actor||r.ownership!=requested.owner||r.restrictions!=requested.restrictions
       ||r.epoch!=requested.epoch||r.geometry!=requested.geometry
       ||level_epoch()!=requested.epoch||captured_sequence!=current.sequence
       ||!core::same_route_corridor(requested,current)||requested.sequence>current.sequence||requested.at>current.at
       ||a.snapshot_seq!=current.sequence||a.observed_at_ms!=current.at||a.node_id!=current.origin_node
       ||a.position.x!=current.origin.x||a.position.y!=current.origin.y||a.position.z!=current.origin.z
       ||r.object->ai_location().level_vertex_id()!=current.origin_node)return false;
    const auto position=value(r.object->Position());
    if(position.x!=current.origin.x||position.y!=current.origin.y||position.z!=current.origin.z)return false;
    const auto source=std::find_if(r.snapshots.begin(),r.snapshots.end(),[&](const SourceStamp& s){
        return s.sequence==requested.sequence&&s.at==requested.at&&s.ownership==requested.owner&&s.restrictions==requested.restrictions;
    });
    return source!=r.snapshots.end();
}
Evidence route_node(void*,const core::RouteProofSource& requested,const core::RouteProofSource&,u32 previous,u32 node,Vec3 centre) {
    auto record=route_record(requested.actor);if(!record)return Evidence::unknown;
    auto& graph=ai().level_graph();if(!graph.valid_vertex_id(node)||!finite_value(centre))return Evidence::no;
    const auto actual=value(graph.vertex_position(node));
    if(actual.x!=centre.x||actual.y!=centre.y||actual.z!=centre.z)return Evidence::no;
    if(previous!=invalid_node){
        if(!graph.valid_vertex_id(previous))return Evidence::no;
        bool linked=false;for(u32 i=0;i<4;++i)linked=linked||graph.vertex(previous)->link(i)==node;
        if(!linked)return Evidence::no;
    }
    return record->object->movement().restrictions().accessible(node)?Evidence::yes:Evidence::no;
}
Evidence route_attachment(void*,const core::RouteProofSource& requested,const core::RouteProofSource& current,bool start,Vec3 centre,float max_height) {
    auto record=route_record(requested.actor);if(!record)return Evidence::unknown;
    auto& graph=ai().level_graph();const auto node=start?requested.origin_node:requested.goal_node;
    const auto point=start?current.origin:current.goal;
    if(!graph.valid_vertex_id(node)||!finite_value(point)||!finite_value(centre)||!std::isfinite(max_height)||max_height<=0)return Evidence::no;
    const auto p=native(point),c=native(centre);
    if(!graph.valid_vertex_position(p)||!graph.valid_vertex_position(c)||!graph.inside(node,p)||!graph.inside(node,c))return Evidence::no;
    // inside() alone is packed XZ: never attach a point on another floor.
    if(std::abs(point.y-graph.vertex_plane_y(node,point.x,point.z))>max_height
       ||graph.check_position_in_direction(node,p,c)!=node)return Evidence::no;
    auto& restrictions=record->object->movement().restrictions();
    // These are genuine native same-cell/floor/restriction attachments, not a
    // claim of swept-body collision safety. Native path/collision remains final.
    return restrictions.accessible(node)&&restrictions.accessible(p)&&restrictions.accessible(c)?Evidence::yes:Evidence::no;
}
}
RouteAuthority route_authority(){RouteAuthority out;out.identity=route_identity;out.node=route_node;out.attachment=route_attachment;return out;}
