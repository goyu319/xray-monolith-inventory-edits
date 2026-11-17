#include "CombatKnowledge.h"
#include "../CombatIds.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>

namespace painter { namespace combat { namespace core {
namespace {
bool finite3(const Vec3& p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
bool valid(const ActorKey& k){return k.id<invalid_id&&k.generation!=0;}
bool recent(TimeMs at,TimeMs now,TimeMs age){return at<=now&&now-at<=age;}
TimeMs after(TimeMs at,TimeMs d){return d>(std::numeric_limits<TimeMs>::max)()-at?(std::numeric_limits<TimeMs>::max)():at+d;}
TimeMs scaled(TimeMs duration,float factor){const double v=double(duration)*factor;return v>=double((std::numeric_limits<TimeMs>::max)())?(std::numeric_limits<TimeMs>::max)():TimeMs(v);}
float flat(const Vec3& a,const Vec3& b){return std::hypot(a.x-b.x,a.z-b.z);}
Vec3 direction(const Vec3& a,const Vec3& b){const float d=flat(a,b);return d>1.e-6f?Vec3{(b.x-a.x)/d,0,(b.z-a.z)/d}:Vec3{};}
Vec3 unit(Vec3 d){return direction({},d);}
Vec3 along(const Vec3& p,const Vec3& d,float n){return {p.x+d.x*n,p.y,p.z+d.z*n};}
float cosine(const Vec3& a,const Vec3& b){return a.x*b.x+a.z*b.z;}
bool in_cone(Vec3 a,Vec3 b,float degrees){return cosine(unit(a),unit(b))>=std::cos(degrees*.01745329251994329577f);}
struct ActorLess {bool operator()(const ActorKey& a,const ActorKey& b)const{return std::tie(a.id,a.generation)<std::tie(b.id,b.generation);}};
struct FoeLess {bool operator()(const FoeId& a,const FoeId& b)const{return std::tie(a.ghost_serial,a.side,a.actor.id,a.actor.generation)<std::tie(b.ghost_serial,b.side,b.actor.id,b.actor.generation);}};
FoeId real(ActorKey a){FoeId f;f.actor=a;return f;}
std::uint64_t mix(std::uint64_t v){v+=0x9e3779b97f4a7c15ULL;v=(v^(v>>30))*0xbf58476d1ce4e5b9ULL;v=(v^(v>>27))*0x94d049bb133111ebULL;return v^(v>>31);}
std::uint64_t bits(float f){std::uint32_t v;std::memcpy(&v,&f,sizeof(v));return v;}
std::uint64_t stamp(std::uint64_t kind,TimeMs at,const Vec3& v,std::uint64_t extra=0){return mix(kind^mix(at)^mix(bits(v.x))^mix(bits(v.y)+37)^mix(bits(v.z)+73)^mix(extra));}
float random01(std::uint64_t& state){state=mix(state);return float(double(state>>11)*(1.0/9007199254740992.0));}
float probability(std::uint64_t v){return float(double(mix(v)>>11)*(1.0/9007199254740992.0));}
const SourceFact* newest(const FoeKnowledge& f,bool own){const SourceFact* result=nullptr;for(std::size_t i=0;i<(own?2u:4u);++i){const auto& s=f.sources[i];if(s.valid&&(!result||s.at_ms>result->at_ms))result=&s;}return result;}
bool has_engaged(const FoeKnowledge& f){return f.shot_at_me.set||f.shot_me.set;}
struct Stop{};
}
const SourceFact* FoeKnowledge::last_known()const{const auto* fact=newest(*this,false);return down||(wounded&&(!fact||fact->at_ms<=wounded_ms))?nullptr:fact;}
const SourceFact* FoeKnowledge::own_last_known()const{const auto* fact=newest(*this,true);return down||(wounded&&(!fact||fact->at_ms<=wounded_ms))?nullptr:fact;}

bool executed_knowledge_context(const WorldSnapshot& w,const ActorSnapshot& a,const Order& o,
    const MemberKnowledge* known,TimeMs maximum_age,MemberKnowledgeContext& out) {
    out={};
    if(!a.alive||a.wounded||a.yield_reasons||!(a.flags&flag_admitted)||a.snapshot_seq!=w.sequence||
        a.observed_at_ms>w.now_ms||!recent(a.observed_at_ms,w.now_ms,maximum_age)||
        a.key!=o.actor||!o.order_seq||a.last_applied.order_seq!=o.order_seq||
        o.level_epoch!=w.level_epoch||o.geometry_revision!=w.geometry_revision||
        o.ownership_generation!=a.ownership_generation||o.restrictions_revision!=a.restrictions_revision||
        o.issued_at_ms>a.last_applied.at_ms||a.last_applied.at_ms>a.observed_at_ms||
        !recent(a.last_applied.at_ms,w.now_ms,maximum_age)||o.expires_at_ms<=w.now_ms)return false;
    out.actor=a.key;
    out.searching=o.kind==OrderKind::search||o.role==roles::search||o.role==roles::search_red||o.role==roles::search_sector;
    out.assault_role=o.kind==OrderKind::entry||o.kind==OrderKind::press||o.kind==OrderKind::rush;
    if(out.searching)out.intent=Intent::other;
    else if(o.kind==OrderKind::withdraw||o.kind==OrderKind::recover||o.role==roles::retreat_cover)out.intent=Intent::retreat;
    else if(o.kind==OrderKind::rush)out.intent=Intent::rush;
    else if(o.kind==OrderKind::press||o.kind==OrderKind::entry||o.kind==OrderKind::bound||o.kind==OrderKind::advance)out.intent=Intent::push;
    else if(o.kind==OrderKind::cover){out.intent=Intent::seek_cover;
        out.running_to_cover=o.pace==Pace::run&&finite3(o.goal_position)&&finite3(a.position)&&
            flat(o.goal_position,a.position)>o.arrival_radius_m;}
    else out.intent=Intent::hold;
    if(known&&known->actor==a.key&&known->ownership_generation==a.ownership_generation){
        for(const auto& f:known->foes){
            const auto* fact=f.last_known();if(!fact||fact->at_ms>w.now_ms)continue;
            const bool named=valid(o.look.target)&&o.look.target==f.id.actor;
            const bool ghost_search=out.searching&&f.id==known->target&&o.look.kind==LookKind::point&&
                finite3(o.look.value)&&flat(o.look.value,fact->position)<.01f&&std::abs(o.look.value.y-fact->position.y)<.01f;
            if(named||ghost_search){out.goal_enemy=f.id;break;}
        }
    }
    return true;
}

struct CombatKnowledge::Impl {
    struct Report {std::uint64_t serial=0,tick=0;ActorKey sender;ReportKind kind=ReportKind::seen;FoeId foe;SourceFact fact;Vec3 sender_position;
        std::uint64_t sender_owner=0;
        std::vector<RoomCellFact> room_cells;};
    struct Born {TimeMs at=0;Vec3 origin,dir;};
    struct GhostOrigin {std::uint64_t side_lifetime=0;FoeId first;
        bool operator<(const GhostOrigin& b)const{return side_lifetime<b.side_lifetime||
            (side_lifetime==b.side_lifetime&&FoeLess{}(first,b.first));}};
    struct Side {SideKnowledge view;std::uint64_t rng=1,serial=0,ghost_serial=0;std::vector<ActorKey> members;
        std::uint64_t lifetime=0;std::uint32_t faction=0;
        TimeMs last_present_ms=0;bool radio=false;std::vector<Report> reports;std::map<FoeId,Born,FoeLess> born;
        std::map<FoeId,FoeId,FoeLess> resolved;std::map<ActorKey,TimeMs,ActorLess> downs;
        // Identity aliases contain no source positions or per-man knowledge.
        std::map<FoeId,GhostOrigin,FoeLess> ghost_origins;std::map<GhostOrigin,FoeId> ghost_ids;};
    struct Member {MemberKnowledge view;std::map<FoeId,FoeKnowledge,FoeLess> foes;
        std::uint32_t faction=0,side_faction=0;
        std::map<FoeId,FoeId,FoeLess> migration_aliases; // current intake only
        std::map<ActorKey,TimeMs,ActorLess> nonhostile;
        std::set<std::uint64_t> seen_events;std::deque<std::uint64_t> event_order;
        TimeMs last_present_ms=0;std::uint64_t present_tick=0;
        TimeMs meter_at=0,heard_report_at=0,known_at=0,dog_end_at=0;bool reported_heard=false,known_once=false,was_empty=false,dog_nosol=false;
        FoeId dog_end;SourceFact heard_shot;FoeId heard_shooter;
        std::map<std::pair<std::uint64_t,std::uint32_t>,TimeMs> room_broadcast_at;
        std::map<std::pair<std::uint64_t,std::uint32_t>,std::size_t> room_broadcast_cursor;
        std::map<std::pair<std::uint64_t,std::uint32_t>,TimeMs> room_threat_barrier;
        TimeMs room_retry_at=0;
        MemberKnowledgeContext context;float ammo_ratio=1;bool empty=false;};
    bool poisoned=false;
    KnowledgeConfig cfg;std::uint32_t seed=1;std::uint64_t epoch=0,sequence=0,tick=0,next_side_lifetime=0;TimeMs time=0;
    Bounded<FactionStyle,24> faction_styles;FactionStyle default_style;
    float voice_range=Config{}.voice_m,voice_probability=Config{}.voice_p;
    float room_floor_tolerance=Config{}.room.floor_tolerance_m;
    std::map<ActorKey,Member,ActorLess> members;std::map<std::uint32_t,Side> sides;
    KnowledgeFrame* out=nullptr;const WorldSnapshot* world=nullptr;const KnowledgeContext* context=nullptr;
    const EngineServices* services=nullptr;const std::function<bool()>* cancelled=nullptr;std::size_t work=0;
    explicit Impl(KnowledgeConfig c):cfg(c){}
    explicit Impl(const Config& engine):cfg(engine.knowledge),seed(engine.seed),faction_styles(engine.factions),default_style(engine.faction_default){voice_range=engine.voice_m;voice_probability=engine.voice_p;room_floor_tolerance=engine.room.floor_tolerance_m;}
    std::unique_ptr<Impl> copy_state(const std::function<bool()>& cancel)const{
        std::size_t copied=0;
        auto check=[&]{if((++copied&31)==0&&cancel&&cancel())throw Stop{};};
        if(cancel&&cancel())throw Stop{};
        auto next=std::make_unique<Impl>(cfg);next->seed=seed;next->epoch=epoch;next->sequence=sequence;next->tick=tick;next->time=time;next->next_side_lifetime=next_side_lifetime;
        next->faction_styles=faction_styles;next->default_style=default_style;next->voice_range=voice_range;next->voice_probability=voice_probability;next->room_floor_tolerance=room_floor_tolerance;
        for(const auto& item:members){check();const auto& old=item.second;Member m;
            m.faction=old.faction;m.side_faction=old.side_faction;
            for(const auto& alias:old.migration_aliases){check();m.migration_aliases.emplace(alias);}
            m.last_present_ms=old.last_present_ms;m.present_tick=old.present_tick;
            m.meter_at=old.meter_at;m.heard_report_at=old.heard_report_at;m.known_at=old.known_at;m.dog_end_at=old.dog_end_at;
            m.reported_heard=old.reported_heard;m.known_once=old.known_once;m.was_empty=old.was_empty;m.dog_nosol=old.dog_nosol;
            m.dog_end=old.dog_end;m.heard_shot=old.heard_shot;m.heard_shooter=old.heard_shooter;m.context=old.context;m.ammo_ratio=old.ammo_ratio;m.empty=old.empty;
            const auto& v=old.view;auto& n=m.view;
            n.actor=v.actor;n.side=v.side;n.ownership_generation=v.ownership_generation;n.alive=v.alive;n.able=v.able;n.reloading=v.reloading;n.in_contact=v.in_contact;
            n.position=v.position;n.target=v.target;n.best_visible=v.best_visible;n.dogfight=v.dogfight;n.target_reason=v.target_reason;
            n.threat_valid=v.threat_valid;n.threat_visible=v.threat_visible;n.escape_line=v.escape_line;n.escape_run=v.escape_run;n.threat=v.threat;
            n.fresh_fire=v.fresh_fire;n.hit_from=v.hit_from;n.meter=v.meter;n.meter_level=v.meter_level;n.known_count=v.known_count;n.unseen_count=v.unseen_count;
            n.last_seen_ms=v.last_seen_ms;n.last_hit_ms=v.last_hit_ms;n.last_heard_ms=v.last_heard_ms;n.has_seen=v.has_seen;n.has_hit=v.has_hit;n.has_heard=v.has_heard;
            // view.foes is regenerated from the authoritative map during update.
            for(const auto& f:old.foes){check();m.foes.emplace(f);}
            for(const auto& f:old.nonhostile){check();m.nonhostile.emplace(f);}
            for(const auto& c:v.contacts){check();n.contacts.push_back(c);}for(const auto& b:v.board){check();n.board.push_back(b);}
            for(const auto& cell:v.room_cells){check();n.room_cells.push_back(cell);}
            for(const auto& room:v.communicated_rooms){check();n.communicated_rooms.push_back(room);}
            m.room_broadcast_at=old.room_broadcast_at;m.room_broadcast_cursor=old.room_broadcast_cursor;
            m.room_threat_barrier=old.room_threat_barrier;m.room_retry_at=old.room_retry_at;
            for(const auto& e:old.seen_events){check();m.seen_events.insert(e);}for(const auto& e:old.event_order){check();m.event_order.push_back(e);}
            next->members.emplace(item.first,std::move(m));
        }
        for(const auto& item:sides){check();const auto& old=item.second;Side s;
            s.lifetime=old.lifetime;s.faction=old.faction;
            s.last_present_ms=old.last_present_ms;s.rng=old.rng;s.serial=old.serial;s.ghost_serial=old.ghost_serial;s.radio=old.radio;
            s.view.side=old.view.side;s.view.known_count=old.view.known_count;s.view.unseen_count=old.view.unseen_count;s.view.last_info_ms=old.view.last_info_ms;
            s.view.last_seen_ms=old.view.last_seen_ms;s.view.last_heard_ms=old.view.last_heard_ms;s.view.last_hit_ms=old.view.last_hit_ms;s.view.has_info=old.view.has_info;
            // confirmed_down is regenerated from downs; hit history is authoritative.
            for(const auto& v:old.view.hits){check();s.view.hits.push_back(v);}for(const auto& v:old.members){check();s.members.push_back(v);}
            for(const auto& v:old.reports){check();s.reports.push_back(v);}for(const auto& v:old.born){check();s.born.emplace(v);}
            for(const auto& v:old.resolved){check();s.resolved.emplace(v);}for(const auto& v:old.downs){check();s.downs.emplace(v);}
            for(const auto& v:old.ghost_origins){check();s.ghost_origins.emplace(v);}for(const auto& v:old.ghost_ids){check();s.ghost_ids.emplace(v);}
            next->sides.emplace(item.first,std::move(s));
        }
        if(cancel&&cancel())throw Stop{};return next;
    }
    void poll(){if((++work&31)==0&&cancelled&&*cancelled&&(*cancelled)())throw Stop{};}
    void start_side(Side& s,std::uint32_t id,std::uint32_t faction){
        if(next_side_lifetime==(std::numeric_limits<std::uint64_t>::max)())throw std::overflow_error("knowledge side lifetime exhausted");
        s=Side{};s.view.side=id;s.faction=faction;s.lifetime=++next_side_lifetime;s.rng=mix(seed^id);}
    FoeId allocate_ghost(Side& s,const GhostOrigin* inherited=nullptr){
        if(s.ghost_serial==(std::numeric_limits<std::uint64_t>::max)())throw std::overflow_error("knowledge ghost identity exhausted");
        FoeId id;id.side=s.view.side;id.ghost_serial=++s.ghost_serial;
        const GhostOrigin origin=inherited?*inherited:GhostOrigin{s.lifetime,id};
        s.ghost_origins.emplace(id,origin);s.ghost_ids.emplace(origin,id);return id;}
    static FoeId local_id(const Member& m,FoeId id){const auto alias=m.migration_aliases.find(id);return alias==m.migration_aliases.end()?id:alias->second;}
    void regroup(Member& m,Side& previous,Side& destination){
        // Facts stay with their actual recipient. Only ghost identity/lifecycle
        // metadata travels; neither side's pending bus or other men's facts do.
        auto& board=m.view.board;board.erase(std::remove_if(board.begin(),board.end(),[&](const BuddyFact& fact){poll();
            return !std::binary_search(destination.members.begin(),destination.members.end(),fact.member,ActorLess{});}),board.end());
        m.view.room_cells.clear();m.room_broadcast_at.clear();m.room_broadcast_cursor.clear();m.room_threat_barrier.clear();m.room_retry_at=0;
        std::set<FoeId,FoeLess> referenced;
        auto remember=[&](FoeId id){if(id.ghost())referenced.insert(id);};
        for(const auto& item:m.foes){poll();remember(item.first);}
        for(const auto& fact:board){poll();remember(fact.enemy);}
        for(const auto id:{m.view.target,m.view.best_visible,m.view.dogfight,m.view.fresh_fire.shooter,m.view.hit_from.shooter,m.heard_shooter,m.dog_end})remember(id);
        for(const auto old:referenced){poll();
            const auto identity=previous.ghost_origins.find(old);
            const GhostOrigin origin=identity==previous.ghost_origins.end()?GhostOrigin{previous.lifetime,old}:identity->second;
            const auto existing=destination.ghost_ids.find(origin);
            const FoeId mapped=existing==destination.ghost_ids.end()?allocate_ghost(destination,&origin):existing->second;
            m.migration_aliases.emplace(old,mapped);
            const auto born=previous.born.find(old);if(born!=previous.born.end())destination.born.emplace(mapped,born->second);
            const auto resolved=previous.resolved.find(old);if(resolved!=previous.resolved.end())destination.resolved.emplace(mapped,resolved->second);
        }
        for(const auto& alias:m.migration_aliases){poll();if(alias.first==alias.second)continue;
            auto node=m.foes.extract(alias.first);if(!node.empty()){node.key()=alias.second;node.mapped().id=alias.second;m.foes.insert(std::move(node));}}
        m.view.target=local_id(m,m.view.target);m.view.best_visible=local_id(m,m.view.best_visible);m.view.dogfight=local_id(m,m.view.dogfight);
        m.view.fresh_fire.shooter=local_id(m,m.view.fresh_fire.shooter);m.view.hit_from.shooter=local_id(m,m.view.hit_from.shooter);
        m.heard_shooter=local_id(m,m.heard_shooter);m.dog_end=local_id(m,m.dog_end);
        for(auto& fact:board){poll();fact.enemy=local_id(m,fact.enemy);}
        m.view.side=destination.view.side;
    }
    void event(KnowledgeEventKind k,const Member& m,FoeId foe={}){KnowledgeEvent e;e.kind=k;e.side=m.view.side;e.actor=m.view.actor;e.foe=foe;e.at_ms=time;out->events.push_back(e);}
    float jitter(Side& s){return cfg.jitter_min+(cfg.jitter_max-cfg.jitter_min)*random01(s.rng);}
    void mark(TimedStatus& status,Side& s,TimeMs at,TimeMs duration){status={true,at,after(at,scaled(duration,jitter(s)))};}
    bool once(Member& m,std::uint64_t key){if(!m.seen_events.insert(key).second)return false;m.event_order.push_back(key);
        while(m.event_order.size()>cfg.event_history_cap){m.seen_events.erase(m.event_order.front());m.event_order.pop_front();}return true;}
    FoeKnowledge& foe(Member& m,Side& s,FoeId id){auto p=m.foes.try_emplace(id);if(p.second){p.first->second.id=id;p.first->second.first_ms=time;p.first->second.duration_jitter=jitter(s);}return p.first->second;}
    void info(Side& s){s.view.has_info=true;s.view.last_info_ms=time;}
    // Curve from frozen Lua SP.miss_factor. Separate typed receipt entry points
    // prevent a generic danger or a guessed distance becoming a bullet near miss.
    float distance_factor(float distance)const{return distance<=cfg.miss_near_m?cfg.miss_near_factor:
        distance<=cfg.miss_full_m?1.f:distance>=cfg.miss_end_m?0.f:(cfg.miss_end_m-distance)/(cfg.miss_end_m-cfg.miss_full_m);}
    float near_miss_factor(const NearMiss& receipt)const{return distance_factor(receipt.miss_m);}
    float impact_factor(float measured_head_to_impact)const{return distance_factor(measured_head_to_impact);}
    void add_meter(Member& m,float value){m.view.meter=std::clamp(m.view.meter+value,0.f,cfg.meter_max);}
    void source(FoeKnowledge& f,KnowledgeSource which,const SourceFact& fact){auto& old=f.sources[std::size_t(which)];if(!old.valid||fact.at_ms>=old.at_ms)old=fact;
        if(f.wounded&&fact.valid&&fact.at_ms>f.wounded_ms)f.wounded=false;}
    static int report_priority(ReportKind kind){
        switch(kind){case ReportKind::down:case ReportKind::man_down:return 3;
        case ReportKind::under_fire:case ReportKind::shot_from:case ReportKind::wounded:return 2;
        case ReportKind::room_cells:return -1;
        case ReportKind::seen:return 0;default:return 1;}}
    static bool same_cell(const RoomCellFact& a,const RoomCellFact& b){
        return a.side==b.side&&a.building==b.building&&a.room==b.room&&a.mission_revision==b.mission_revision&&
            a.level_epoch==b.level_epoch&&a.map_hash==b.map_hash&&a.geometry_revision==b.geometry_revision&&
            a.attempt==b.attempt&&a.sample==b.sample&&a.observer==b.observer&&a.observer_owner==b.observer_owner;}
    static bool has_cell(const Member& m,const RoomCellFact& fact){
        return std::any_of(m.view.room_cells.begin(),m.view.room_cells.end(),[&](const RoomCellFact& known){
            return same_cell(known,fact)&&known.observed_at_ms>=fact.observed_at_ms;});}
    bool remember_cell(Member& m,const RoomCellFact& fact){
        auto& cells=m.view.room_cells;auto found=std::find_if(cells.begin(),cells.end(),[&](const RoomCellFact& old){return same_cell(old,fact);});
        if(found!=cells.end()){if(found->observed_at_ms>=fact.observed_at_ms)return false;*found=fact;return true;}
        // Saturation fails closed: an unretained cell cannot certify a room.
        constexpr std::size_t cap=4096;if(cells.size()>=cap)return false;
        cells.push_back(fact);return true;}
    bool retained_cell(const RoomCellFact& fact,const Side& side)const{
        if(!fact.observed_at_ms||fact.observed_at_ms>time||!valid(fact.observer)||!fact.observer_owner||
            fact.side!=side.view.side||fact.level_epoch!=world->level_epoch||fact.map_hash!=world->map_hash||
            fact.geometry_revision!=world->geometry_revision||fact.building==invalid_room||fact.room==invalid_room)return false;
        const auto actual=std::find_if(world->sides.begin(),world->sides.end(),[&](const SideSnapshot& row){return row.side_id==fact.side;});
        return actual!=world->sides.end()&&actual->mission.kind==MissionKind::clear_building&&
            actual->mission.building==fact.building&&actual->mission.revision==fact.mission_revision&&
            std::none_of(world->actors.begin(),world->actors.end(),[&](const ActorSnapshot& source){return
                source.key.id==fact.observer.id&&(source.key!=fact.observer||
                source.ownership_generation!=fact.observer_owner||
                (source.alive&&std::find(actual->members.begin(),actual->members.end(),fact.observer)==actual->members.end()));});}
    bool current_cell(const RoomCellFact& fact,const Side& side)const{
        if(!retained_cell(fact,side))return false;
        const auto source=std::find_if(world->actors.begin(),world->actors.end(),[&](const ActorSnapshot& actor){return actor.key==fact.observer;});
        return source!=world->actors.end()&&source->ownership_generation==fact.observer_owner&&
            std::find(side.members.begin(),side.members.end(),fact.observer)!=side.members.end();}
    bool newer_room_threat(const Member& holder,const RoomCellFact& cell){
        if(!services->buildings)return false;
        for(const auto& proof:context->room_proofs){poll();
            if(proof.attempt!=cell.attempt||proof.physical.side!=cell.side||
                proof.physical.building!=cell.building||proof.physical.room!=cell.room||
                proof.mission!=cell.mission_revision||proof.epoch!=cell.level_epoch||
                proof.map!=cell.map_hash||proof.geometry!=cell.geometry_revision)continue;
            const auto barrier=holder.room_threat_barrier.find({cell.attempt,cell.room});
            if(barrier!=holder.room_threat_barrier.end()&&proof.physical.at_ms<=barrier->second)return true;
            for(const auto& building:services->buildings->buildings)if(building.id==cell.building)
                for(const auto& room:building.rooms)if(room.id==cell.room)
                    for(const auto& item:holder.foes){poll();const auto& foe=item.second;if(foe.down)continue;
                        const auto* fact=foe.last_known();if(!fact||!fact->valid||fact->at_ms<=proof.physical.at_ms)continue;
                        const auto& p=fact->position;
                        if(p.x>=room.min.x&&p.x<=room.max.x&&p.z>=room.min.z&&p.z<=room.max.z&&
                            std::abs(p.y-room.floor_y)<=cfg.eye_m+room_floor_tolerance)return true;}
        }
        return false;}
    void assemble_rooms(Member& m,Side& side){
        m.view.communicated_rooms.clear();
        if(!m.view.alive||!m.view.ownership_generation||!services->buildings||
            services->buildings->level_epoch!=world->level_epoch||services->buildings->map_hash!=world->map_hash)return;
        for(const auto& proof:context->room_proofs){poll();
            if(proof.physical.side!=side.view.side||proof.epoch!=world->level_epoch||proof.map!=world->map_hash||
                proof.geometry!=world->geometry_revision||proof.cells.empty()||proof.cells.size()>4096||
                !proof.physical.at_ms||proof.physical.at_ms>time)continue;
            const auto current=std::find_if(world->sides.begin(),world->sides.end(),[&](const SideSnapshot& row){return row.side_id==side.view.side;});
            if(current==world->sides.end()||current->mission.kind!=MissionKind::clear_building||
                current->mission.building!=proof.physical.building||current->mission.revision!=proof.mission)continue;
            const Room* room=nullptr;
            for(const auto& building:services->buildings->buildings)if(building.id==proof.physical.building)
                for(const auto& row:building.rooms)if(row.id==proof.physical.room){room=&row;break;}
            if(!room)continue;
            bool complete=true;
            for(const auto& cell:proof.cells){poll();if(cell.side!=side.view.side||cell.building!=proof.physical.building||
                    cell.room!=proof.physical.room||cell.mission_revision!=proof.mission||cell.level_epoch!=proof.epoch||
                    cell.map_hash!=proof.map||cell.geometry_revision!=proof.geometry||cell.attempt!=proof.attempt||
                    cell.observed_at_ms>proof.physical.at_ms||!has_cell(m,cell)){complete=false;break;}}
            TimeMs newest_hostile=0;
            for(const auto& item:m.foes){poll();const auto& foe=item.second;if(foe.down)continue;
                const auto* fact=foe.last_known();if(!fact||!fact->valid||fact->at_ms<=proof.physical.at_ms)continue;
                const auto& p=fact->position;
                if(p.x>=room->min.x&&p.x<=room->max.x&&p.z>=room->min.z&&p.z<=room->max.z&&
                    std::abs(p.y-room->floor_y)<=cfg.eye_m+room_floor_tolerance)
                    newest_hostile=(std::max)(newest_hostile,fact->at_ms);}
            const auto key=std::make_pair(proof.attempt,proof.physical.room);
            if(newest_hostile){auto& barrier=m.room_threat_barrier[key];barrier=(std::max)(barrier,newest_hostile);}
            const auto barrier=m.room_threat_barrier.find(key);
            if(complete&&(barrier==m.room_threat_barrier.end()||proof.physical.at_ms>barrier->second))
                m.view.communicated_rooms.push_back(proof.physical);
        }
    }
    void queue_cells(Member& m,Side& side,const std::vector<RoomCellFact>& cells){
        if(cells.empty())return;
        if(side.reports.size()>=cfg.outbox_cap){
            auto lowest=std::min_element(side.reports.begin(),side.reports.end(),[](const Report& a,const Report& b){return report_priority(a.kind)<report_priority(b.kind);});
            ++out->report_overflow;if(lowest==side.reports.end()||report_priority(lowest->kind)>=report_priority(ReportKind::room_cells))return;
            side.reports.erase(lowest);}
        Report report;report.serial=++side.serial;report.tick=tick;report.sender=m.view.actor;report.kind=ReportKind::room_cells;
        report.sender_position=m.view.position;report.sender_owner=m.view.ownership_generation;
        report.room_cells=cells;side.reports.push_back(std::move(report));}
    void queue(Member& m,Side& s,ReportKind kind,FoeId id,const SourceFact& f={}){poll();
        // Keep the original report and its observation time until delivered.
        // Updating its fact in place would bypass the communication latency.
        for(const auto& pending:s.reports)if(pending.sender==m.view.actor&&pending.kind==kind&&pending.foe==id&&
            pending.fact.valid==f.valid&&pending.fact.at_ms==f.at_ms&&pending.fact.position.x==f.position.x&&
            pending.fact.position.y==f.position.y&&pending.fact.position.z==f.position.z)return;
        if(s.reports.size()>=cfg.outbox_cap){
            auto lowest=std::min_element(s.reports.begin(),s.reports.end(),[](const Report& a,const Report& b){return report_priority(a.kind)<report_priority(b.kind);});
            ++out->report_overflow;
            if(lowest==s.reports.end()||report_priority(lowest->kind)>=report_priority(kind))return;
            s.reports.erase(lowest);
        }
        s.reports.push_back({++s.serial,tick,m.view.actor,kind,id,f,m.view.position,m.view.ownership_generation});}
    void contact(Member& m,Vec3 p,SoundKind kind,TimeMs at,bool report=false,ActorKey reporter={}){
        m.view.contacts.push_back({p,at,kind,report,!m.known_once||!recent(m.known_at,time,cfg.peace_ms),report?reporter:m.view.actor});
        std::stable_sort(m.view.contacts.begin(),m.view.contacts.end(),[](const LooseContact& a,const LooseContact& b){return a.at_ms<b.at_ms;});
        if(m.view.contacts.size()>cfg.contacts_cap)m.view.contacts.erase(m.view.contacts.begin());}
    Vec3 ground(Vec3 p){if(services->project){Vec3 snap;if(services->project(p,cfg.project_horizontal_m,cfg.project_vertical_m,&snap)!=invalid_node&&finite3(snap))p=snap;}return p;}
    Vec3 disperse(Member& m,Side& s,Vec3 p,SoundKind kind){const float r=(std::min)(cfg.disp_cap_m,flat(m.view.position,p)/(kind==SoundKind::gunshot?cfg.disp_shot:cfg.disp_other)*jitter(s));
        const float a=random01(s.rng)*6.283185307179586f,rr=r*std::sqrt(random01(s.rng));p.x+=std::cos(a)*rr;p.z+=std::sin(a)*rr;return ground(p);}
    FoeKnowledge* attribute(Member& m,Vec3 p,SoundKind kind,TimeMs at){FoeKnowledge* best=nullptr;float distance=(std::numeric_limits<float>::max)();
        for(auto& item:m.foes){poll();auto& f=item.second;if(!f.visible||f.down||f.wounded)continue;const auto* k=f.last_known();if(!k)continue;const float d=flat(k->position,p);if(d<=cfg.heard_near_m&&d<distance){best=&f;distance=d;}}
        if(!best)for(auto& item:m.foes){poll();auto& f=item.second;const auto* k=f.last_known();if(!k||f.id.ghost())continue;
            const float radius=(std::max)(cfg.heard_near_m,(std::min)(cfg.disp_cap_m,flat(m.view.position,k->position)/cfg.disp_shot*cfg.jitter_max));
            const float d=flat(k->position,p);if(d<=radius&&d<distance){best=&f;distance=d;}}
        if(best)source(*best,KnowledgeSource::heard,{true,p,at,m.view.actor,kind});else contact(m,p,kind,at);return best;}
    float ray(const Vec3& p,const Vec3& q){if(!services->rays)return std::numeric_limits<float>::quiet_NaN();Segment segment{p,q};float hit=std::numeric_limits<float>::quiet_NaN();services->rays(&segment,&hit,1);return hit;}
    const Building* building(const ActorSnapshot& a)const{const auto* cache=services->buildings;
        if(!cache||cache->level_epoch!=epoch||cache->map_hash!=world->map_hash)return nullptr;
        for(const auto& b:cache->buildings)if(b.id==a.building_id)return &b;return nullptr;}
    bool was_seen(std::uint32_t side,std::uint32_t node)const{for(const auto& n:context->seen_nodes)if(n.side==side&&n.node==node&&n.at_ms<=time)return true;return false;}
    bool red_point(Member& m,const ActorSnapshot& a,Vec3 dir,Vec3& result){const auto* b=building(a);if(!b||!services->node_position||!services->rays)return false;
        bool found=false;float best=(std::numeric_limits<float>::max)();std::set<std::uint32_t> checked;
        for(const auto& portal:b->portals)for(const auto* list:{&portal.lit_a,&portal.lit_b})for(const auto& lit:*list){poll();
            if(!lit.standing||!checked.insert(lit.node).second||was_seen(m.view.side,lit.node))continue;Vec3 p;
            if(!services->node_position(lit.node,&p)||!finite3(p)||std::abs(p.y-m.view.position.y)>=cfg.red_floor_m)continue;
            const float d=flat(m.view.position,p);const Vec3 u=direction(m.view.position,p);
            if(d<cfg.red_source_min_m||d>cfg.red_source_max_m||!in_cone(dir,u,cfg.red_source_cone_deg))continue;
            const float score=(1-cosine(dir,u))*cfg.red_angle_cost+d*cfg.red_distance_cost;if(score>=best)continue;
            Vec3 eye=m.view.position;eye.y+=cfg.eye_m;Vec3 end=p;end.y+=cfg.eye_m;
            const float hit=ray(eye,end);if(!std::isfinite(hit)||hit>=0)continue;best=score;result=p;found=true;
        }return found;}
    Vec3 ghost_point(Member& m,const ActorSnapshot& a,Vec3 dir){float range=(std::numeric_limits<float>::max)();
        for(const auto& item:m.foes){poll();const auto& f=item.second;const auto* k=f.last_known();if(!k||f.id.ghost())continue;
            const auto d=flat(m.view.position,k->position);if(d>0&&in_cone(dir,direction(m.view.position,k->position),cfg.hit_cone_deg))range=(std::min)(range,d);}
        const bool known=std::isfinite(range)&&range<(std::numeric_limits<float>::max)();
        if(known)range=(std::max)(cfg.hit_min_m,range);else range=cfg.hit_default_m;
        Vec3 point;if(!known&&red_point(m,a,dir,point))return ground(point);
        Vec3 eye=m.view.position;eye.y+=cfg.eye_m;
        if(building(a)){const auto hit=ray(eye,along(eye,dir,range));if(std::isfinite(hit)&&hit>=0)range=(std::min)(range,(std::max)(0.f,hit-cfg.hit_back_m));}
        else if(!known)for(float probe:cfg.ghost_probe_m){const auto hit=ray(eye,along(eye,dir,probe));if(!std::isfinite(hit))break;if(hit>=0){range=(std::max)(cfg.probe_min_m,probe*cfg.probe_fraction);break;}}
        return ground(along(m.view.position,dir,range));}
    FoeKnowledge& ghost(Member& m,Side& s,const ActorSnapshot& a,Vec3 dir,bool hit,TimeMs at){FoeKnowledge* selected=nullptr;float best=-2;
        for(auto& item:m.foes){poll();auto& f=item.second;const auto* k=f.last_known();if(!f.id.ghost()||!k||!recent(k->at_ms,time,cfg.ghost_keep_ms))continue;
            const auto u=direction(m.view.position,k->position);const float c=cosine(dir,u);if(in_cone(dir,u,cfg.ghost_cone_deg)&&c>best){selected=&f;best=c;}}
        const Vec3 p=ghost_point(m,a,dir);
        if(!selected){const auto id=allocate_ghost(s);selected=&foe(m,s,id);s.born[id]={at,m.view.position,dir};event(KnowledgeEventKind::ghost_born,m,id);}
        source(*selected,KnowledgeSource::heard,{true,p,at,m.view.actor,SoundKind::gunshot});mark(selected->shot_at_me,s,at,cfg.shot_at_me_ms);if(hit)mark(selected->shot_me,s,at,cfg.shot_me_ms);
        queue(m,s,ReportKind::shot_from,selected->id,{true,p,at,m.view.actor,SoundKind::gunshot});
        auto& fresh=m.view.fresh_fire;if(!fresh.valid||fresh.shooter!=selected->id||!recent(fresh.at_ms,at,cfg.uf_escape_ms))fresh={true,selected->id,at,at,p,dir,m.view.position,0,hit};
        fresh.at_ms=at;fresh.point=p;fresh.direction=dir;fresh.origin=m.view.position;fresh.hit=hit;++fresh.count;return *selected;}
    void resolve(Side& s,FoeId id,FoeId actual,Member& observer){if(!s.resolved.emplace(id,actual).second)return;
        event(KnowledgeEventKind::ghost_resolved,observer,id);out->events.back().resolved_to=actual;
        for(const auto& key:s.members){auto i=members.find(key);if(i==members.end())continue;auto& m=i->second;
            m.foes.erase(id);if(m.view.target==id)m.view.target={};if(m.view.fresh_fire.shooter==id)m.view.fresh_fire={};
            if(m.view.hit_from.shooter==id){m.view.hit_from.shooter=actual;m.view.hit_from.unseen=false;}}
    }
    void resolve_visible(Member& m,Side& s){std::vector<std::pair<FoeId,FoeId>> resolutions;
        for(const auto& pair:m.foes){poll();const auto& ghost=pair.second;const auto* point=ghost.last_known();if(!ghost.id.ghost()||!point)continue;
            const auto dir=direction(m.view.position,point->position);const float range=flat(m.view.position,point->position);
            for(const auto& visible:m.foes){const auto& v=visible.second;const auto* p=v.last_known();if(!v.visible||!p||v.id.ghost())continue;
                if(in_cone(dir,direction(m.view.position,p->position),cfg.ghost_merge_deg)||flat(point->position,p->position)<=(std::max)(cfg.ghost_merge_min_m,range*cfg.ghost_merge_range_k)){resolutions.push_back({ghost.id,v.id});break;}}
        }for(const auto& r:resolutions)resolve(s,r.first,r.second,m);}
    void down(Member& m,Side& s,ActorKey key){auto& f=foe(m,s,real(key));const auto* before=newest(f,false);SourceFact p=before?*before:SourceFact{};
        const bool changed=!f.down;f.down=true;f.visible=false;f.wounded=false;f.down_ms=time;f.status_reporter=m.view.actor;
        if(changed){p.at_ms=time;p.reporter=m.view.actor;queue(m,s,ReportKind::down,f.id,p);}
        if(s.downs.emplace(key,time).second){event(KnowledgeEventKind::enemy_down,m,f.id);}else s.downs[key]=time;
        if(p.valid){std::vector<FoeId> resolve_ids;for(const auto& born:s.born){poll();if(!recent(born.second.at,time,cfg.ghost_dying_ms))continue;
            const float d=flat(born.second.origin,p.position);if(d>0&&in_cone(born.second.dir,direction(born.second.origin,p.position),d<=cfg.hit_cone_close_m?cfg.hit_cone_close_deg:cfg.hit_cone_deg))resolve_ids.push_back(born.first);}
            for(const auto& id:resolve_ids)resolve(s,id,f.id,m);}}
    FoeKnowledge* visible_shooter(Member& m,Vec3 dir){FoeKnowledge* best=nullptr;float angle=-2;
        for(auto& item:m.foes){poll();auto& f=item.second;const auto* k=f.last_known();if(!f.visible||!k)continue;const auto d=flat(m.view.position,k->position);const auto u=direction(m.view.position,k->position);
            if(in_cone(dir,u,d<=cfg.hit_cone_close_m?cfg.hit_cone_close_deg:cfg.hit_cone_deg)&&cosine(dir,u)>angle){best=&f;angle=cosine(dir,u);}}return best;}
    void hit(Member& m,Side& s,const ActorSnapshot& a,Vec3 dir,TimeMs at,bool direct){auto* from=visible_shooter(m,dir);
        if(from){mark(from->shot_at_me,s,at,cfg.shot_at_me_ms);if(direct)mark(from->shot_me,s,at,cfg.shot_me_ms);}
        else from=&ghost(m,s,a,dir,direct,at);
        if(!direct)return;
        add_meter(m,cfg.meter_hit);m.view.has_hit=true;m.view.last_hit_ms=at;s.view.last_hit_ms=at;info(s);
        const auto* k=from->last_known();if(!k)return;m.view.hit_from={true,from->id.ghost(),from->id,k->position,m.view.position,dir,at};
        m.view.hit_from.observer=a.key;s.view.hits.push_back(m.view.hit_from);if(s.view.hits.size()>cfg.hit_log_cap)s.view.hits.erase(s.view.hits.begin());
        if(from->id.ghost()){contact(m,k->position,SoundKind::gunshot,at);queue(m,s,ReportKind::under_fire,from->id,*k);}}
    void hear(Member& m,Side& s,const Heard& h,bool physical_sound_only=true){if(physical_sound_only&&h.kind==SoundKind::gunshot&&flat(m.view.position,h.position)>cfg.gunfire_max_m)return;
        const Vec3 p=disperse(m,s,h.position,h.kind);auto* f=attribute(m,p,h.kind,h.at_ms);m.view.has_heard=true;m.view.last_heard_ms=h.at_ms;
        if(h.kind==SoundKind::weapon&&f){mark(f->reloading,s,h.at_ms,cfg.reloading_ms);queue(m,s,ReportKind::foe_vulnerable,f->id,{true,p,h.at_ms,m.view.actor,h.kind});}
        if(h.kind==SoundKind::gunshot){m.heard_shot={true,p,h.at_ms,m.view.actor,h.kind};m.heard_shooter=f?f->id:FoeId{};
            if(flat(m.view.position,p)<=cfg.clear_heard_m){s.view.last_heard_ms=h.at_ms;info(s);}
            if(physical_sound_only&&f&&f->visible&&f->facing_me){add_meter(m,cfg.proxy_shot);mark(f->shot_at_me,s,h.at_ms,cfg.shot_at_me_ms);}}
        if(!m.reported_heard||!recent(m.heard_report_at,time,cfg.heard_report_ms-std::min<TimeMs>(1,cfg.heard_report_ms))){m.reported_heard=true;m.heard_report_at=time;
            queue(m,s,ReportKind::heard,f?f->id:FoeId{},{true,p,h.at_ms,m.view.actor,h.kind});}}
    void receive(Member& m,Side& s,const Report& r){poll();
        if(r.foe.ghost()&&s.resolved.count(r.foe))return;
        if(r.kind==ReportKind::seen||r.kind==ReportKind::heard||r.kind==ReportKind::shot_from){
            if(!r.fact.valid)return;if(!r.foe.valid()){contact(m,r.fact.position,r.fact.sound,r.fact.at_ms,true,r.sender);return;}
            auto& f=foe(m,s,r.foe);if(f.visible||(f.down&&r.fact.at_ms<=f.down_ms))return;
            auto fact=r.fact;fact.reporter=r.sender;source(f,r.kind==ReportKind::seen?KnowledgeSource::squad_seen:KnowledgeSource::squad_heard,fact);info(s);
        }else if(r.kind==ReportKind::down&&r.foe.valid()){
            auto& f=foe(m,s,r.foe);const auto* k=newest(f,false);if(k&&k->at_ms>r.fact.at_ms)return;
            f.down=true;f.visible=false;f.down_ms=r.fact.at_ms;f.status_reporter=r.sender;
        }else if(r.kind==ReportKind::wounded&&r.foe.valid()){
            auto& f=foe(m,s,r.foe);const auto* k=newest(f,false);if(k&&k->at_ms>r.fact.at_ms)return;
            f.wounded=true;f.visible=false;f.wounded_ms=r.fact.at_ms;f.wounded_position=r.fact.position;f.status_reporter=r.sender;
        }else if(r.kind==ReportKind::foe_vulnerable&&r.foe.valid()){
            auto f=m.foes.find(r.foe);if(f!=m.foes.end()&&!f->second.down){auto& status=f->second.reloading;const auto previous=status;
                mark(status,s,r.fact.at_ms,cfg.reloading_ms);if(previous.set){status.at_ms=(std::max)(status.at_ms,previous.at_ms);status.until_ms=(std::max)(status.until_ms,previous.until_ms);}}
        }else if(r.kind==ReportKind::suppressed&&r.foe.valid()&&!r.foe.ghost()){
            const auto sender=members.find(r.sender);auto f=m.foes.find(r.foe);
            if(!r.fact.valid||r.fact.at_ms>time||time-r.fact.at_ms>=cfg.suppressed_ms||
                sender==members.end()||sender->second.present_tick!=tick||!sender->second.view.alive||!sender->second.view.able||
                sender->second.view.side!=s.view.side||sender->second.view.ownership_generation!=r.sender_owner||
                f==m.foes.end()||f->second.down||f->second.wounded)return;
            auto& foe=f->second;if(foe.suppressed.set&&foe.suppressed.at_ms>r.fact.at_ms)return;
            foe.suppressed={true,r.fact.at_ms,after(r.fact.at_ms,cfg.suppressed_ms)};
            foe.suppression_reporter=r.sender;foe.suppression_owner=r.sender_owner;
        }else if(r.kind==ReportKind::room_cells){
            const auto sender=members.find(r.sender);
            if(sender==members.end()||sender->second.present_tick!=tick||
                sender->second.view.side!=s.view.side||sender->second.view.ownership_generation!=r.sender_owner||
                std::find(s.members.begin(),s.members.end(),r.sender)==s.members.end())return;
            for(const auto& cell:r.room_cells){poll();if(!retained_cell(cell,s)||
                (!sender->second.view.alive&&cell.observer!=r.sender)||
                !has_cell(sender->second,cell)||
                newer_room_threat(sender->second,cell)||
                (cell.observer==r.sender&&!current_cell(cell,s)))continue;
                remember_cell(m,cell);}
        }else if(r.kind==ReportKind::reloading||r.kind==ReportKind::man_down||r.kind==ReportKind::under_fire){
            const ActorKey subject=r.kind==ReportKind::man_down&&valid(r.foe.actor)?r.foe.actor:r.sender;
            auto& board=m.view.board;auto i=std::find_if(board.begin(),board.end(),[&](const BuddyFact& b){return b.member==subject&&b.kind==r.kind;});
            BuddyFact fact{subject,r.kind,r.kind==ReportKind::man_down?FoeId{}:r.foe,r.fact.position,r.fact.at_ms};if(i==board.end())board.push_back(fact);else if(fact.at_ms>=i->at_ms)*i=fact;}
    }
    void deliver(Side& s){std::vector<Report> pending;for(const auto& r:s.reports){poll();if(tick-r.tick<cfg.latency_ticks){pending.push_back(r);continue;}
        if(r.kind==ReportKind::room_cells){const auto sender=std::find_if(world->actors.begin(),world->actors.end(),[&](const ActorSnapshot& a){return a.key==r.sender;});
            if(sender==world->actors.end()||sender->ownership_generation!=r.sender_owner||
                std::find(s.members.begin(),s.members.end(),r.sender)==s.members.end())continue;}
        for(const auto& key:s.members){if(key.id==r.sender.id)continue;auto member=members.find(key);if(member==members.end()||member->second.present_tick!=tick||!member->second.view.alive)continue;
            auto& m=member->second;const bool voice=flat(r.sender_position,m.view.position)<=voice_range;
            const auto h=seed^mix(s.view.side)^mix(r.sender.id)^mix(r.sender.generation)^mix(key.id+65537ULL)^mix(key.generation+13)^mix(r.serial);
            const bool ok=s.radio||(voice&&probability(h)<voice_probability);if(ok)receive(m,s,r);
            KnowledgeEvent e;e.kind=KnowledgeEventKind::report;e.side=s.view.side;e.actor=r.sender;e.receiver=key;e.foe=r.foe;e.at_ms=time;e.report=r.kind;e.delivered=ok;out->events.push_back(e);
        }}s.reports=std::move(pending);}
    void intake(Member& m,Side& s,const ActorSnapshot& a){const bool was_alive=m.view.alive,was_able=m.view.able,was_reload=m.view.reloading;
        m.view.alive=a.alive;m.view.able=a.alive&&!a.wounded;m.view.position=a.position;m.view.reloading=a.reloading||a.weapon.state==WeaponState::reloading;
        m.empty=a.weapon.ammo_in_mag==0;m.ammo_ratio=a.weapon.mag_size?float(a.weapon.ammo_in_mag)/float(a.weapon.mag_size):(m.empty?0.f:1.f);
        if((!m.view.alive&&was_alive)||(!m.view.able&&was_able))queue(m,s,ReportKind::man_down,{}, {true,a.position,time,a.key,SoundKind::injury});
        if((m.view.reloading||m.empty)&&!(was_reload||m.was_empty))queue(m,s,ReportKind::reloading,{}, {true,a.position,time,a.key,SoundKind::weapon});m.was_empty=m.empty;
        const TimeMs dt=time>=m.meter_at?time-m.meter_at:0;add_meter(m,-cfg.meter_decay_per_second*float(double(dt)/1000));m.meter_at=time;
        std::set<FoeId,FoeLess> previous_visible;for(auto& f:m.foes){if(f.second.visible)previous_visible.insert(f.first);f.second.visible=false;f.second.facing_me=false;}
        m.view.best_visible={};if(!a.alive)return;
        auto sight=[&](const Contact& c,bool selected){poll();if(!valid(c.key)||c.key==a.key||!finite3(c.position)||c.observed_at_ms>a.observed_at_ms)return;
            if(!c.hostile){m.nonhostile[c.key]=time;return;}
            m.nonhostile.erase(c.key);if(!c.visible||!c.alive_when_observed)return;
            auto& f=foe(m,s,real(c.key));if(c.wounded){if(!f.wounded){event(KnowledgeEventKind::enemy_wounded,m,f.id);queue(m,s,ReportKind::wounded,f.id,{true,c.position,time,a.key,SoundKind::other});}
                f.wounded=true;f.wounded_ms=time;f.wounded_position=c.position;f.status_reporter=a.key;return;}
            f.visible=true;f.facing_me=c.facing_me;f.down=false;f.wounded=false;source(f,KnowledgeSource::seen,{true,c.position,time,a.key,SoundKind::other});
            if(selected)m.view.best_visible=f.id;queue(m,s,ReportKind::seen,f.id,f.sources[0]);s.downs.erase(c.key);s.view.last_seen_ms=time;info(s);
            m.view.has_seen=true;m.view.last_seen_ms=time;
        };
        for(std::size_t i=0;i<a.visible_enemies.count;++i)sight(a.visible_enemies.items[i],false);sight(a.enemy,true);
        for(const auto& id:previous_visible){auto i=m.foes.find(id);if(i!=m.foes.end()&&!i->second.visible&&!i->second.down&&i->second.sources[0].valid)queue(m,s,ReportKind::seen,id,i->second.sources[0]);}
        auto friendly=[&](ActorKey key){
            auto contact=m.nonhostile.find(key);if(contact!=m.nonhostile.end()&&recent(contact->second,time,cfg.forget_ms))return true;
            if(std::find(s.members.begin(),s.members.end(),key)!=s.members.end())return true;
            auto known=members.find(key);if(known!=members.end()&&known->second.view.side==m.view.side)return true;
            return false;
        };
        for(std::size_t i=0;i<a.seen_dead.count;++i){const auto key=a.seen_dead.items[i];if(valid(key)&&key!=a.key&&!friendly(key))down(m,s,key);}
        resolve_visible(m,s);
        for(std::size_t i=0;i<a.heard.count;++i){const auto& h=a.heard.items[i];if(!finite3(h.position)||!std::isfinite(h.power)||h.power<0||!recent(h.at_ms,time,cfg.input_age_ms)||h.at_ms>a.observed_at_ms){++out->invalid_facts;continue;}
            if(once(m,stamp(1+std::uint64_t(h.kind),h.at_ms,h.position)))hear(m,s,h);}
        std::map<std::uint64_t,std::uint32_t> occurrences;
        for(std::size_t i=0;i<a.hits.count;++i){const auto& h=a.hits.items[i];const Vec3 dir=unit(h.dir);if(!finite3(h.dir)||cosine(dir,dir)<.5f||!recent(h.at_ms,time,cfg.input_age_ms)||h.at_ms>a.observed_at_ms){++out->invalid_facts;continue;}
            const auto key=stamp(100,h.at_ms,h.dir);if(once(m,mix(key^mix(++occurrences[key]))))hit(m,s,a,dir,h.at_ms,true);}
        // Attribution is separate from meter distance: an impact's position is
        // never a shooter position. Only a personally heard bearing can supply it.
        auto attribute_round=[&](TimeMs at,Vec3 dir){
            info(s);
            if(cosine(dir,dir)<.5f&&m.heard_shot.valid&&recent(m.heard_shot.at_ms,at,cfg.uf_dir_ms)){
                dir=direction(m.view.position,m.heard_shot.position);
                auto known=m.foes.find(m.heard_shooter);
                if(known!=m.foes.end()&&!known->second.visible&&known->second.last_known()&&!visible_shooter(m,dir)){
                    auto& f=known->second;mark(f.shot_at_me,s,at,cfg.shot_at_me_ms);
                    const auto* fact=f.last_known();auto& fresh=m.view.fresh_fire;
                    if(!fresh.valid||fresh.shooter!=f.id||!recent(fresh.at_ms,at,cfg.uf_escape_ms))fresh={true,f.id,at,at,fact->position,dir,m.view.position,0,false};
                    fresh.at_ms=at;fresh.point=fact->position;fresh.direction=dir;fresh.origin=m.view.position;++fresh.count;
                    queue(m,s,ReportKind::shot_from,f.id,{true,fact->position,at,m.view.actor,SoundKind::gunshot});return;
                }
            }
            if(cosine(dir,dir)>.5f)hit(m,s,a,dir,at,false);
        };
        for(std::size_t i=0;i<a.near_misses.count;++i){const auto& n=a.near_misses.items[i];poll();
            if(!recent(n.at_ms,time,cfg.input_age_ms)||n.at_ms>a.observed_at_ms||!std::isfinite(n.miss_m)||n.miss_m<0||!finite3(n.origin)||!finite3(n.dir)){++out->invalid_facts;continue;}
            const auto key=stamp(104,n.at_ms,n.origin,stamp(105,n.at_ms,n.dir,bits(n.miss_m)));if(!once(m,mix(key^mix(++occurrences[key]))))continue;
            // A measured crack is locally audible even if its source was beyond
            // ordinary hearing range. Its origin is dispersed, never exposed raw.
            Heard sound{SoundKind::gunshot,n.origin,1.f,n.at_ms,{}};hear(m,s,sound,false);
            add_meter(m,cfg.caliber*near_miss_factor(n));attribute_round(n.at_ms,unit(n.dir));
        }
        if(recent(a.danger.at_ms,time,cfg.input_age_ms)&&a.danger.at_ms<=a.observed_at_ms&&once(m,stamp(102,a.danger.at_ms,a.danger.position))){
            if(a.danger.kind==DangerKind::hit&&finite3(a.danger.position)){
                // Contract v2.14: the native ricochet position is a real impact,
                // not the shooter's position or a measured bullet near miss.
                const float dx=a.danger.position.x-a.position.x,dy=a.danger.position.y-(a.position.y+cfg.eye_m),dz=a.danger.position.z-a.position.z;
                const float head_to_impact=std::hypot(std::hypot(dx,dz),dy);
                if(std::isfinite(head_to_impact)){add_meter(m,cfg.caliber*impact_factor(head_to_impact));attribute_round(a.danger.at_ms,{});}
            }else if(a.danger.kind==DangerKind::attack)++out->missing_impact_distance;
        }
        for(const auto& f:context->suppressed_foes)if(f.observer==a.key&&!f.foe.ghost()&&
            f.ownership_generation==a.ownership_generation&&f.level_epoch==world->level_epoch&&
            f.at_ms<=a.observed_at_ms&&recent(f.at_ms,time,cfg.input_age_ms)&&recent(f.at_ms,time,cfg.suppressed_ms)){
            auto i=m.foes.find(local_id(m,f.foe));if(i!=m.foes.end()&&!i->second.down&&!i->second.wounded&&
                once(m,stamp(103,f.at_ms,{},f.foe.actor.id))){
                const auto* known=i->second.last_known();if(!known||!known->valid)continue;
                i->second.suppressed={true,f.at_ms,after(f.at_ms,cfg.suppressed_ms)};
                i->second.suppression_reporter=a.key;i->second.suppression_owner=a.ownership_generation;
                queue(m,s,ReportKind::suppressed,i->second.id,{true,known->position,f.at_ms,a.key});}}
    }
    bool searched(const Member& m,const SourceFact& f){if(!services->project||!services->buildings||!services->buildings->locate)return false;
        const auto* cache=services->buildings;if(cache->level_epoch!=epoch||cache->map_hash!=world->map_hash)return false;
        const auto node=services->project(f.position,cfg.project_horizontal_m,cfg.project_vertical_m,nullptr);if(node==invalid_node)return false;
        const auto loc=cache->locate(node);for(const auto& r:m.view.communicated_rooms)if(r.side==m.view.side&&r.building==loc.first&&r.room==loc.second&&r.at_ms>f.at_ms&&r.at_ms<=time)return true;return false;}
    void expire(Member& m,Side& s){
        for(auto i=m.nonhostile.begin();i!=m.nonhostile.end();)if(!recent(i->second,time,cfg.forget_ms))i=m.nonhostile.erase(i);else ++i;
        for(auto i=m.foes.begin();i!=m.foes.end();){poll();auto& f=i->second;f.search_open=m.context.searching&&m.context.goal_enemy==f.id;
            const auto* fact=newest(f,false);const bool resolved=f.id.ghost()&&s.resolved.count(f.id);
            const bool ghost_expired=f.id.ghost()&&(!fact||!recent(fact->at_ms,time,cfg.ghost_keep_ms)||searched(m,*fact));
            const bool forgotten=f.down?!recent(f.down_ms,time,cfg.forget_ms):
                f.wounded?!recent(f.wounded_ms,time,cfg.forget_ms):(!fact||!recent(fact->at_ms,time,f.search_open?cfg.search_forget_ms:cfg.forget_ms));
            if(resolved||ghost_expired||forgotten){const auto id=f.id;if(ghost_expired)event(KnowledgeEventKind::ghost_expired,m,id);
                if(m.view.target==id)m.view.target={};if(m.view.fresh_fire.shooter==id)m.view.fresh_fire={};if(m.view.hit_from.shooter==id)m.view.hit_from.shooter={};i=m.foes.erase(i);}else ++i;}
        auto& contacts=m.view.contacts;contacts.erase(std::remove_if(contacts.begin(),contacts.end(),[&](const LooseContact& c){return !recent(c.at_ms,time,cfg.contact_keep_ms);}),contacts.end());
        auto& board=m.view.board;board.erase(std::remove_if(board.begin(),board.end(),[&](const BuddyFact& b){return !recent(b.at_ms,time,cfg.forget_ms);}),board.end());
    }
    void dog_stop(Member& m,bool no_solution=false){if(!m.view.dogfight.valid())return;m.dog_end=m.view.dogfight;m.dog_end_at=time;m.dog_nosol=no_solution;event(KnowledgeEventKind::dog_end,m,m.view.dogfight);m.view.dogfight={};}
    void dog_update(Member& m){const auto& c=m.context;
        if(c.dog_no_solution&&c.dog_no_solution_at_ms<=time&&c.dog_no_solution_target==m.view.dogfight&&c.dog_no_solution_at_ms>m.dog_end_at){dog_stop(m,true);m.dog_end_at=c.dog_no_solution_at_ms;}
        if(m.view.reloading||m.empty||!m.view.able||c.intent==Intent::rush||c.intent==Intent::push||c.assault_role){dog_stop(m);return;}
        auto existing=m.foes.find(m.view.dogfight);if(m.view.dogfight.valid()){
            const auto* k=existing!=m.foes.end()?existing->second.last_known():nullptr;
            if(!k||flat(m.view.position,k->position)>cfg.dog_end_m||(!existing->second.visible&&!recent(existing->second.sources[0].valid?existing->second.sources[0].at_ms:k->at_ms,time,cfg.dog_unseen_ms)))dog_stop(m);}
        FoeKnowledge* best=nullptr;float distance=(std::numeric_limits<float>::max)();const float limit=m.view.dogfight.valid()?cfg.dog_end_m:cfg.dog_start_m;
        for(auto& item:m.foes){poll();auto& f=item.second;const auto* k=f.last_known();if(!k||f.id.ghost())continue;const float d=flat(m.view.position,k->position);
            const bool new_sight=f.sources[0].valid&&recent(f.sources[0].at_ms,time,cfg.dog_seen_ms)&&(f.id!=m.dog_end||f.sources[0].at_ms>m.dog_end_at);
            const bool new_hit=f.shot_me.set&&recent(f.shot_me.at_ms,time,scaled(cfg.dog_shotme_ms,f.duration_jitter))&&(f.id!=m.dog_end||f.shot_me.at_ms>m.dog_end_at);
            bool hot=new_sight||new_hit||f.id==m.view.dogfight;
            if(m.dog_nosol&&f.id==m.dog_end&&recent(m.dog_end_at,time,cfg.dog_nosol_cool_ms)&&!(f.shot_me.set&&f.shot_me.at_ms>m.dog_end_at))hot=false;
            if(hot&&d<=limit&&d<distance){best=&f;distance=d;}}
        if(!m.view.dogfight.valid()&&(c.running_to_cover||c.intent==Intent::retreat)&&m.ammo_ratio<cfg.dog_ammo)return;
        if(best&&m.view.dogfight!=best->id){dog_stop(m);m.view.dogfight=best->id;event(KnowledgeEventKind::dog_start,m,best->id);}
    }
    void choose(Member& m){dog_update(m);
        if(!m.view.alive){m.view.target={};m.view.best_visible={};m.view.target_reason=TargetReason::none;
            m.view.threat_valid=m.view.threat_visible=m.view.escape_line=m.view.escape_run=m.view.in_contact=false;
            m.view.known_count=m.view.unseen_count=0;m.view.foes.clear();return;}
FoeId pick;TargetReason reason=TargetReason::none;auto current=m.foes.find(m.view.target);
        FoeKnowledge* cur=current!=m.foes.end()&&current->second.last_known()?&current->second:nullptr;
        if(m.view.dogfight.valid()){pick=m.view.dogfight;reason=TargetReason::dogfight;}
        if(!pick.valid()){FoeKnowledge* best=nullptr;float d=(std::numeric_limits<float>::max)();for(auto& pair:m.foes){auto& f=pair.second;const auto* k=f.last_known();if(!f.visible||!k)continue;float distance=flat(m.view.position,k->position);if(distance<d){best=&f;d=distance;}}
            if(best){pick=best->id;reason=TargetReason::visible;if(cur&&cur->visible&&cur!=best&&d>=cfg.keep_target_k*flat(m.view.position,cur->last_known()->position)){pick=cur->id;reason=TargetReason::keep;}}}
        if(!pick.valid()){FoeKnowledge* best=nullptr;for(auto& pair:m.foes){auto& f=pair.second;if(!f.last_known()||!f.shot_me.set||!recent(f.shot_me.at_ms,time,cfg.shot_me_pick_ms))continue;
                if(!best||(best->id.ghost()&&!f.id.ghost())||(best->id.ghost()==f.id.ghost()&&f.shot_me.at_ms>best->shot_me.at_ms))best=&f;}
            if(best){pick=best->id;reason=best->id.ghost()?TargetReason::unseen_shooter:TargetReason::shot_me;}}
        const auto intent=m.context.intent;const bool no_keep=intent==Intent::none||intent==Intent::hold||intent==Intent::seek_cover||intent==Intent::retreat||intent==Intent::shift_cover;
        if(!pick.valid()&&cur&&(cur->search_open||!no_keep)){pick=cur->id;reason=TargetReason::keep;}
        if(!pick.valid()){FoeKnowledge* best=nullptr;for(auto& pair:m.foes){auto& f=pair.second;if(!f.last_known()||!f.sources[0].valid)continue;
                if(!best||(has_engaged(f)&&!has_engaged(*best))||(has_engaged(f)==has_engaged(*best)&&f.sources[0].at_ms>best->sources[0].at_ms))best=&f;}
            if(best){pick=best->id;reason=TargetReason::recent;}}
        if(!pick.valid()){FoeKnowledge *closest=nullptr,*engaged=nullptr;float d=(std::numeric_limits<float>::max)(),ed=d;
            for(auto& pair:m.foes){auto& f=pair.second;const auto* k=f.last_known();if(!k)continue;const float distance=flat(m.view.position,k->position);
                if(distance<d){closest=&f;d=distance;}if(has_engaged(f)&&distance<ed){engaged=&f;ed=distance;}}
            if(closest){if(d<=cfg.engaged_near_m||!engaged){pick=closest->id;reason=d<=cfg.engaged_near_m?TargetReason::nearest_known:TargetReason::closest_known;}
                else{pick=engaged->id;reason=TargetReason::engaged;}}}
        if(pick!=m.view.target){event(KnowledgeEventKind::target,m,pick);out->events.back().target_reason=reason;}m.view.target=pick;m.view.target_reason=reason;
        m.view.threat_valid=false;m.view.threat_visible=false;auto target=m.foes.find(pick);if(target!=m.foes.end()){const auto* k=target->second.last_known();if(k){m.view.threat_valid=true;m.view.threat=k->position;m.view.threat_visible=target->second.visible;}}
        if(!m.view.threat_valid&&!m.view.contacts.empty()){const auto& c=m.view.contacts.back();m.view.threat_valid=true;m.view.threat=c.position;}
        m.view.in_contact=false;for(const auto& f:m.foes)if(f.second.visible){m.view.in_contact=true;break;}
        if((m.view.has_seen&&recent(m.view.last_seen_ms,time,cfg.squad_seen_ms-std::min<TimeMs>(1,cfg.squad_seen_ms)))||
            (m.view.has_hit&&recent(m.view.last_hit_ms,time,cfg.squad_seen_ms-std::min<TimeMs>(1,cfg.squad_seen_ms))))m.view.in_contact=true;
        if(target!=m.foes.end()&&target->second.shot_me.set&&recent(target->second.shot_me.at_ms,time,cfg.squad_seen_ms-std::min<TimeMs>(1,cfg.squad_seen_ms)))m.view.in_contact=true;
        const auto& uf=m.view.fresh_fire;auto shooter=m.foes.find(uf.shooter);const bool unseen=shooter==m.foes.end()||!shooter->second.visible;
        m.view.escape_line=uf.valid&&unseen&&recent(uf.at_ms,time,cfg.uf_escape_ms);m.view.escape_run=m.view.escape_line&&uf.hit&&recent(uf.at_ms,time,cfg.uf_run_ms);
        m.view.known_count=m.view.unseen_count=0;m.view.foes.clear();for(const auto& item:m.foes){const auto& f=item.second;const auto* k=f.last_known();if(k&&recent(k->at_ms,time,cfg.assess_foe_ms)){if(f.id.ghost())++m.view.unseen_count;else ++m.view.known_count;}if(!f.down)m.view.foes.push_back(f);}
        if(m.view.known_count||m.view.threat_visible){m.known_once=true;m.known_at=time;}
        std::uint8_t level=0;for(float threshold:cfg.meter_thresholds)if(m.view.meter>=threshold)++level;
        if(level!=m.view.meter_level){m.view.meter_level=level;event(KnowledgeEventKind::meter,m);out->events.back().value=m.view.meter;}
    }
    bool valid_config()const{
        for(float v:{cfg.jitter_min,cfg.jitter_max,cfg.hit_cone_deg,cfg.hit_cone_close_deg,cfg.hit_cone_close_m,cfg.ghost_cone_deg,cfg.ghost_merge_deg,
            cfg.ghost_merge_min_m,cfg.ghost_merge_range_k,cfg.hit_default_m,cfg.hit_min_m,cfg.hit_back_m,cfg.probe_fraction,cfg.probe_min_m,
            cfg.ghost_probe_m[0],cfg.ghost_probe_m[1],cfg.red_source_cone_deg,cfg.red_source_min_m,cfg.red_source_max_m,cfg.red_floor_m,
            cfg.red_angle_cost,cfg.red_distance_cost,cfg.eye_m,cfg.disp_shot,cfg.disp_other,cfg.disp_cap_m,cfg.heard_near_m,
            cfg.clear_heard_m,cfg.gunfire_max_m,voice_range,voice_probability,cfg.meter_max,cfg.meter_decay_per_second,cfg.meter_hit,cfg.caliber,cfg.proxy_shot,
            cfg.meter_thresholds[0],cfg.meter_thresholds[1],cfg.meter_thresholds[2],cfg.meter_thresholds[3],
            cfg.miss_near_m,cfg.miss_full_m,cfg.miss_end_m,cfg.miss_near_factor,cfg.keep_target_k,cfg.engaged_near_m,cfg.dog_start_m,cfg.dog_end_m,
            cfg.dog_ammo,cfg.project_horizontal_m,cfg.project_vertical_m})if(!std::isfinite(v)||v<0)return false;
        return cfg.disp_shot>0&&cfg.disp_other>0&&cfg.keep_target_k<=1&&cfg.dog_ammo<=1&&cfg.dog_end_m>=cfg.dog_start_m&&
        cfg.red_source_max_m>=cfg.red_source_min_m&&cfg.hit_cone_deg<=180&&cfg.hit_cone_close_deg<=180&&cfg.ghost_cone_deg<=180&&cfg.ghost_merge_deg<=180&&
        cfg.miss_end_m>cfg.miss_full_m&&cfg.miss_full_m>=cfg.miss_near_m&&cfg.miss_near_m>=0&&cfg.contacts_cap&&cfg.event_history_cap&&cfg.hit_log_cap&&cfg.outbox_cap&&
        cfg.latency_ticks>=1&&voice_probability<=1&&cfg.jitter_min>0&&cfg.jitter_max>=cfg.jitter_min&&std::is_sorted(cfg.meter_thresholds.begin(),cfg.meter_thresholds.end());}
};

CombatKnowledge::CombatKnowledge(Config c):impl_(new Impl(c)){}
CombatKnowledge::CombatKnowledge(KnowledgeConfig c):CombatKnowledge([&]{Config engine;engine.knowledge=c;return engine;}()){}
CombatKnowledge::~CombatKnowledge()=default;
CombatKnowledge::CombatKnowledge(CombatKnowledge&&)noexcept=default;
CombatKnowledge& CombatKnowledge::operator=(CombatKnowledge&&)noexcept=default;
std::unique_ptr<CombatKnowledge> CombatKnowledge::clone(const std::function<bool()>& cancel)const{
    if(impl_->poisoned)return nullptr;
    try{auto staged=impl_->copy_state(cancel);auto result=std::make_unique<CombatKnowledge>(impl_->cfg);result->impl_=std::move(staged);return result;}
    catch(const Stop&){return nullptr;}
}
void CombatKnowledge::reset(){auto fresh=std::make_unique<Impl>(impl_->cfg);fresh->seed=impl_->seed;fresh->faction_styles=impl_->faction_styles;fresh->default_style=impl_->default_style;fresh->voice_range=impl_->voice_range;fresh->voice_probability=impl_->voice_probability;impl_=std::move(fresh);}
KnowledgeReject CombatKnowledge::validate_input(const WorldSnapshot& world,const std::function<bool()>& cancel)const {
    try {
        if(cancel&&cancel())return KnowledgeReject::cancelled;
        if(impl_->poisoned)return KnowledgeReject::failed;
        if(!impl_->valid_config()||impl_->faction_styles.count>impl_->faction_styles.items.size())return KnowledgeReject::invalid_config;
        if(!world.level_epoch||!world.sequence)return KnowledgeReject::malformed_input;
        if(world.level_epoch<impl_->epoch||(world.level_epoch==impl_->epoch&&
            (world.sequence<=impl_->sequence||world.now_ms<impl_->time)))return KnowledgeReject::stale_input;
        std::size_t work=0;auto poll=[&]{if((++work&31)==0&&cancel&&cancel())throw Stop{};};
        std::set<std::uint32_t> ids,side_ids;std::set<ActorKey,ActorLess> assigned;
        for(const auto& a:world.actors) {
            poll();
            if(!valid(a.key)||!ids.insert(a.key.id).second||a.level_epoch!=world.level_epoch||
                a.snapshot_seq!=world.sequence||a.observed_at_ms>world.now_ms||!finite3(a.position)||
                a.visible_enemies.count>a.visible_enemies.items.size()||a.heard.count>a.heard.items.size()||
                a.hits.count>a.hits.items.size()||a.near_misses.count>a.near_misses.items.size()||
                a.seen_dead.count>a.seen_dead.items.size())return KnowledgeReject::malformed_input;
        }
        for(const auto& side:world.sides) {
            poll();if(!side_ids.insert(side.side_id).second)return KnowledgeReject::malformed_input;
            for(const auto& key:side.members){poll();if(!valid(key)||!assigned.insert(key).second)return KnowledgeReject::malformed_input;}
            for(const auto& shared:side.known_enemies){poll();if(shared.reporters.count>shared.reporters.items.size())return KnowledgeReject::malformed_input;}
        }
        if(cancel&&cancel())return KnowledgeReject::cancelled;
        return KnowledgeReject::none;
    }catch(const Stop&){return KnowledgeReject::cancelled;}
    catch(...){return KnowledgeReject::failed;}
}
bool CombatKnowledge::reset_side(std::uint32_t side,const std::function<bool()>& cancel) {
    if(impl_->poisoned)return false;
    if(cancel&&cancel())return false;
    impl_->poisoned=true;std::size_t work=0;
    for(auto it=impl_->members.begin();it!=impl_->members.end();) {
        if((++work&31)==0&&cancel&&cancel())return false;
        if(it->second.view.side==side)it=impl_->members.erase(it);else ++it;
    }
    impl_->sides.erase(side);
    if(cancel&&cancel())return false;
    impl_->poisoned=false;return true;
}
KnowledgeFrame CombatKnowledge::update(const WorldSnapshot& world,const KnowledgeContext& context,
    const EngineServices& services,const std::function<bool()>& cancel) {
    return update_observed(world,{},context,services,cancel);
}
KnowledgeFrame CombatKnowledge::update_observed(const WorldSnapshot& world,const std::vector<ObservedCasualty>& observations,
    const KnowledgeContext& context,const EngineServices& services,const std::function<bool()>& cancel) {
    KnowledgeFrame output;output.level_epoch=world.level_epoch;output.sequence=world.sequence;
    output.rejection=validate_input(world,cancel);
    if(output.rejection!=KnowledgeReject::none){output.cancelled=output.rejection==KnowledgeReject::cancelled;return output;}
    try {
        auto staged=clone(cancel);
        if(!staged){output.cancelled=true;output.rejection=KnowledgeReject::cancelled;return output;}
        output=staged->update_disposable_observed(world,observations,context,services,cancel);
        if(output.accepted)impl_=std::move(staged->impl_);
        return output;
    }catch(...){output.rejection=KnowledgeReject::failed;return output;}
}
KnowledgeFrame CombatKnowledge::update_disposable(const WorldSnapshot& world,const KnowledgeContext& context,const EngineServices& services,const std::function<bool()>& cancel){
    return update_disposable_observed(world,{},context,services,cancel);
}
KnowledgeFrame CombatKnowledge::update_disposable_observed(const WorldSnapshot& world,const std::vector<ObservedCasualty>& observations,
    const KnowledgeContext& context,const EngineServices& services,const std::function<bool()>& cancel){
    KnowledgeFrame output;output.level_epoch=world.level_epoch;output.sequence=world.sequence;
    try{
        output.rejection=validate_input(world,cancel);
        if(output.rejection!=KnowledgeReject::none){output.cancelled=output.rejection==KnowledgeReject::cancelled;return output;}
        if(world.level_epoch!=impl_->epoch)reset();
        auto& s=*impl_;s.poisoned=true;
        s.out=&output;s.world=&world;s.context=&context;s.services=&services;s.cancelled=&cancel;s.work=0;s.epoch=world.level_epoch;s.sequence=world.sequence;s.time=world.now_ms;++s.tick;
        std::map<ActorKey,const ActorSnapshot*,ActorLess> actors;std::set<std::uint32_t> ids;
        for(const auto& a:world.actors){s.poll();if(!valid(a.key)||!ids.insert(a.key.id).second||a.level_epoch!=world.level_epoch||a.snapshot_seq!=world.sequence||a.observed_at_ms>world.now_ms||!finite3(a.position)||
            a.visible_enemies.count>a.visible_enemies.items.size()||a.heard.count>a.heard.items.size()||a.hits.count>a.hits.items.size()||a.near_misses.count>a.near_misses.items.size()||a.seen_dead.count>a.seen_dead.items.size())throw std::runtime_error("knowledge prevalidation mismatch");actors.emplace(a.key,&a);}
        std::map<ActorKey,std::uint32_t,ActorLess> membership;std::set<std::uint32_t> current_sides;
        for(const auto& side:world.sides){if(!current_sides.insert(side.side_id).second)throw std::runtime_error("knowledge prevalidation mismatch");auto p=s.sides.try_emplace(side.side_id);auto& state=p.first->second;
            if(p.second||state.faction!=side.faction)s.start_side(state,side.side_id,side.faction);
            state.last_present_ms=world.now_ms;state.members=side.members;std::sort(state.members.begin(),state.members.end(),ActorLess{});
            state.radio=s.default_style.comms==1;for(std::size_t i=0;i<s.faction_styles.count;++i)if(s.faction_styles.items[i].faction==side.faction){state.radio=s.faction_styles.items[i].comms==1;break;}
            for(const auto& policy:context.radios)if(policy.side==side.side_id)state.radio=policy.radio;
            for(const auto& key:side.members){if(!valid(key)||!membership.emplace(key,side.side_id).second)throw std::runtime_error("knowledge prevalidation mismatch");}
            for(const auto& shared:side.known_enemies){if(shared.reporters.count>shared.reporters.items.size())throw std::runtime_error("knowledge prevalidation mismatch");
                // A merged row alone does not say which source/time each reporter
                // owns. Current personal facts generate the actual delivered reports.
                bool verified=false;for(std::size_t i=0;i<shared.reporters.count;++i){auto observer=actors.find(shared.reporters.items[i]);if(observer==actors.end())continue;
                    auto matches=[&](const Contact& c){return c.key==shared.key&&c.hostile&&c.visible&&c.position.x==shared.position.x&&c.position.y==shared.position.y&&c.position.z==shared.position.z;};
                    if(matches(observer->second->enemy))verified=true;for(std::size_t j=0;j<observer->second->visible_enemies.count;++j)if(matches(observer->second->visible_enemies.items[j]))verified=true;}
                if(!verified)++output.unverified_shared_rows;}
        }
        // Retain personal memory across temporary control/roster gaps, never current
        // visibility or gameplay publication. ActorKey, not the lease, owns it.
        for(auto it=s.sides.begin();it!=s.sides.end();){
            if(!current_sides.count(it->first)){
                it->second.members.clear();
                if(!recent(it->second.last_present_ms,world.now_ms,s.cfg.forget_ms)){it=s.sides.erase(it);continue;}
            }++it;
        }
        for(auto it=s.members.begin();it!=s.members.end();){
            const bool replaced=ids.count(it->first.id)&&!actors.count(it->first);
            const bool dormant=it->second.present_tick+1<s.tick||!actors.count(it->first)||!membership.count(it->first);
            if(replaced||!s.sides.count(it->second.view.side)||(dormant&&!recent(it->second.last_present_ms,world.now_ms,s.cfg.forget_ms))){it=s.members.erase(it);continue;}
            if(!actors.count(it->first)||!membership.count(it->first)){
                for(auto& foe:it->second.foes){foe.second.visible=false;foe.second.facing_me=false;}
                it->second.view.best_visible={};
            }++it;
        }
        std::set<std::pair<std::uint64_t,std::uint32_t>> active_room_attempts;
        for(const auto& cell:context.room_cells)active_room_attempts.emplace(cell.attempt,cell.room);
        for(const auto& proof:context.room_proofs)active_room_attempts.emplace(proof.attempt,proof.physical.room);
        // All members exist and have current own positions before communication.
        for(const auto& entry:actors){s.poll();auto side=membership.find(entry.first);if(side==membership.end()){++output.unassigned_actors;continue;}const auto& a=*entry.second;
            auto& destination=s.sides.at(side->second);auto p=s.members.try_emplace(a.key);auto& m=p.first->second;m.migration_aliases.clear();
            const bool replaced_origin=!p.second&&m.view.side!=side->second&&s.sides.at(m.view.side).faction!=m.side_faction;
            if(p.second||m.faction!=a.faction||m.side_faction!=destination.faction||replaced_origin){m=Impl::Member{};m.view.actor=a.key;m.view.side=side->second;
                m.faction=a.faction;m.side_faction=destination.faction;m.view.ownership_generation=a.ownership_generation;m.meter_at=world.now_ms;}
            else if(m.view.side!=side->second)s.regroup(m,s.sides.at(m.view.side),destination);
            if(m.view.ownership_generation!=a.ownership_generation){m.view.room_cells.clear();m.room_broadcast_at.clear();m.room_broadcast_cursor.clear();m.room_threat_barrier.clear();m.room_retry_at=0;}
            m.view.ownership_generation=a.ownership_generation;m.last_present_ms=world.now_ms;m.present_tick=s.tick;
            auto& cells=m.view.room_cells;cells.erase(std::remove_if(cells.begin(),cells.end(),[&](const RoomCellFact& fact){return
                !active_room_attempts.count({fact.attempt,fact.room})||
                fact.side!=side->second||fact.level_epoch!=world.level_epoch||fact.map_hash!=world.map_hash||
                fact.geometry_revision!=world.geometry_revision||destination.view.side!=fact.side||
                // Already delivered memory survives an absent/dead reporter.
                // A live reporter changing owner/side or the same ID being
                // reborn is a new authority, so the old observation retires.
                std::any_of(world.actors.begin(),world.actors.end(),[&](const ActorSnapshot& source){return
                    source.key.id==fact.observer.id&&(source.key!=fact.observer||
                    source.ownership_generation!=fact.observer_owner||
                    (source.alive&&std::find(destination.members.begin(),destination.members.end(),fact.observer)==destination.members.end()));})||
                std::none_of(world.sides.begin(),world.sides.end(),[&](const SideSnapshot& current){return current.side_id==fact.side&&
                    current.mission.kind==MissionKind::clear_building&&current.mission.building==fact.building&&
                    current.mission.revision==fact.mission_revision;});}),cells.end());
            for(auto it=m.room_broadcast_at.begin();it!=m.room_broadcast_at.end();){
                if(!active_room_attempts.count(it->first))it=m.room_broadcast_at.erase(it);else ++it;}
            for(auto it=m.room_broadcast_cursor.begin();it!=m.room_broadcast_cursor.end();){
                if(!active_room_attempts.count(it->first))it=m.room_broadcast_cursor.erase(it);else ++it;}
            for(auto it=m.room_threat_barrier.begin();it!=m.room_threat_barrier.end();){
                if(!active_room_attempts.count(it->first))it=m.room_threat_barrier.erase(it);else ++it;}
            m.view.position=a.position;m.context={};m.context.actor=a.key;for(const auto& c:context.members)if(c.actor==a.key){m.context=c;break;}
            m.context.goal_enemy=Impl::local_id(m,m.context.goal_enemy);m.context.dog_no_solution_target=Impl::local_id(m,m.context.dog_no_solution_target);}
        for(const auto& entry:actors){auto i=s.members.find(entry.first);if(i!=s.members.end()&&i->second.present_tick==s.tick)s.intake(i->second,s.sides.at(i->second.view.side),*entry.second);}
        // Only the actual observer may originate a room-cell report. Replayed
        // worker results are stamped by their capture; a changed owner, side,
        // mission, map or geometry cannot acquire the previous man's fact.
        std::map<ActorKey,std::vector<RoomCellFact>,ActorLess> room_input;
        for(const auto& cell:context.room_cells){s.poll();auto a=actors.find(cell.observer);auto mi=membership.find(cell.observer);
            if(a==actors.end()||mi==membership.end()||mi->second!=cell.side||!a->second->alive||
                a->second->ownership_generation!=cell.observer_owner||cell.observed_at_ms>a->second->observed_at_ms)continue;
            auto side=s.sides.find(cell.side);if(side==s.sides.end()||!s.current_cell(cell,side->second))continue;
            room_input[cell.observer].push_back(cell);}
        for(const auto& entry:actors){s.poll();auto member=s.members.find(entry.first);
            if(member==s.members.end()||!entry.second->alive||member->second.present_tick!=s.tick||
                (member->second.room_retry_at&&world.now_ms<member->second.room_retry_at))continue;
            member->second.room_retry_at=after(world.now_ms,1000);
            for(const auto& cell:member->second.view.room_cells)if(
                s.retained_cell(cell,s.sides.at(member->second.view.side))&&
                !s.newer_room_threat(member->second,cell))room_input[entry.first].push_back(cell);}
        for(auto& input:room_input){s.poll();auto m=s.members.find(input.first);if(m==s.members.end()||m->second.present_tick!=s.tick)continue;
            auto& side=s.sides.at(m->second.view.side);
            std::map<std::pair<std::uint64_t,std::uint32_t>,std::vector<RoomCellFact>> batches;
            for(const auto& cell:input.second){s.poll();const auto key=std::make_pair(cell.attempt,cell.room);
                s.remember_cell(m->second,cell);
                auto& list=batches[key];auto prior=std::find_if(list.begin(),list.end(),[&](const RoomCellFact& old){return Impl::same_cell(old,cell);});
                if(prior==list.end())list.push_back(cell);else if(prior->observed_at_ms<cell.observed_at_ms)*prior=cell;}
            unsigned packets=0;
            for(auto& group:batches){s.poll();if(packets>=2)break;const auto key=group.first;
                const auto prior=m->second.room_broadcast_at.find(key);
                // A measured cell may refresh on every capture. Coalesce those
                // refreshes instead of issuing a new packet every frame.
                if(prior!=m->second.room_broadcast_at.end()&&world.now_ms>=prior->second&&world.now_ms-prior->second<1000)continue;
                std::vector<RoomCellFact> needed;
                for(const auto& cell:group.second){s.poll();bool missing=false;
                    for(const auto& peer:side.members){auto receiver=s.members.find(peer);
                        if(receiver!=s.members.end()&&receiver->second.present_tick==s.tick&&receiver->second.view.alive&&
                            receiver->first!=m->first&&!Impl::has_cell(receiver->second,cell)){missing=true;break;}}
                    if(missing)needed.push_back(cell);}
                if(needed.empty())continue;
                constexpr std::size_t packet_cap=32;
                auto& cursor=m->second.room_broadcast_cursor[key];if(cursor>=needed.size())cursor=0;
                std::vector<RoomCellFact> packet;packet.reserve((std::min)(packet_cap,needed.size()));
                for(std::size_t i=0;i<(std::min)(packet_cap,needed.size());++i)packet.push_back(needed[(cursor+i)%needed.size()]);
                cursor=(cursor+packet.size())%needed.size();s.queue_cells(m->second,side,packet);
                m->second.room_broadcast_at[key]=world.now_ms;++packets;
            }}
        // Source-qualified passive casualty observations enter the same real
        // delayed radio/voice bus as own status reports, never enemy knowledge.
        for(const auto& fact:observations){s.poll();
            auto observer=actors.find(fact.observer);auto oi=membership.find(fact.observer),pi=membership.find(fact.patient);
            const bool valid_fact=valid(fact.patient)&&fact.patient!=fact.observer&&fact.visible&&fact.friendly&&
                (!fact.alive||fact.wounded)&&finite3(fact.position)&&fact.level_epoch==world.level_epoch&&fact.snapshot_seq==world.sequence&&
                fact.at_ms==world.now_ms&&observer!=actors.end()&&oi!=membership.end()&&pi!=membership.end()&&
                oi->second==fact.side&&pi->second==fact.side&&observer->second->alive&&
                observer->second->observed_at_ms==fact.at_ms&&observer->second->faction==fact.faction&&
                observer->second->ownership_generation==fact.observer_owner&&observer->second->restrictions_revision==fact.observer_restrictions;
            if(!valid_fact){++output.invalid_facts;continue;}
            auto mi=s.members.find(fact.observer);if(mi==s.members.end()){++output.invalid_facts;continue;}
            auto& m=mi->second;auto& side=s.sides.at(fact.side);
            if(side.faction!=fact.faction){++output.invalid_facts;continue;}
            // Seeing the same body every tick is not a succession of new losses.
            const auto prior=std::find_if(m.view.board.begin(),m.view.board.end(),[&](const BuddyFact& b){return b.kind==ReportKind::man_down&&b.member==fact.patient&&recent(b.at_ms,world.now_ms,s.cfg.forget_ms);});
            if(prior!=m.view.board.end())continue;
            Impl::Report report{};report.sender=fact.observer;report.kind=ReportKind::man_down;report.foe=real(fact.patient);
            report.fact={true,fact.position,fact.at_ms,fact.observer,SoundKind::other};report.sender_position=observer->second->position;
            s.receive(m,side,report); // the real observer knows immediately
            s.queue(m,side,report.kind,report.foe,report.fact);
        }
        for(auto& side:s.sides)s.deliver(side.second);
        std::map<std::uint32_t,std::set<FoeId,FoeLess>> retained_ghosts;
        for(auto& member:s.members){s.poll();auto& m=member.second;auto& side=s.sides.at(m.view.side);
            if(m.present_tick==s.tick)s.assemble_rooms(m,side);else m.view.communicated_rooms.clear();
            s.expire(m,side);
            for(const auto& f:m.foes){s.poll();if(f.first.ghost())retained_ghosts[m.view.side].insert(f.first);}
            if(m.present_tick==s.tick){s.choose(m);output.members.push_back(m.view);}}
        for(auto& pair:s.sides){auto& side=pair.second;std::set<FoeId,FoeLess> known,unseen;for(const auto& key:side.members){auto m=s.members.find(key);if(m==s.members.end()||m->second.present_tick!=s.tick||!m->second.view.alive)continue;
                for(const auto& f:m->second.foes){const auto* k=f.second.last_known();if(k&&recent(k->at_ms,world.now_ms,s.cfg.assess_foe_ms)){if(f.first.ghost())unseen.insert(f.first);else known.insert(f.first);}}}
            for(auto i=side.downs.begin();i!=side.downs.end();)if(!recent(i->second,world.now_ms,s.cfg.forget_ms))i=side.downs.erase(i);else ++i;
            side.view.known_count=known.size();side.view.unseen_count=unseen.size();side.view.confirmed_down.clear();for(const auto& down:side.downs)side.view.confirmed_down.push_back(down.first);if(current_sides.count(pair.first))output.sides.push_back(side.view);
            for(auto i=side.born.begin();i!=side.born.end();)if(!recent(i->second.at,world.now_ms,s.cfg.ghost_keep_ms))i=side.born.erase(i);else ++i;
            // A resolved tombstone outlives the last report (<= one pending tick).
            std::set<FoeId,FoeLess> pending;for(const auto& r:side.reports)if(r.foe.ghost())pending.insert(r.foe);
            for(auto i=side.resolved.begin();i!=side.resolved.end();)if(!side.born.count(i->first)&&!pending.count(i->first))i=side.resolved.erase(i);else ++i;
            const auto refs=retained_ghosts.find(pair.first);
            for(auto i=side.ghost_origins.begin();i!=side.ghost_origins.end();){s.poll();
                if((refs==retained_ghosts.end()||!refs->second.count(i->first))&&!pending.count(i->first)&&!side.born.count(i->first)&&!side.resolved.count(i->first)){
                    side.ghost_ids.erase(i->second);i=side.ghost_origins.erase(i);
                }else ++i;}}
        if(cancel&&cancel())throw Stop{};s.out=nullptr;s.world=nullptr;s.context=nullptr;s.services=nullptr;s.cancelled=nullptr;s.poisoned=false;output.accepted=true;return output;
    }catch(const Stop&){KnowledgeFrame result;result.level_epoch=world.level_epoch;result.sequence=world.sequence;result.cancelled=true;result.rejection=KnowledgeReject::cancelled;return result;}
    catch(...){KnowledgeFrame result;result.level_epoch=world.level_epoch;result.sequence=world.sequence;result.rejection=KnowledgeReject::failed;return result;}
}
}}}
