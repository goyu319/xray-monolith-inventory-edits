#include "CombatOutdoorAssessment.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <limits>
#include <set>
#include <tuple>
namespace painter { namespace combat { namespace core {
namespace {
struct Cancelled{};
void poll(const std::function<bool()>& c){if(c&&c())throw Cancelled{};}
using Key=std::pair<std::uint32_t,std::uint64_t>;
Key key(ActorKey k){return {k.id,k.generation};}
bool positive(float n){return std::isfinite(n)&&n>0;}
bool equal(Vec3 a,Vec3 b){return a.x==b.x&&a.y==b.y&&a.z==b.z;}
float flat(Vec3 a,Vec3 b){return std::hypot(a.x-b.x,a.z-b.z);}
OutdoorStamp stamp(const WorldSnapshot&w,const SideSnapshot&s){return {w.level_epoch,w.map_hash,w.geometry_revision,w.sequence,s.group,w.now_ms,s.side_id,s.faction,s.mission.revision};}
bool same(const OutdoorStamp&a,const OutdoorStamp&b){return a.level_epoch==b.level_epoch&&a.map_hash==b.map_hash&&a.geometry_revision==b.geometry_revision&&
    a.snapshot_seq==b.snapshot_seq&&a.group==b.group&&a.at_ms==b.at_ms&&a.side==b.side&&a.faction==b.faction&&a.mission_revision==b.mission_revision;}
bool context(const OutdoorStamp&a,const OutdoorStamp&b){auto x=a,y=b;x.snapshot_seq=y.snapshot_seq=0;x.at_ms=y.at_ms=0;return same(x,y);}
bool mission_same(const Mission&a,const Mission&b){if(a.kind!=b.kind||a.revision!=b.revision||a.building!=b.building||a.entry_portal!=b.entry_portal||a.rooms.count!=b.rooms.count)return false;
    for(std::size_t i=0;i<a.rooms.count;++i)if(a.rooms.items[i]!=b.rooms.items[i])return false;return true;}
bool policy_valid(const OutdoorAssessmentPolicy&p){return p.trend_ms&&p.persist_ms&&p.cooldown_ms&&p.unseen_hit_ms&&p.sound_ms&&p.withdraw_cap_ms&&
    positive(p.unsafe_m)&&positive(p.unseen_merge_m)&&positive(p.casualty_rally_m)&&positive(p.arrived_m)&&positive(p.safety_height_m)&&
    positive(p.steady_health)&&p.steady_health<=1&&positive(p.brittle_health)&&p.brittle_health<=1&&
    positive(p.steady_able_fraction)&&p.steady_able_fraction<=1&&positive(p.brittle_able_fraction)&&p.brittle_able_fraction<=1;}
bool current(const ActorSnapshot&a,const WorldSnapshot&w,const Config&c){return valid_key(a.key)&&a.level_epoch==w.level_epoch&&a.snapshot_seq==w.sequence&&
    fresh(a.observed_at_ms,w.now_ms,c.snapshot_max_age_ms)&&finite(a.position)&&std::isfinite(a.health);}
bool combat(OutdoorPosture p){return p==OutdoorPosture::contact||p==OutdoorPosture::defend||p==OutdoorPosture::attack||p==OutdoorPosture::assault;}
bool order_applied(const Order&o,const ActorSnapshot&a,const WorldSnapshot&w){return o.actor==a.key&&o.level_epoch==w.level_epoch&&o.geometry_revision==w.geometry_revision&&
    o.ownership_generation==a.ownership_generation&&o.restrictions_revision==a.restrictions_revision&&o.snapshot_seq<=w.sequence&&
    o.order_seq&&o.order_seq==a.last_applied.order_seq&&a.accessible_current_order&&a.current_position_accessible&&o.goal_node!=invalid_node&&finite(o.goal_position)&&
    o.issued_at_ms<=a.last_applied.at_ms&&a.last_applied.at_ms<=a.observed_at_ms&&o.expires_at_ms>w.now_ms;}
bool fact_equal(const SourceFact&a,const SourceFact&b){return a.valid&&b.valid&&a.at_ms==b.at_ms&&a.reporter==b.reporter&&a.sound==b.sound&&equal(a.position,b.position);}
const SourceFact* source(const FoeKnowledge&f,TimeMs now,TimeMs max_age){const SourceFact* best=nullptr;for(const auto&x:f.sources)
    if(x.valid&&valid_key(x.reporter)&&finite(x.position)&&fresh(x.at_ms,now,max_age)&&(!best||x.at_ms>best->at_ms))best=&x;return best;}
struct Known {FoeId foe;SourceFact source;};
}
CombatOutdoorAssessment::CombatOutdoorAssessment(Config c,OutdoorAssessmentPolicy p):config_(std::move(c)),policy_(p){}
void CombatOutdoorAssessment::reset(){CombatOutdoorAssessment fresh_state(config_,policy_);*this=std::move(fresh_state);}
OutdoorAssessmentResult CombatOutdoorAssessment::update(const WorldSnapshot&w,const SideSnapshot&s,const KnowledgeFrame&k,
    const OutdoorAssessmentRequest&request,const std::function<bool()>&cancelled){
    OutdoorAssessmentResult fail;fail.stamp=stamp(w,s);
    try{poll(cancelled);
        if(!policy_valid(policy_)||!w.level_epoch||!w.sequence||!s.side_id||s.members.empty()||s.mission.rooms.count>s.mission.rooms.items.size()||
            config_.factions.count>config_.factions.items.size()||!k.accepted||k.cancelled||k.level_epoch!=w.level_epoch||k.sequence!=w.sequence){fail.reason="invalid_policy_or_capture";return fail;}
        if(initialized_&&(w.level_epoch<result_.stamp.level_epoch||(w.level_epoch==result_.stamp.level_epoch&&
            (w.sequence<result_.stamp.snapshot_seq||w.now_ms<result_.stamp.at_ms)))){fail.status=OutdoorStatus::stale;fail.reason="backwards_capture";return fail;}
        if(initialized_&&w.level_epoch==result_.stamp.level_epoch&&w.sequence==result_.stamp.snapshot_seq){
            if(same(fail.stamp,result_.stamp)&&mission_same(mission_,s.mission))return result_;
            fail.status=OutdoorStatus::stale;fail.reason="changed_duplicate_capture";return fail;}
        std::map<Key,const ActorSnapshot*> actors;std::map<Key,const MemberKnowledge*> knowledge;std::set<std::uint32_t> ids;
        for(const auto&a:w.actors){poll(cancelled);if(!valid_key(a.key)||!ids.insert(a.key.id).second){fail.reason="ambiguous_actors";return fail;}actors.emplace(key(a.key),&a);}
        for(const auto&m:k.members){poll(cancelled);if(!valid_key(m.actor)||!knowledge.emplace(key(m.actor),&m).second){fail.reason="ambiguous_knowledge";return fail;}}
        std::set<Key> roster;for(auto a:s.members){poll(cancelled);if(!valid_key(a)||!roster.insert(key(a)).second){fail.reason="ambiguous_roster";return fail;}}
        auto get=[&](ActorKey a)->const ActorSnapshot*{auto i=actors.find(key(a));return i==actors.end()?nullptr:i->second;};
        auto km=[&](ActorKey a)->const MemberKnowledge*{const auto* own=get(a);auto it=knowledge.find(key(a));
            return own&&current(*own,w,config_)&&it!=knowledge.end()&&it->second->side==s.side_id&&it->second->ownership_generation==own->ownership_generation?it->second:nullptr;};
        bool continuity=false,lease_changed=false;for(const auto&old:members_){poll(cancelled);const auto*a=get(old.actor);if(roster.count(key(old.actor))&&a){
            continuity|=a->ownership_generation==old.owner;lease_changed|=a->ownership_generation!=old.owner;}}
        const bool reset_state=!initialized_||!context(result_.stamp,fail.stamp)||!mission_same(mission_,s.mission)||!continuity;
        auto n=*this;if(reset_state)n.reset();auto&r=n.result_;r={};r.stamp=fail.stamp;r.status=OutdoorStatus::ready;r.reset=reset_state;r.roster=s.members.size();
        if(lease_changed&&!n.executing_){n.pending_=OutdoorAssessment::unknown;n.pending_at_=0;n.active_serial_=0;}
        n.initialized_=true;n.mission_=s.mission;
        const FactionStyle*style=&config_.faction_default;for(std::size_t i=0;i<config_.factions.count;++i)if(config_.factions.items[i].faction==s.faction){style=&config_.factions.items[i];break;}
        if(style->morale>2){fail.reason="invalid_faction_policy";return fail;}
        std::vector<Member> current_members;std::vector<const ActorSnapshot*> able;std::vector<const MemberKnowledge*> minds;
        for(auto akey:s.members){poll(cancelled);const auto*a=get(akey);if(!a||!current(*a,w,config_)){++r.unknown;continue;}
            if(a->faction!=s.faction){fail.reason="faction_conflict";return fail;}++r.observed;
            if(const auto*m=km(akey))minds.push_back(m);
            auto old=std::find_if(n.members_.begin(),n.members_.end(),[&](const Member&m){return m.actor==akey;});
            if(old!=n.members_.end()&&old->alive&&(!a->alive||(!old->wounded&&a->wounded))&&
                std::none_of(n.losses_.begin(),n.losses_.end(),[&](const Loss&l){return l.actor==akey;}))n.losses_.push_back({akey,a->ownership_generation,w.now_ms});
            current_members.push_back({akey,a->ownership_generation,a->alive,a->wounded,w.now_ms});
            if(!a->alive)++r.dead;else if(a->wounded)++r.hurt;else{++r.able;r.mean_health+=std::clamp(a->health,0.f,1.f);able.push_back(a);}}
        for(const auto& old:n.members_){poll(cancelled);
            if(fresh(old.observed_at,w.now_ms,policy_.trend_ms)&&
               std::none_of(current_members.begin(),current_members.end(),[&](const Member& m){return m.actor==old.actor;}))
                current_members.push_back(old);
        }
        n.members_=std::move(current_members);if(r.able)r.mean_health/=float(r.able);
        auto hit=[&](ActorKey actor,TimeMs at){
            if(!at||!fresh(at,w.now_ms,policy_.trend_ms))return;
            const auto* observed=get(actor);if(!observed||at>observed->observed_at_ms)return;
            if(std::none_of(n.hits_.begin(),n.hits_.end(),[&](const TakenHit& x){return x.actor==actor&&x.at_ms==at;}))
                n.hits_.push_back({actor,at});
        };
        for(const auto* a:able){poll(cancelled);if(a->hits.count<=a->hits.items.size())
            for(std::size_t j=0;j<a->hits.count;++j)if(finite(a->hits.items[j].dir))hit(a->key,a->hits.items[j].at_ms);}
        for(const auto* a:able){poll(cancelled);
            if(a->witnessed_severs.count>a->witnessed_severs.items.size())continue;
            for(std::size_t j=0;j<a->witnessed_severs.count;++j){const auto& e=a->witnessed_severs.items[j];
                if(!valid_key(e.subject)||!e.revision||e.observer_owner!=a->ownership_generation||
                    !e.at_ms||e.at_ms>a->observed_at_ms||!fresh(e.at_ms,w.now_ms,policy_.trend_ms))continue;
                if(std::none_of(n.sever_shocks_.begin(),n.sever_shocks_.end(),[&](const SeverShock& old){return
                    old.observer==a->key&&old.subject==e.subject&&old.revision==e.revision&&old.component==e.component;}))
                    n.sever_shocks_.push_back({a->key,e.subject,e.observer_owner,e.revision,e.component,e.at_ms});
            }
        }
        n.sever_shocks_.erase(std::remove_if(n.sever_shocks_.begin(),n.sever_shocks_.end(),[&](const SeverShock& e){
            const auto* observer=get(e.observer);return !observer||!roster.count(key(e.observer))||
                observer->ownership_generation!=e.owner||!fresh(e.at_ms,w.now_ms,policy_.trend_ms);
        }),n.sever_shocks_.end());
        r.witnessed_severs=n.sever_shocks_.size();
        for(const auto* m:minds){poll(cancelled);
            if(m->hit_from.valid&&m->hit_from.observer==m->actor&&finite(m->hit_from.direction)&&finite(m->hit_from.point))hit(m->actor,m->hit_from.at_ms);
            // A personally received man-down report can retain a casualty
            // whose body is no longer an admitted ActorSnapshot. Absence alone
            // remains unknown and never creates a death.
            for(const auto& fact:m->board){poll(cancelled);if(fact.kind!=ReportKind::man_down||
                (!roster.count(key(fact.member))&&std::none_of(n.members_.begin(),n.members_.end(),[&](const Member& prior){return prior.actor==fact.member;}))||
                !fact.at_ms||fact.at_ms>get(m->actor)->observed_at_ms||!fresh(fact.at_ms,w.now_ms,policy_.trend_ms))continue;
                if(std::none_of(n.losses_.begin(),n.losses_.end(),[&](const Loss& loss){return loss.actor==fact.member;}))
                    n.losses_.push_back({fact.member,0,fact.at_ms});}
        }
        n.hits_.erase(std::remove_if(n.hits_.begin(),n.hits_.end(),[&](const TakenHit& h){return !roster.count(key(h.actor))||!fresh(h.at_ms,w.now_ms,policy_.trend_ms);}),n.hits_.end());
        r.hits_taken=n.hits_.size();
        n.losses_.erase(std::remove_if(n.losses_.begin(),n.losses_.end(),[&](const Loss&l){return !fresh(l.at_ms,w.now_ms,policy_.trend_ms);}),n.losses_.end());r.ours=n.losses_.size();
        const auto*lead=get(request.leader);const auto*leader=roster.count(key(request.leader))?km(request.leader):nullptr;
        std::vector<Known> foes;std::vector<Vec3> ghost_points;
        auto known_down=[&](FoeId foe,TimeMs at){
            if(!foe.valid()||foe.ghost()||roster.count(key(foe.actor))||!at||!fresh(at,w.now_ms,policy_.trend_ms))return;
            if(std::none_of(n.downs_.begin(),n.downs_.end(),[&](const Down& d){return d.foe==foe;}))n.downs_.push_back({foe,at});
        };
        n.observed_downs_.erase(std::remove_if(n.observed_downs_.begin(),n.observed_downs_.end(),[&](const ObservedDown& d){return !fresh(d.at_ms,w.now_ms,policy_.trend_ms);}),n.observed_downs_.end());
        // K1 deliberately omits dead foes from its published per-man foe list.
        // Capture the actual event's original time, but credit it only when
        // the leader saw it or the bus reports delivery to this leader.
        for(const auto& event:k.events){poll(cancelled);
            if(event.kind!=KnowledgeEventKind::enemy_down||event.side!=s.side_id||!event.foe.valid()||event.foe.ghost()||
                roster.count(key(event.foe.actor))||!event.at_ms||!fresh(event.at_ms,w.now_ms,policy_.trend_ms)||
                !roster.count(key(event.actor))||!km(event.actor)||event.at_ms>get(event.actor)->observed_at_ms)continue;
            if(std::none_of(n.observed_downs_.begin(),n.observed_downs_.end(),[&](const ObservedDown& d){return d.foe==event.foe&&d.reporter==event.actor;}))
                n.observed_downs_.push_back({event.foe,event.actor,event.at_ms});
            if(leader&&event.actor==request.leader)known_down(event.foe,event.at_ms);
        }
        if(leader&&lead&&lead->alive){
            for(const auto& event:k.events){poll(cancelled);
                if(event.kind!=KnowledgeEventKind::report||event.report!=ReportKind::down||!event.delivered||event.side!=s.side_id||
                    event.receiver!=request.leader||event.at_ms>lead->observed_at_ms)continue;
                for(const auto& original:n.observed_downs_)if(original.foe==event.foe&&original.reporter==event.actor&&original.at_ms<=event.at_ms)
                    known_down(original.foe,original.at_ms);
            }
            // Explicit seen_dead is a current personal observation. A naked
            // timestamp-free side aggregate can never invent a new death.
            if(lead->seen_dead.count<=lead->seen_dead.items.size())for(std::size_t j=0;j<lead->seen_dead.count;++j){
                const auto corpse=lead->seen_dead.items[j];bool confirmed=false;
                for(const auto& side:k.sides)if(side.side==s.side_id&&std::find(side.confirmed_down.begin(),side.confirmed_down.end(),corpse)!=side.confirmed_down.end())confirmed=true;
                if(confirmed){FoeId foe;foe.actor=corpse;known_down(foe,lead->observed_at_ms);}
            }
            for(const auto&f:leader->foes){poll(cancelled);if(!f.id.valid()||(!f.id.ghost()&&roster.count(key(f.id.actor))))continue;
                const bool confirmed_down=f.down&&f.down_ms&&f.down_ms<=lead->observed_at_ms&&valid_key(f.status_reporter);
                if(confirmed_down){if(!f.id.ghost()&&
                    std::none_of(n.downs_.begin(),n.downs_.end(),[&](const Down&d){return d.foe==f.id;}))n.downs_.push_back({f.id,f.down_ms});continue;}
                const auto*sf=source(f,w.now_ms,config_.knowledge.assess_foe_ms);if(!sf||sf->at_ms>lead->observed_at_ms)continue;
                bool friendly=false;if(!f.id.ghost()){
                    auto rel=std::find_if(n.relations_.begin(),n.relations_.end(),[&](const Relation&x){return x.observer==request.leader&&x.target==f.id.actor;});
                    TimeMs relation_at=rel==n.relations_.end()?0:rel->at_ms;
                    auto observe=[&](const Contact&t){if(t.key==f.id.actor&&!t.hostile&&t.observed_at_ms&&t.observed_at_ms>=sf->at_ms&&t.observed_at_ms<=lead->observed_at_ms)
                        relation_at=(std::max)(relation_at,t.observed_at_ms);};
                    observe(lead->enemy);for(std::size_t j=0;j<(std::min)(std::size_t(lead->visible_enemies.count),lead->visible_enemies.items.size());++j)observe(lead->visible_enemies.items[j]);
                    if(relation_at){if(rel==n.relations_.end())n.relations_.push_back({request.leader,f.id.actor,relation_at});else rel->at_ms=relation_at;friendly=relation_at>=sf->at_ms;}}
                if(friendly)continue;
                if(!f.id.ghost()&&f.wounded&&f.wounded_ms&&f.wounded_ms<=lead->observed_at_ms&&
                    fresh(f.wounded_ms,w.now_ms,policy_.trend_ms)&&valid_key(f.status_reporter))++r.foe_wounded;
                auto old=std::find_if(foes.begin(),foes.end(),[&](const Known&x){return x.foe==f.id;});
                if(old==foes.end())foes.push_back({f.id,*sf});else if(sf->at_ms>old->source.at_ms)old->source=*sf;
            }
        }
        n.relations_.erase(std::remove_if(n.relations_.begin(),n.relations_.end(),[&](const Relation&x){return x.observer!=request.leader||!leader||
            std::none_of(leader->foes.begin(),leader->foes.end(),[&](const FoeKnowledge&f){return !f.id.ghost()&&f.id.actor==x.target;});}),n.relations_.end());
        for(const auto&f:foes)if(f.foe.ghost())ghost_points.push_back(f.source.position);else ++r.known;
        bool fired_on=false;for(const auto*m:minds){poll(cancelled);const auto&h=m->hit_from;
            if(!h.valid||h.observer!=m->actor||!finite(h.point)||!finite(h.direction)||h.at_ms>get(m->actor)->observed_at_ms)continue;
            fired_on|=fresh(h.at_ms,w.now_ms,policy_.trend_ms);
            if(!h.unseen||!fresh(h.at_ms,w.now_ms,policy_.unseen_hit_ms))continue;
            bool near=false;for(const auto&f:foes)if(!f.foe.ghost()&&flat(h.point,f.source.position)<=policy_.unseen_merge_m)near=true;
            for(auto point:ghost_points)near|=flat(h.point,point)<=policy_.unseen_merge_m;if(!near)ghost_points.push_back(h.point);
        }
        // Ghost count is separate; duplicate attributed hit places are not an
        // additional enemy census or a second copy of the same K1 ghost.
        std::vector<Vec3> unique_ghosts;for(auto p:ghost_points){bool duplicate=false;for(auto q:unique_ghosts)duplicate|=flat(p,q)<=policy_.unseen_merge_m;
            if(!duplicate)unique_ghosts.push_back(p);}r.unseen=unique_ghosts.size();r.downs=n.downs_.size();
        for(const auto&d:n.downs_)if(fresh(d.at_ms,w.now_ms,policy_.trend_ms))++r.theirs;
        r.broken=style->morale!=2&&(r.mean_health<(style->morale==1?policy_.brittle_health:policy_.steady_health)||
            float(r.able)<=float(r.roster)*(style->morale==1?policy_.brittle_able_fraction:policy_.steady_able_fraction));
        r.outmatched=r.known+r.unseen>(std::max)(std::size_t(1),r.able);
        r.losing=r.ours>r.theirs&&r.dead+r.hurt>r.downs&&(r.known||r.unseen||fired_on||r.ours);
        r.pressure=r.ours||fired_on||r.unseen||r.hits_taken||r.witnessed_severs;
        // Exchange cost unit is one observed hit. Losing one available rifle
        // costs more than a full roster's ordinary advance advantage; known
        // enemy incapacitations earn the same credit. No hidden health/census.
        const float casualty_cost=float((std::max)(std::size_t(1),r.roster))+1.f;
        const auto enemies=r.known+r.unseen;
        const float superiority=r.able>enemies?float(r.able-enemies)/float((std::max)(std::size_t(1),r.able)):0.f;
        r.advance_cost=float(r.ours)*casualty_cost+float(r.hits_taken);
        // A visible committed sever makes pressing costly; it never proves
        // death, foe location, or a safe withdrawal. Faction morale is policy.
        if(style->morale!=2)r.advance_cost+=float((std::min)(r.witnessed_severs,r.able))*casualty_cost;
        r.hold_cost=float(r.theirs+r.foe_wounded)*casualty_cost+superiority+(style->doctrine==1?0.f:1.f);
        r.exchange_hold=r.advance_cost>r.hold_cost;
        for(const auto& loss:n.losses_)r.exchange_at_ms=(std::max)(r.exchange_at_ms,loss.at_ms);
        for(const auto& h:n.hits_)r.exchange_at_ms=(std::max)(r.exchange_at_ms,h.at_ms);
        for(const auto& shock:n.sever_shocks_)r.exchange_at_ms=(std::max)(r.exchange_at_ms,shock.at_ms);
        if(r.exchange_at_ms)r.exchange_until_ms=saturated_after(r.exchange_at_ms,policy_.trend_ms);
        // Holding is assessed again as new damage arrives. A later loss/hit
        // after a completed persistence window can make the protected fallback
        // preferable; existing route, casualty and cooldown gates still apply.
        const bool failing_hold=r.exchange_hold&&n.exchange_hold_at_&&request.posture==OutdoorPosture::defend&&
            r.exchange_at_ms>n.exchange_hold_at_&&w.now_ms-n.exchange_hold_at_>=policy_.persist_ms;
        if(r.exchange_hold&&!n.exchange_hold_at_)n.exchange_hold_at_=w.now_ms;
        else if(!r.exchange_hold)n.exchange_hold_at_=0;
        const auto*fallback=request.fallback;
        const bool fallback_valid=fallback&&fallback->revision&&same(fallback->stamp,r.stamp)&&fallback->node!=invalid_node&&finite(fallback->point);
        const char*safety_reason="fallback_unknown";
        if(fallback_valid){r.fallback_node=fallback->node;r.fallback=fallback->point;r.safe_fallback=Evidence::yes;safety_reason="safe";
            std::set<Key> routes;for(const auto&path:fallback->routes){poll(cancelled);if(!routes.insert(key(path.actor)).second){r.safe_fallback=Evidence::unknown;safety_reason="ambiguous_route";}}
            for(const auto*a:able){poll(cancelled);auto route=std::find_if(fallback->routes.begin(),fallback->routes.end(),[&](const OutdoorFallbackRoute&x){return x.actor==a->key;});
                if(route==fallback->routes.end()||route->owner!=a->ownership_generation||route->restrictions_revision!=a->restrictions_revision||route->origin_node!=a->node_id||
                    route->goal_node!=fallback->node||!equal(route->goal_position,fallback->point)||route->route!=Evidence::yes||route->cover!=Evidence::yes){
                    r.safe_fallback=Evidence::unknown;safety_reason="route_or_cover_unknown";}}
            if(!leader||!lead||!lead->alive){r.safe_fallback=Evidence::unknown;safety_reason="leader_knowledge_unknown";}
            for(const auto&f:foes){poll(cancelled);if(flat(f.source.position,fallback->point)<=policy_.unsafe_m){r.safe_fallback=Evidence::no;safety_reason="known_source_near_fallback";continue;}
                const OutdoorFallbackLine*line=nullptr;bool duplicate=false;for(const auto&x:fallback->lines)if(x.foe==f.foe&&fact_equal(x.source,f.source)){duplicate|=line!=nullptr;line=&x;}
                Vec3 from=f.source.position,to=fallback->point;from.y+=policy_.safety_height_m;to.y+=policy_.safety_height_m;
                if(!line||duplicate||!equal(line->ray_from,from)||!equal(line->ray_to,to)||line->visible==Evidence::unknown){if(r.safe_fallback!=Evidence::no){r.safe_fallback=Evidence::unknown;safety_reason="source_line_unknown";}}
                else if(line->visible==Evidence::yes){r.safe_fallback=Evidence::no;safety_reason="fallback_in_known_view";}}
            for(const auto*m:minds)for(const auto&sound:m->contacts){poll(cancelled);if(finite(sound.position)&&valid_key(sound.reporter)&&sound.at_ms<=get(m->actor)->observed_at_ms&&
                fresh(sound.at_ms,w.now_ms,policy_.sound_ms)&&flat(sound.position,fallback->point)<=policy_.unsafe_m){r.safe_fallback=Evidence::no;safety_reason="sound_near_fallback";}}
            r.casualty_obligation_clear=Evidence::yes;if(!style->leave_wounded){if(r.unknown)r.casualty_obligation_clear=Evidence::unknown;
                for(auto akey:s.members){const auto*a=get(akey);if(a&&current(*a,w,config_)&&a->alive&&a->wounded&&flat(a->position,fallback->point)>policy_.casualty_rally_m)r.casualty_obligation_clear=Evidence::no;}}
        }
        const bool exact_execution=request.execution&&same(request.execution->stamp,r.stamp)&&request.executing_serial&&request.executing_serial==n.active_serial_;
        std::map<Key,const OutdoorWithdrawalLegReceipt*> legs;
        if(exact_execution)for(const auto&leg:request.execution_legs){poll(cancelled);
            auto inserted=legs.emplace(key(leg.actor),&leg);if(!inserted.second)inserted.first->second=nullptr;}
        std::set<Key> executed;bool moving=false;
        if(exact_execution)for(const auto&row:request.execution->actors){poll(cancelled);const auto*a=get(row.applied.actor);
            if(!a||!roster.count(key(a->key))||!current(*a,w,config_)||!a->alive||a->wounded||row.applied.kind!=OrderKind::withdraw||!order_applied(row.applied,*a,w))continue;
            const auto li=legs.find(key(a->key));const auto*leg=li==legs.end()?nullptr:li->second;
            if(!leg||!same(leg->stamp,r.stamp)||leg->assessment_serial!=n.active_serial_||leg->owner!=a->ownership_generation||
                leg->restrictions_revision!=a->restrictions_revision||leg->order_seq!=row.applied.order_seq||leg->goal_node!=row.applied.goal_node||
                !equal(leg->goal_position,row.applied.goal_position))continue;
            if(flat(row.applied.goal_position,n.active_point_)>policy_.arrived_m&&flat(row.applied.goal_position,n.active_point_)>=flat(a->position,n.active_point_))continue;
            executed.insert(key(a->key));moving=true;}
        if(moving&&!n.executing_){n.executing_=n.has_withdrawn_=true;n.withdraw_at_=w.now_ms;r.execution_started=true;}
        if(n.has_withdrawn_&&w.now_ms-n.withdraw_at_<policy_.cooldown_ms)r.cooldown_remaining_ms=policy_.cooldown_ms-(w.now_ms-n.withdraw_at_);
        bool awaiting_execution=false;
        if(n.executing_){r.withdrawal_episode_serial=n.active_serial_;bool arrived=!able.empty()&&!r.unknown;for(const auto*a:able)arrived=arrived&&executed.count(key(a->key))&&flat(a->position,n.active_point_)<=policy_.arrived_m;
            r.completed=arrived;r.capped=!arrived&&w.now_ms-n.withdraw_at_>=policy_.withdraw_cap_ms;
            if(r.completed||r.capped){n.executing_=false;n.active_serial_=0;r.verdict=OutdoorAssessment::proceed;r.reason=r.completed?"withdraw_arrival_observed":"withdraw_cap_uncompleted";}
            else {r.withdrawing=true;r.verdict=OutdoorAssessment::withdraw;r.reason="withdraw_executing";r.serial=n.active_serial_;r.fallback=n.active_point_;r.fallback_node=n.active_node_;}
        }else if(request.posture==OutdoorPosture::withdraw&&n.active_serial_){
            // Posture may acknowledge the proposal before any native action is
            // applied. Keep that episode link without spending execution time.
            // Current safety remains a separate gate on issuing any new leg.
            awaiting_execution=true;r.verdict=OutdoorAssessment::withdraw;r.serial=n.active_serial_;
            r.withdrawal_episode_serial=n.active_serial_;r.pending_since_ms=n.pending_at_;
            if(!fallback_valid||r.fallback_node!=n.active_node_||!equal(r.fallback,n.active_point_)||r.unknown||r.able<2){
                r.safe_fallback=Evidence::unknown;r.reason="withdraw_pending_safety_unknown";
            }else r.reason=r.safe_fallback==Evidence::yes&&r.casualty_obligation_clear==Evidence::yes?
                "withdraw_pending_native_ack":"withdraw_pending_safety_unavailable";
            r.fallback_node=n.active_node_;r.fallback=n.active_point_;
        }else if(!combat(request.posture)&&!r.exchange_hold){n.pending_=OutdoorAssessment::unknown;n.pending_at_=0;r.verdict=OutdoorAssessment::proceed;r.reason="noncombat";}
        else {
            OutdoorAssessment desired=OutdoorAssessment::proceed;r.reason="continue";
            if(r.unknown||!leader){desired=OutdoorAssessment::hold;r.reason="incomplete_current_knowledge";}
            else if((r.broken&&(r.outmatched||r.losing))||(failing_hold&&style->morale!=2)){
                desired=OutdoorAssessment::withdraw;r.reason="assessed_withdraw";
                if(r.safe_fallback!=Evidence::yes){desired=OutdoorAssessment::hold;r.reason=safety_reason;}
                else if(r.able<2){desired=OutdoorAssessment::hold;r.reason="last_man_holds";}
                else if(!style->leave_wounded&&r.casualty_obligation_clear!=Evidence::yes){desired=OutdoorAssessment::hold;r.reason="casualty_obligation";}
                else if(r.cooldown_remaining_ms){desired=OutdoorAssessment::hold;r.reason="withdraw_cooldown";}
            }
            if(desired==OutdoorAssessment::proceed&&r.exchange_hold){desired=OutdoorAssessment::hold;r.reason="exchange_hold";}
            if(desired==OutdoorAssessment::hold&&r.exchange_hold){
                // Current damage already makes advancing the more expensive
                // action. Persistence is for committing withdrawal, not for
                // walking into fire while a hold verdict waits in a queue.
                n.pending_=desired;n.pending_at_=w.now_ms;r.pending_since_ms=n.pending_at_;r.verdict=desired;
            }
            else if(desired==OutdoorAssessment::proceed){n.pending_=OutdoorAssessment::unknown;n.pending_at_=0;r.verdict=desired;}
            else {if(n.pending_!=desired){n.pending_=desired;n.pending_at_=w.now_ms;}r.pending_since_ms=n.pending_at_;
                r.verdict=w.now_ms-n.pending_at_>=policy_.persist_ms?desired:OutdoorAssessment::unknown;if(r.verdict==OutdoorAssessment::unknown)r.reason="assessment_pending";}
        }
        if(!r.withdrawing&&!awaiting_execution){const bool changed=reset_state||r.verdict!=result_.verdict||r.fallback_node!=result_.fallback_node||!equal(r.fallback,result_.fallback);
            if(changed||!n.serial_){if(n.serial_==(std::numeric_limits<std::uint64_t>::max)()){fail.reason="serial_exhausted";return fail;}++n.serial_;}
            r.serial=n.serial_;
            if(r.verdict==OutdoorAssessment::withdraw){n.active_serial_=r.serial;r.withdrawal_episode_serial=n.active_serial_;n.active_node_=r.fallback_node;n.active_point_=r.fallback;}
            else if(!n.executing_)n.active_serial_=0;
        }
        poll(cancelled);*this=std::move(n);return result_;
    }catch(const Cancelled&){fail.status=OutdoorStatus::cancelled;fail.reason="cancelled";return fail;}
    catch(...){fail.status=OutdoorStatus::failed;fail.reason="exception";return fail;}
}
}}}
