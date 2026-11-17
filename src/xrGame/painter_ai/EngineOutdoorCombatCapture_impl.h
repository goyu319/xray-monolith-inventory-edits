// Included by EngineCombatAdapter.cpp inside engine namespace. The native
// pointers are local to this authority-thread call and never enter evidence.
namespace {
struct NativeCaptureBudget {
    std::size_t left=0;
    bool take(std::size_t n=1){if(left<n)return false;left-=n;return true;}
};
core::OutdoorQuerySource outdoor_source(const WorldSnapshot&w,const ActorSnapshot&a,
    const Contact& contact,TimeMs seen_at){
    core::OutdoorQuerySource s;s.actor=a.key;s.level_epoch=w.level_epoch;s.snapshot_seq=w.sequence;
    s.geometry_revision=w.geometry_revision;s.ownership_generation=a.ownership_generation;
    s.restrictions_revision=a.restrictions_revision;s.at_ms=w.now_ms;s.origin_node=a.node_id;
    s.actor_position=a.position;s.focus.actor=contact.key;s.focus_at_ms=seen_at;s.focus_position=contact.position;return s;
}
bool outdoor_static_clear(Fvector const&from,Fvector const&to,NativeCaptureBudget& b,bool&clear){
    if(!b.take())return false;Fvector d;d.sub(to,from);float range=d.magnitude();
    if(!_valid(from)||!_valid(to)||!_valid(range)||range<=EPS)return false;d.div(range);
    collide::rq_result hit;clear=!Level().ObjectSpace.RayPick(from,d,range,collide::rqtStatic,hit,nullptr);return true;
}
bool outdoor_trajectory(CAI_Stalker&actor,CGrenade&grenade,Fvector const&target){
    // The exact native non-door minimum-velocity throw calculation, without
    // equip, painter_throw(), throw_target(), or any mutation of the actor.
    Fvector release=actor.Position();const char* offset="third_person_throw_point_offset";
    if(pSettings->line_exist(grenade.cNameSect(),offset))release.add(pSettings->r_fvector3(grenade.cNameSect(),offset));
    else{release.y+=2.f;Fvector local;actor.XFORM().transform_dir(local,grenade.throw_point_offset());release.add(local);}
    Fvector velocity;velocity.sub(target,release);static const float max_native[]={30.f,40.f,50.f,60.f};
    if(unsigned(g_SingleGameDifficulty)>=4||velocity.magnitude()>max_native[unsigned(g_SingleGameDifficulty)])return false;
    const float gravity=physics_world()->Gravity();if(!_valid(gravity)||gravity<=0)return false;
    const float time=ThrowMinVelTime(velocity,gravity);if(!_valid(time)||time<=0)return false;
    TransferenceToThrowVel(velocity,time,gravity);Fvector collided,box;box.set(0,0,0);collide::rq_results storage;
    return !trajectory_intersects_geometry(time,release,target,velocity,collided,&actor,nullptr,storage,nullptr,nullptr,box);
}
bool outdoor_friends_clear(CAI_Stalker&actor,CGrenade&grenade,Fvector const&target,float policy_radius){
    const float blast=grenade.painter_blast_radius();if(!_valid(blast)||blast<=0||!_valid(policy_radius)||policy_radius<=0)return false;
    const float radius=_max(blast,policy_radius);
    // No hidden enemy census: this native safety query returns only a bool,
    // vetoing every live non-hostile body including the actor/player.
    xr_vector<CObject*> neighbors;Level().ObjectSpace.GetNearest(neighbors,target,radius,nullptr);
    for(CObject*o:neighbors){auto body=smart_cast<CEntityAlive*>(o);if(body&&body->g_Alive()&&
        (body==&actor||!actor.is_relation_enemy(body)))return false;}
    return true;
}
}
void capture_outdoor_combat(const WorldSnapshot&w,core::OutdoorCaptureEvidence&out,
    const core::OutdoorDispatchPolicy&policy,std::size_t budget,const core::KnowledgeFrame*known){
    out.solo_aim.clear();out.solo_lean.clear();out.solo_throws.clear();
    out.search_observations.clear();
    static std::uint64_t epoch=0;static u32 frame=u32(-1);static std::uint32_t after=invalid_id;
    if(epoch!=w.level_epoch){epoch=w.level_epoch;frame=u32(-1);after=invalid_id;}
    if(frame==Device.dwFrame||w.sequence!=captured_sequence||w.level_epoch!=captured_epoch||
        w.geometry_revision!=captured_geometry||out.source.level_epoch!=w.level_epoch||
        out.source.snapshot_seq!=w.sequence||out.source.geometry_revision!=w.geometry_revision||out.source.at_ms!=w.now_ms||!policy.solo.enabled)return;
    frame=Device.dwFrame;NativeCaptureBudget work{budget};
    std::map<std::uint32_t,const ActorSnapshot*> actors;
    for(const auto&a:w.actors)if(!actors.emplace(a.key.id,&a).second)return;
    std::map<std::pair<std::uint32_t,std::uint64_t>,const core::MemberKnowledge*> prior_members;
    if(known&&known->accepted&&!known->cancelled&&known->level_epoch==w.level_epoch&&known->sequence<=w.sequence)
        for(const auto& member:known->members){auto added=prior_members.emplace(std::make_pair(member.actor.id,member.actor.generation),&member);if(!added.second)added.first->second=nullptr;}
    std::map<std::uint32_t,Record*> ordered;
    for(const auto&item:records)ordered.emplace(item.second->key.id,item.second.get());
    auto it=ordered.upper_bound(after);if(it==ordered.end())it=ordered.begin();
    for(std::size_t count=0;count<ordered.size();++count){
        Record&r=*it->second;if(work.left)after=it->first;if(++it==ordered.end())it=ordered.begin();
        auto ai=actors.find(r.key.id);if(ai==actors.end()||ai->second->key!=r.key||!registered(r)||!r.owned)continue;
        const auto&a=*ai->second;auto&actor=*r.object;
        if(!a.alive||a.yield_reasons||a.ownership_generation!=r.ownership||actor.painter_forced_action()!=ids::action)continue;
        core::OutdoorCapturedEye eye;if(!measured_outdoor_eye(w,a,value(actor.eye_matrix.c),!!_valid(actor.eye_matrix),eye))continue;
        if(work.left&&r.has_order&&!r.provisional&&r.order.kind==OrderKind::search){
            const auto prior=prior_members.find({a.key.id,a.key.generation});
            if(prior!=prior_members.end()&&prior->second){
                core::OutdoorSensorQuota quota{work.left,0};
                auto observation=core::measure_outdoor_search_observation(combat_config(),w,a,eye,r.order,*prior->second,quota,
                    [&](const Segment&ray){Fvector from=native(ray.a),direction;direction.sub(native(ray.b),from);const float length=direction.magnitude();
                        if(!_valid(from)||!_valid(direction)||!_valid(length)||length<=EPS)return Evidence::unknown;
                        direction.div(length);collide::rq_result hit;
                        return Level().ObjectSpace.RayPick(from,direction,length,collide::rqtBoth,hit,&actor)?Evidence::no:Evidence::yes;});
                work.left=quota.queries_left;
                if(core::valid_key(observation.source.actor))out.search_observations.push_back(std::move(observation));
            }
        }
        // Attained pose is cheap actor data, not a geometry query. Every owned
        // actor receives it even when the shared native query budget is empty.
        // Workers test the current knowledge target once against static geometry.
        if(policy.solo.combat.enabled){
            core::OutdoorSoloLeanCapture proof;
            proof.source=outdoor_source(w,a,Contact{},0);proof.measured_eye=eye.eye;
            proof.pose_only=true;proof.gate="pose_unknown";u8 pose=0;
            if(actor.animation().painter_attained_pose(pose)&&
                (pose==MonsterSpace::eBodyStateStand||pose==MonsterSpace::eBodyStateCrouch)){
                proof.attained_stance=pose==MonsterSpace::eBodyStateCrouch?Stance::crouch:Stance::stand;
                proof.measured=true;proof.gate="pose_measured";
            }
            out.solo_lean.push_back(proof);
        }
        if(!work.left)continue;
        const Contact*focus=nullptr;Contact reported;bool report_focus=false;TimeMs seen_at=0;
        if(a.enemy.visible&&a.enemy.hostile&&a.enemy.alive_when_observed&&a.enemy.last_seen_ms&&a.enemy.last_seen_ms<=w.now_ms&&a.enemy.observed_at_ms<=w.now_ms&&finite_value(a.enemy.position)){
            focus=&a.enemy;seen_at=w.now_ms;
        }else if(r.outdoor_seen_at&&r.outdoor_seen_at<=w.now_ms&&w.now_ms-r.outdoor_seen_at<combat_config().knowledge.forget_ms){
            focus=&r.outdoor_seen;seen_at=r.outdoor_seen_at;
        }
        if(!focus){auto prior=prior_members.find({a.key.id,a.key.generation});
            if(prior!=prior_members.end()&&prior->second&&prior->second->ownership_generation==a.ownership_generation&&prior->second->alive&&prior->second->able){
                const auto& member=*prior->second;
                for(const auto& foe:member.foes){if(!work.take())break;if(foe.id!=member.target||foe.id.ghost()||foe.down||foe.wounded)continue;
                    const auto* fact=foe.last_known();if(!fact||!fact->valid||!core::valid_key(fact->reporter)||!finite_value(fact->position)||
                        !core::fresh(fact->at_ms,w.now_ms,combat_config().memory_max_age_ms))break;
                    // This temporary value carries the already-received K1 point
                    // into geometry queries only. It is never a native Contact.
                    reported.key=foe.id.actor;reported.position=fact->position;reported.hostile=true;reported.alive_when_observed=true;
                    focus=&reported;seen_at=fact->at_ms;report_focus=true;break;
                }
            }
        }
        if(!focus)continue;
        const auto source=outdoor_source(w,a,*focus,seen_at);const Fvector target=native(focus->position);
        auto enemy=actor.memory().enemy().selected();const bool visible=focus==&a.enemy&&enemy&&key_of(*enemy)==focus->key&&actor.memory().visual().visible_now(enemy);
        if(visible&&a.last_applied.order_seq&&a.last_applied.weapon>=1&&a.last_applied.weapon<=2&&work.take(2)){
            // Native cone/friend verdict plus an ACTUAL muzzle direction hit
            // on this exact visible target; an ordinary eye ray is insufficient.
            const u32 flags=actor.painter_target_fire_flags(enemy);Fvector muzzle,direction;actor.g_fireParams(nullptr,muzzle,direction);
            const float range=target.distance_to(muzzle)+enemy->Radius();collide::rq_result hit;
            if(_valid(muzzle)&&_valid(direction)&&_valid(range)&&range>0&&direction.square_magnitude()>EPS){
                direction.normalize();const bool exact=Level().ObjectSpace.RayPick(muzzle,direction,range,collide::rqtBoth,hit,&actor)&&hit.O==enemy;
                core::OutdoorSoloAimEvidence proof;proof.source=source;proof.applied_order_seq=a.last_applied.order_seq;proof.measured_eye=eye.eye;
                proof.weapon_line_clear=exact&&(flags&3)==3?Evidence::yes:Evidence::no;
                proof.teammate_line_clear=(flags&8)&&!(flags&4)?Evidence::yes:Evidence::no;out.solo_aim.push_back(proof);
            }
        }
        if(!report_focus&&!visible&&!r.throwing.active(r.ownership)&&a.grenades.count&&a.grenades.slot_item!=invalid_id&&r.throwing.serial<(std::numeric_limits<std::uint32_t>::max)()&&work.left>=3){
            auto grenade=smart_cast<CGrenade*>(actor.inventory().Get(a.grenades.slot_item,true));if(!grenade||armed_item(grenade))continue;
            const float distance=actor.Position().distance_to(target);if(distance<policy.solo.throw_min_m||distance>policy.solo.throw_max_m)continue;
            const float target_height=policy.solo.combat.target_height_m;if(!_valid(target_height)||target_height<=0)continue;
            Fvector threat_point=target;threat_point.y+=target_height;
            bool exposed=true;if(!outdoor_static_clear(actor.eye_matrix.c,threat_point,work,exposed)||exposed)continue;
            if(!work.take()||!outdoor_trajectory(actor,*grenade,target))continue;
            if(!work.take()||!outdoor_friends_clear(actor,*grenade,target,policy.solo.throw_friend_m))continue;
            core::OutdoorSoloThrow proof;proof.source=source;proof.item=grenade->ID();proof.serial=r.throwing.serial+1;
            proof.protected_start=proof.trajectory_supported=proof.teammates_safe=Evidence::yes;out.solo_throws.push_back(proof);
        }
    }
}
