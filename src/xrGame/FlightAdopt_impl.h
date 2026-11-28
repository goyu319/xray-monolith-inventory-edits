// Owner-thread visual detailing never alters the simulated shell.
bool poll_detail_impl(Owner& entry) {
    using namespace flight_detail;
    if(!entry.flight)return false;
    auto& state=*entry.flight;
    if(entry.geometry!=1){entry.flight.reset();return false;}
    const u32 now=Device.dwTimeGlobal,frame=Device.dwFrame;
    bool preparedNow=false;
    if(state.retryAfter&&s32(now-state.retryAfter)<0)return state.phase==Phase::Waiting;
    if(state.phase!=Phase::Admitted){
        auto exact=state.member.group->exact(now);
        const auto& group=*state.member.group;
        if(state.phase==Phase::Waiting&&(group.failures>=2||group.exhausted||state.validationFailures>=2)){
            entry.session->resume_flight_birth();
            state.phase=Phase::Recovering;
            Msg("[painter_flight] id=%u phase=survivor_gameplay_restored component=%u revision=%llu",entry.object->ID(),state.member.component,entry.first_revision);
        }
        if(group.exhausted||state.validationFailures>=3){
            Msg("[painter_flight] id=%u phase=exact_unavailable component=%u revision=%llu",entry.object->ID(),state.member.component,entry.first_revision);
            entry.flight.reset();return false;
        }
        if(!exact||!prepares.take(frame))return state.phase==Phase::Waiting;
        if(state.phase==Phase::Recovering&&entry.session->completed())return false;
        painter_flight_perf::Scope tick("refine");
        const auto& member=state.member;
        const auto piece=std::find_if(exact->pieces.begin(),exact->pieces.end(),[&](const auto& p){return p.component==member.component;});
        if(piece==exact->pieces.end()||!piece->seed||piece->component==exact->source_component){
            ++state.validationFailures;state.retry(now,"component",entry.object->ID());return state.phase==Phase::Waiting;
        }
        std::vector<Mesh> raw{piece->mesh};
        std::vector<std::vector<part_child_ownership::Decision>> plan{member.plan};
        if(const char* issue=part_child_ownership::validate_part_packets(raw,member.roles,plan)){
            ++state.validationFailures;state.retry(now,issue,entry.object->ID());return state.phase==Phase::Waiting;
        }
        auto packets=part_child_ownership::build_part_packets(std::move(raw),member.roles,plan);
        auto canonical=std::make_shared<Mesh>(std::move(packets.front()));
        restamp(*canonical,entry.identity.spawn_cookie,0,entry.first_revision);
        // The later draw publication must witness the live display generation.
        auto display=std::make_shared<Mesh>(*canonical);
        restamp(*display,entry.identity.spawn_cookie,entry.geometry,entry.first_revision);
        if(!entry.session->finish_flight_birth(entry.seed,piece->seed,canonical,entry.geometry,entry.first_revision)){
            ++state.validationFailures;state.retry(now,"canonical_session",entry.object->ID());return state.phase==Phase::Waiting;
        }
        entry.seed=piece->seed;entry.birth=canonical;
        state.display=std::move(display);
        state.phase=Phase::Admitted;state.member.group.reset();
        preparedNow=true;
    }
    if(entry.session->completed())return false;
    if(state.ticket&&!state.ticket->ready())return false;
    if(!state.ticket&&!preparedNow&&!prepares.take(frame))return false;
    if(state.ticket&&!commits.take(frame))return false;
    painter_flight_perf::Scope tick("refine");
    auto failedDisplay=[&](const char* why){
        if(++state.displayFailures<3){state.retry(now,why,entry.object->ID());return;}
        Msg("[painter_flight] id=%u phase=exact_visual_unavailable component=%u why=%s",entry.object->ID(),state.member.component,why);
        entry.flight.reset();
    };
    RuntimeSkinUpdate update;
    update.owner_cookie=entry.identity.spawn_cookie;update.expected_geometry_generation=entry.geometry;
    update.damage_revision=entry.first_revision;update.separation_packet=true;
    update.flight_detail=true;update.exact_rig=entry.rig;
    if(!state.ticket){
        auto ticket=std::make_shared<painter_damaged_fragment::RenderTicket>();
        ticket->mesh=state.display;
        int outcome=RuntimeStageOutcome::unavailable;
        ticket->handle=entry.visual->LL_StageRuntimeGeometry(update,state.display,"models\\model","wm\\painter_stump_limb",
            entry.textures.empty()?nullptr:&entry.textures,entry.identity.spawn_cookie,outcome);
        if(!ticket->handle){failedDisplay("detail_stage");return false;}
        ticket->attempted=true;state.ticket=std::move(ticket);return false;
    }
    PainterBurnPublication burnPublication(entry.visual);
    if(!burnPublication.ready())return false;
    if(!entry.visual->LL_InstallStagedRuntimeGeometry(state.ticket->handle,update,entry.identity.spawn_cookie)){
        failedDisplay("detail_binding");return false;
    }
    const auto component=state.member.component,elapsed=frame-state.firstFrame;
    entry.flight.reset();
    Msg("[painter_flight] id=%u phase=exact_visible component=%u revision=%llu frame=%u proxy_frames=%u",
        entry.object->ID(),component,entry.first_revision,frame,elapsed);
    return false;
}

bool recover_flight_attempt(Owner& entry,const char* why)noexcept {
    using flight_detail::Phase;
    if(!entry.flight)return false;
    auto& state=*entry.flight;
    const bool admitted=state.phase==Phase::Admitted;
    unsigned& failures=admitted?state.displayFailures:state.validationFailures;
    ++failures;
    if(!admitted&&state.phase==Phase::Waiting&&failures>=2){
        entry.session->resume_flight_birth();
        state.phase=Phase::Recovering;
    }
    state.retryAfter=Device.dwTimeGlobal+250;
    state.ticket.reset();
    const bool waiting=state.phase==Phase::Waiting;
    const auto component=state.member.component;
    const auto count=failures;
    if(count>=3)entry.flight.reset();
    try{
        Msg("[painter_flight] id=%u phase=detail_exception component=%u admitted=%u attempt=%u why=%s",
            entry.object->ID(),component,unsigned(admitted),count,why);
    }catch(...){}
    return count<3&&waiting;
}
bool poll_detail(Owner& entry)noexcept {
    try{return poll_detail_impl(entry);}
    catch(const std::bad_alloc&){return recover_flight_attempt(entry,"allocation_failed");}
    catch(...){return recover_flight_attempt(entry,"publication_exception");}
}
