// Native shader recipe for the one-sample burned torn interior.
static const char* const names[]={"findTorn","intent","traceHistory","traceLive","traceBaked","finishHit","materialTorn"};
static constexpr int parameterCounts[]={19,8,8,8,8,31,25};
static constexpr Pin inputPins[]={{"painter_rest","PAINTER_REST",3,false},
{"painter_L0","PAINTER_LROW0",3,false},
{"painter_L1","PAINTER_LROW1",3,false},
{"painter_L2","PAINTER_LROW2",3,false},
{"tcdh","TEXCOORD0",2,false},
{"position","TEXCOORD1",4,false},
{"vel_curr","VEL_CURRENT",4,false},
{"vel_prev","VEL_PREVIOUS",4,false},
{"painter_hpos","SV_Position",4,false},
{"primitive","SV_PrimitiveID",1,true}};
static constexpr Pin outputPins[]={{"res_position","SV_Target0",4,false},
{"res_C","SV_Target1",4,false},
{"res_H","SV_Target2",4,false},
{"res_velocity","SV_Target3",4,false},
{"res_depth","SV_DepthGreaterEqual",1,false}};
static constexpr Wire wires[]={{-1,0,0,0},{-1,1,0,1},{-1,2,0,2},{-1,3,0,3},{-1,4,0,4},{-1,5,0,5},{-1,6,0,6},{-1,7,0,7},{-1,8,0,8},{-1,9,0,9},{-1,0,1,0},{-1,4,1,1},{-1,9,1,2},{0,10,1,3},{0,12,1,4},{0,13,1,5},{1,7,2,0},{1,6,2,1},{0,14,2,2},{1,7,3,0},{1,6,3,1},{0,14,3,2},{1,7,4,0},{1,6,4,1},{0,14,4,2},{-1,0,5,0},{-1,5,5,1},{-1,4,5,2},{-1,9,5,3},{0,13,5,4},{0,14,5,5},{0,10,5,6},{0,11,5,7},{0,12,5,8},{1,7,5,9},{2,3,5,10},{2,4,5,11},{2,5,5,12},{2,6,5,13},{2,7,5,14},{3,3,5,15},{3,4,5,16},{3,5,5,17},{3,6,5,18},{3,7,5,19},{4,3,5,20},{4,4,5,21},{4,5,5,22},{4,6,5,23},{4,7,5,24},{-1,0,6,0},{-1,1,6,1},{-1,2,6,2},{-1,3,6,3},{-1,4,6,4},{-1,5,6,5},{-1,6,6,6},{-1,7,6,7},{-1,8,6,8},{-1,9,6,9},{5,25,6,10},{5,26,6,11},{5,27,6,12},{5,28,6,13},{5,29,6,14},{0,15,6,15},{0,16,6,16},{0,17,6,17},{0,18,6,18},{5,30,6,19},{6,20,7,0},{6,21,7,1},{6,22,7,2},{6,23,7,3},{6,24,7,4}};
static constexpr Bound slots[]={{"smp_base",0u,1u,D3D_SIT_SAMPLER},
{"s_painter_removal_pages",0u,1u,D3D_SIT_TEXTURE},
{"s_painter_removal_data",1u,1u,D3D_SIT_TEXTURE},
{"s_painter_slits",2u,1u,D3D_SIT_TEXTURE},
{"s_anatomy_pages",3u,1u,D3D_SIT_TEXTURE},
{"s_anatomy_data",4u,1u,D3D_SIT_TEXTURE},
{"s_painter_blood_cells",5u,1u,D3D_SIT_TEXTURE},
{"s_tissue_1",6u,1u,D3D_SIT_TEXTURE},
{"s_tissue_2",7u,1u,D3D_SIT_TEXTURE},
{"s_tissue_3",8u,1u,D3D_SIT_TEXTURE},
{"s_tissue_4",9u,1u,D3D_SIT_TEXTURE},
{"s_tissue_5",10u,1u,D3D_SIT_TEXTURE},
{"s_tissue_6",11u,1u,D3D_SIT_TEXTURE},
{"s_tissue_7",12u,1u,D3D_SIT_TEXTURE},
{"s_tissue_8",13u,1u,D3D_SIT_TEXTURE},
{"s_tissue_9",14u,1u,D3D_SIT_TEXTURE},
{"s_tissue_10",15u,1u,D3D_SIT_TEXTURE},
{"s_tissue_11",16u,1u,D3D_SIT_TEXTURE},
{"s_tissue_12",17u,1u,D3D_SIT_TEXTURE},
{"s_tissue_13",18u,1u,D3D_SIT_TEXTURE},
{"s_tissue_14",19u,1u,D3D_SIT_TEXTURE},
{"s_tissue_15",20u,1u,D3D_SIT_TEXTURE},
{"s_tissue_bone_bump",21u,1u,D3D_SIT_TEXTURE},
{"s_tear_params",22u,1u,D3D_SIT_TEXTURE},
{"s_tear_map",23u,1u,D3D_SIT_TEXTURE},
{"s_tear_bake",24u,1u,D3D_SIT_TEXTURE},
{"s_painter_burn_field",25u,1u,D3D_SIT_TEXTURE},
{"s_painter_burn_faces",26u,1u,D3D_SIT_TEXTURE},
{"s_painter_burn_walls",27u,1u,D3D_SIT_TEXTURE},
{"s_painter_burn_baked",28u,1u,D3D_SIT_TEXTURE},
{"dynamic_transforms",1u,1u,D3D_SIT_CBUFFER},
{"static_globals",2u,1u,D3D_SIT_CBUFFER},
{"PainterTissueState",0u,1u,D3D_SIT_CBUFFER}};
static const char* const wrappers[]={R"BURN(export void findTorn(float3 painter_rest,float3 painter_L0,float3 painter_L1,float3 painter_L2,float2 tcdh,float4 position,float4 vel_curr,float4 vel_prev,float4 painter_hpos,uint primitive,out precise float3 initP,out precise float3 initN,out uint initL,out uint found,out precise float3 ray,out uint4 witness,out precise float3 cutpos,out float3 dpx,out float3 dpy){
painter_vmi_v2p I;I.painter_rest=painter_rest;I.painter_L0=painter_L0;I.painter_L1=painter_L1;I.painter_L2=painter_L2;I.tcdh=tcdh;I.position=position;I.vel_curr=vel_curr;I.vel_prev=vel_prev;I.painter_hpos=painter_hpos;ray = painter_vm_rest_ray ( I . painter_L0 , I . painter_L1 , I . painter_L2 , I . position . xyz ) ; 
    dpx=ddx(I.painter_rest);dpy=ddy(I.painter_rest);found=painter_vmi_find(I.painter_rest,ray,I.tcdh.x,initP,initN,initL);witness=uint4(painter_lod_has_crossing,painter_lod_wall_finish,painter_lod_detached_record,painter_lod_body_burst_face);cutpos=painter_lod_cut_position;
})BURN",
R"BURN(export void intent(float3 painter_rest,float2 tcdh,uint primitive,float3 initP,uint initL,uint found,out float3 from,out uint mode){
PbGrid grid=pb_grid();float3 rest=painter_rest;float boxU=tcdh.x;uint label=initL;from=found?initP:rest;mode=0u;[ branch ] if ( painter_burn_draw . z > .5 && painter_burn_meta . y > 0. ) { 
         
        bool receded ; 
        [ branch ] if ( found && grid . live ) receded = pb_existing_heat ( grid , from , label ) > 0. ; 
        else receded = pb_front_surface ( from ) . y > .5 ; 
        if ( found && grid . live && grid . legacy && ! pb_legacy_remains ( from , label ) && pb_old_front_surface ( from ) . y > .5 ) receded = true ; 
        if ( ! found && receded && pb_keep_skin ( rest , painter_burn_source_role ( primitive ) ) ) receded = false ; 
        if ( ! found && grid . live && receded && painter_burn_source_role ( primitive ) == 1u ) { 
            float3 normal ; receded = pb_breached ( rest , primitive , normal ) ; 
        } 
        if ( ! found && receded && grid . live && painter_burn_source_role ( primitive ) == 1u 
        && pb_local ( grid , rest ) . x <= pb_cut_start ( rest ) ) receded = false ; 
        if ( ( found || boxU >= ( - 500.0 ) ) && receded && ( ! found || ! pb_hard_label ( label ) ) ) { 
            mode=grid.live?(painter_burn_bake.x>.5?3u:2u):1u;}}
})BURN",
R"BURN(export void traceHistory(uint mode,float3 from,float3 ray,out precise float3 historyP,out precise float3 historyN,out uint historyL,out float historyHeat,out uint historyFound){
historyP=0.;historyN=0.;historyL=0u;historyHeat=0.;historyFound=0u;[branch]if(mode==1u){historyFound=pb_old_trace_recession(from,ray,historyP,historyN,historyL);historyHeat=historyFound?1.:0.;}
})BURN",
R"BURN(export void traceLive(uint mode,float3 from,float3 ray,out precise float3 liveP,out precise float3 liveN,out uint liveL,out float liveHeat,out uint liveFound){
liveP=0.;liveN=0.;liveL=0u;liveHeat=0.;liveFound=0u;[branch]if(mode==2u){liveFound=pb_trace_local(pb_grid(),from,ray,liveP,liveN,liveL,liveHeat);}
})BURN",
R"BURN(export void traceBaked(uint mode,float3 from,float3 ray,out precise float3 bakedP,out precise float3 bakedN,out uint bakedL,out float bakedHeat,out uint bakedFound){
bakedP=0.;bakedN=0.;bakedL=0u;bakedHeat=0.;bakedFound=0u;[branch]if(mode==3u){bakedFound=pb_trace_baked(pb_grid(),from,ray,bakedP,bakedN,bakedL,bakedHeat);}
})BURN",
R"BURN(bool chooseHit(float3 painter_rest, float4 position, float2 tcdh, uint primitive, uint found, float3 ray, float3 initP, float3 initN, uint initL, uint mode, float3 historyP, float3 historyN, uint historyL, float historyHeat, uint historyFound, float3 liveP, float3 liveN, uint liveL, float liveHeat, uint liveFound, float3 bakedP, float3 bakedN, uint bakedL, float bakedHeat, uint bakedFound, out float3 P, out float3 N, out uint label, out float fireFloor, out float layerAge){PbGrid grid=pb_grid();float3 rest=painter_rest;float boxU=tcdh.x;float3 viewP=position.xyz;P=initP;N=initN;label=initL;fireFloor=0.;layerAge=-1.;if(mode!=0u){bool survived=false;if(mode==1u){P=historyP;N=historyN;label=historyL;fireFloor=historyHeat;survived=historyFound;}if(mode==2u){P=liveP;N=liveN;label=liveL;fireFloor=liveHeat;survived=liveFound;}if(mode==3u){P=bakedP;N=bakedN;label=bakedL;fireFloor=bakedHeat;survived=bakedFound;}if ( survived && grid . live && ! pb_hard_label ( label ) && fireFloor > 0. ) { 
                float2 local = pb_local ( grid , P ) ; 
                layerAge = painter_burn_source_role ( primitive ) == 2u ? local . x : pb_underlayer_age ( P , local . x ) ; 
            } 
            return survived;}if ( found ) { 
        if ( grid . live && pb_hard_label ( label ) ) { 
            float2 sample = pb_local ( grid , P ) ; 
            float since = pb_existing_heat ( grid , P , 2u ) ; 
            if ( since > 0. ) fireFloor = 1. + since ; 
        } 
        return true ; 
    } 
    if ( ! pb_first_under_cloth ( rest , ray , viewP , primitive , boxU , P , N , label ) ) return false ; 

    fireFloor = smoothstep ( .67 , .97 , pb_front_surface ( rest ) . x ) ; 
    if ( grid . live && ! pb_hard_label ( label ) ) { 
        float3 dr = cross ( ddx ( rest ) , ddy ( rest ) ) ; dr *= rsqrt ( max ( dot ( dr , dr ) , 1.e-20 ) ) ; 
        layerAge = pb_local_underlayer ( rest , dr , painter_burn_source_role ( primitive ) , pb_local ( grid , rest ) . x ) ; 
    } 
    return true ; }
export void finishHit(float3 painter_rest,float4 position,float2 tcdh,uint primitive,uint found,float3 ray,float3 initP,float3 initN,uint initL,uint mode,float3 historyP,float3 historyN,uint historyL,float historyHeat,uint historyFound,float3 liveP,float3 liveN,uint liveL,float liveHeat,uint liveFound,float3 bakedP,float3 bakedN,uint bakedL,float bakedHeat,uint bakedFound,out float3 P,out float3 N,out uint label,out float fireFloor,out float layerAge,out uint visible){
visible=chooseHit(painter_rest,position,tcdh,primitive,found,ray,initP,initN,initL,mode,historyP,historyN,historyL,historyHeat,historyFound,liveP,liveN,liveL,liveHeat,liveFound,bakedP,bakedN,bakedL,bakedHeat,bakedFound,P,N,label,fireFloor,layerAge);
})BURN",
R"BURN(export void materialTorn(float3 painter_rest,float3 painter_L0,float3 painter_L1,float3 painter_L2,float2 tcdh,float4 position,float4 vel_curr,float4 vel_prev,float4 painter_hpos,uint primitive,float3 P,float3 N,uint label,float fireFloor,float layerAge,uint4 witness,float3 cutpos,float3 dpx,float3 dpy,uint visible,out float4 res_position,out float4 res_C,out float4 res_H,out float4 res_velocity,out float res_depth){
painter_vmi_v2p I;I.painter_rest=painter_rest;I.painter_L0=painter_L0;I.painter_L1=painter_L1;I.painter_L2=painter_L2;I.tcdh=tcdh;I.position=position;I.vel_curr=vel_curr;I.vel_prev=vel_prev;I.painter_hpos=painter_hpos;painter_lod_has_crossing=witness[0]!=0u;painter_lod_wall_finish=witness[1]!=0u;painter_lod_detached_record=witness[2]!=0u;painter_lod_body_burst_face=witness[3]!=0u;painter_lod_cut_position=cutpos;painter_vm_out o=(painter_vm_out)0;[branch] if(!visible){res_position=0.;res_C=0.;res_H=0.;res_velocity=0.;res_depth=I.painter_hpos.z;discard;return;}const float3 d = P - I . painter_rest ; 
    const float3 Pe = I . position . xyz + float3 ( dot ( I . painter_L0 , d ) , dot ( I . painter_L1 , d ) , dot ( I . painter_L2 , d ) ) ; 

    float3 skinPlane = cross ( dpx , dpy ) ; 
    float skinArea = dot ( skinPlane , skinPlane ) ; 
    float inset = ( I . tcdh . x >= ( - 500.0 ) && skinArea > 1e-18 ) 
    ? abs ( dot ( d , skinPlane ) ) * rsqrt ( max ( 1e-18 , skinArea ) ) : 0.0 ; 
    o . g = painter_vm_shade ( P , N , label , I . painter_L0 , I . painter_L1 , I . painter_L2 , Pe , I . position . w , dpx , dpy , inset ) ;precise float4 strictColor=o.g.C;o.g.C=strictColor; 
    if ( layerAge >= 0. && ! pb_hard_label ( label ) ) 
    { 

        PbGrid colourGrid = pb_grid ( ) ; 
        [ branch ] if ( colourGrid . live && layerAge > 0. && layerAge < 24. ) 
        { 
            float rawAge = pb_local ( colourGrid , P ) . x ; 
            layerAge = max ( 0. , layerAge + pb_optical_age ( colourGrid , P , rawAge ) - rawAge ) ; 
        } 
        pb_revealed_layer ( o . g , P , N , Pe , float3x3 ( I . painter_L0 , I . painter_L1 , I . painter_L2 ) , label , layerAge , fireFloor ) ; 
    } 
    else if ( fireFloor > 0. && ! pb_hard_label ( label ) ) 
    pb_material_gbuffer ( o . g , P , N , Pe , 4u , float2 ( fireFloor * .98 , 0. ) ) ; 

    if ( fireFloor > 0. ) { 
        PbGrid thermalGrid = pb_grid ( ) ; 
        if ( thermalGrid . live ) { 
            float dose = saturate ( pb_local ( thermalGrid , P ) . x / 7. ) ; 
            float heat = pb_local_heat ( thermalGrid , dose , 160u ) ; 
            if ( heat > 0. ) { o . g . H . r = max ( o . g . H . r , heat ) ; o . g . H . a = max ( o . g . H . a , 1. ) ; } 
        } 
    } 
    bool exposedBurn = ( fireFloor > 0. || layerAge >= 0. ) ; 
    uint wallToken = painter_burn_wall_token ( P , N , label ) ; 
    if ( wallToken != 0u && label != 0u ) { 
        uint domain = ( label == 16u ) ? 3u : 2u ; 
        float2 q = painter_burn_surface_sample ( P , N , domain , wallToken ) ; 
        q . x = pb_sparse_optical ( P , N , domain , wallToken , q . x ) ; 
        pb_material_gbuffer ( o . g , P , N , Pe , domain == 3u ? 5u : 4u , q ) ; 
        exposedBurn = exposedBurn || ( domain == 2u && q . x >= .96 ) ; 
    } 
    if ( fireFloor > 0. && pb_hard_label ( label ) ) 
    o . g . C = pb_fired_mineral ( P , label , max ( 0. , fireFloor - 1. ) ) ; 
    if ( exposedBurn ) { 

        
        float receiverMaterial = float ( L_material . w ) ; 
        
        o . g . position . w = gbuf_pack_hemi_mtl ( saturate ( I . position . w ) , receiverMaterial ) ; 
    } 
    o . g . velocity = ssfx_mv_calc ( I . vel_curr , I . vel_prev , 0 , 0 ) ; 
    const float z0 = max ( I . position . z , 1e-4 ) , z1 = max ( Pe . z , z0 ) ; 
    o . depth = saturate ( max ( I . painter_hpos . z , 1.0 - ( 1.0 - I . painter_hpos . z ) * z0 / z1 ) ) ; 
    res_position=o.g.position;res_C=o.g.C;res_H=o.g.H;res_velocity=o.g.velocity;res_depth=o.depth;
})BURN"};
static const char* const bakedScan=R"BURN(bool pb_trace_baked ( PbGrid g , float3 start , float3 ray , out float3 P , out float3 N , out uint label , out float intensity ) { 
    P = start ; N = - ray ; label = 0 ; intensity = 0. ; 
    painter_vm_ctx c = painter_vm_context ( ) ; 
    float3 org , cell ; pb_bake_box ( g , org , cell ) ; 
    const float cmin = min ( cell . x , min ( cell . y , cell . z ) ) , cmax = max ( cell . x , max ( cell . y , cell . z ) ) ; 
    const float stepLen = .7 * cmin , fine = .3 * cmin ; 
    const float3 invAbsRay = rcp ( max ( abs ( ray ) , 1.e-9 ) ) ; 
    const float3 nextFace = step ( 0. , ray ) ; 
    bool cached = false ; PbShaft sa = ( PbShaft ) 0 , sb = ( PbShaft ) 0 ; 
    float t = 0. , lastTaken = - 1. ; 
    uint searches = 0u ; 
    uint outer = 0u , fineStep = 0u , bisectStep = 0u ; 
    
    uint phase = 0u , keptMaterial = 0u ; 
    float nearT = 0. , removedT = - 1. , low = 0. , high = 0. ; 
    [ loop ] while ( outer < 160u ) { 
        const float sampleT = phase == 0u ? t : ( phase == 1u ? nearT : .5 * ( low + high ) ) ; 
        float3 p = start + ray * sampleT ; 
        [ branch ] if ( phase == 0u ) { 
            int3 b = int3 ( floor ( ( p - org ) / cell ) ) ; 
            if ( any ( b < 0 ) || any ( b >= int3 ( uint3 ( 128u , 256u , 64u ) ) ) ) break ; 
            if ( s_painter_burn_baked . Load ( int4 ( b , 0 ) ) == 1u ) { 
                float3 edge = org + ( float3 ( b ) + nextFace ) * cell ; 
                float3 edgeT = abs ( edge - p ) * invAbsRay ; 
                float toEdge = min ( edgeT . x , min ( edgeT . y , edgeT . z ) ) ; 
                uint run = min ( 160u - outer , max ( 1u , ( uint ) ceil ( max ( 0. , toEdge / stepLen - 1.e-4 ) ) ) ) ; 
                lastTaken = t + float ( run - 1u ) * stepLen ; 
                t += float ( run ) * stepLen ; 
                outer += run ; 
                continue ; 
            } 
        } 
        uint mat = keptMaterial ; 
        bool owned = false ; 
        [ branch ] if ( phase != 2u ) { 
            mat = painter_vm_anatomy ( c , p ) ; 
            if ( mat != 0u && mat != 16u && ( mat & 64u ) == 0u ) owned = pb_owned ( c , p , mat ) ; 
        } 
        float heat = 0. ; 
        [ branch ] if ( phase == 2u || ( phase == 1u && owned && ! pb_hard_label ( mat ) ) ) 
        heat = pb_existing_heat_cached ( g , p , mat , cached , sa , sb ) ; 
        [ branch ] if ( phase == 0u ) { 
            if ( ! owned ) { t += stepLen ; ++ outer ; continue ; } 
            if ( searches < 2u ) { 
                ++ searches ; 
                nearT = max ( 0. , ( lastTaken >= 0. ? lastTaken : t ) - cmax ) ; 
                removedT = - 1. ; 
                fineStep = 0u ; 
                phase = 1u ; 
                continue ; 
            } 
        } else if ( phase == 1u ) { 
            if ( ! owned || ( ! pb_hard_label ( mat ) && heat > 0. ) ) { 
                if ( owned ) removedT = nearT ; 
                nearT += fine ; 
                if ( ++ fineStep == 16u ) { 
                    if ( removedT >= 0. ) lastTaken = removedT ; 
                    t = max ( t + stepLen , nearT ) ; 
                    ++ outer ; 
                    phase = 0u ; 
                } 
                continue ; 
            } 
            keptMaterial = mat ; 
            if ( removedT >= 0. && ! pb_hard_label ( mat ) ) { 
                low = removedT ; high = nearT ; 
                bisectStep = 0u ; 
                phase = 2u ; 
                continue ; 
            } 
            t = nearT ; 
        } else { 
            if ( heat > 0. ) low = sampleT ; else high = sampleT ; 
            if ( ++ bisectStep < 4u ) continue ; 
            t = high ; 
            p = start + ray * t ; 
        } 
        
        P = p ; label = mat & 63u ; 
        float2 s = pb_local ( g , p ) ; 
        float3 grad ; 
        if ( pb_hard_label ( mat ) ) { 
            float3 d = float3 ( 1. / c . aScale , 0. , 0. ) ; 
            grad = float3 ( float ( pb_bone_at ( c , p - d . xyy ) ) - float ( pb_bone_at ( c , p + d . xyy ) ) , 
            float ( pb_bone_at ( c , p - d . yxy ) ) - float ( pb_bone_at ( c , p + d . yxy ) ) , 
            float ( pb_bone_at ( c , p - d . yyx ) ) - float ( pb_bone_at ( c , p + d . yyx ) ) ) ; 
            intensity = 1. + max ( 0. , pb_post_char ( s , p , mat ) ) ; 
        } else { 
            
            float3 q = ( p - g . origin ) * g . inv ; 
            int3 tc = clamp ( int3 ( floor ( q ) ) , 0 , int3 ( 14 , 30 , 6 ) ) ; 
            PbTile tile = pb_tile ( g , uint3 ( tc ) ) ; 
            float3 f = saturate ( q - float3 ( tc ) ) ; 
            grad = .009 * pb_weight_gradient ( tile . lowAge , tile . highAge , f , g . inv ) - pb_weight_gradient ( tile . lowDepth , tile . highDepth , f , g . inv ) 
            - pb_resistance_normal ( p ) ; 
            intensity = saturate ( ( s . x - 4. - s . y / .009 ) / 3. ) ; 
        } 
        N = dot ( grad , grad ) > 1.e-9 ? normalize ( grad ) : - ray ; 
        if ( dot ( N , ray ) > 0. ) N = - N ; 
        return true ; 
    } 
    return false ; 
})BURN";
static const painter_portable_shader::Digest expectedBytes={0xd6,0xfa,0xe7,0x5f,0xed,0x87,0x59,0x16,0xee,0xf2,0xfd,0x6d,0x2e,0xb9,0x7c,0x22,0x5e,0x00,0x3d,0x22,0x94,0x4a,0x33,0xb8,0x7d,0xd8,0xfe,0xc6,0x46,0xa8,0x0f,0xed};
