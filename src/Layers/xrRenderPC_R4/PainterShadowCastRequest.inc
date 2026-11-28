// Checked light-depth programs use their own cache records.
    if (FAILED(_result) && useGeneratedShaderCache && m_ShaderOptions.empty() &&
        
        !strcmp(name,"painter_burn_painter_vmi_shadow_tear") && !strcmp(pTarget,"ps_5_0") &&
        !strcmp(pFunctionName,"main") && Flags==D3DCOMPILE_PACK_MATRIX_ROW_MAJOR) {
        includer inputs;
        painter_portable_shader::Digest signature{};
        const unsigned samples=o.dx10_msaa ? unsigned(o.dx10_msaa_samples) : 1u;
        const char* certified=nullptr;
        if(painter_portable_shader::key(name,pFunctionName,pTarget,Flags,pSrcData,SrcDataLen,defines,inputs,signature) &&
           (certified=painter_shadow_cast::certifiedProfile(samples,painter_portable_shader::hex(signature)))!=nullptr) {
            const std::string usual(file_name);
            const std::string side=usual.substr(0,usual.find_last_of("/\\")+1)+"sc1_depth_"+std::to_string(samples)+"_"+std::string(certified,24);
            if(side.size()<sizeof(string_path)){
                bool rejectedByDevice=false;
                if(IReader* saved=FS.r_open(side.c_str())){
                    if(saved->length()>=40 && saved->length()<=1048576 && saved->r_u32()==source_crc){
                        const auto recorded=saved->r_u32();
                        if(crc32(saved->pointer(),saved->elapsed())==recorded &&
                           painter_burn_tear_link::certifiedPayload(saved->pointer(),saved->elapsed(),certified)){
                            _result=painter_compile_load::active ? painter_compile_load::checkedCache(saved->pointer(),saved->elapsed()) :
                                createForRequest(pTarget,(DWORD*)saved->pointer(),saved->elapsed(),side.c_str(),result,o.disasm);
                            rejectedByDevice=FAILED(_result);
                        }
                    }
                    FS.r_close(saved);
                    if(SUCCEEDED(_result))return _result;
                }
                if(!rejectedByDevice){
                    if(painter_compile_load::active){
                        auto& batch=*painter_compile_load::active;
                        if(batch.paths.count(side)){++batch.duplicate;return S_OK;}
                        auto recipe=painter_shadow_cast::capture(batch.tornLibraries,name,pFunctionName,pTarget,Flags,pSrcData,SrcDataLen,defines,&inputs);
                        recipe.certified=certified;
                        if(recipe.ready()){
                            const auto before=batch.jobs.size();
                            if(batch.capture(side.c_str(),name,source_crc,pSrcData,SrcDataLen,defines,pFunctionName,pTarget,Flags,inputs)){
                                if(batch.jobs.size()>before){
                                    auto& job=batch.jobs.back();job.burnedShadow=std::move(recipe);job.ordinaryPath=usual;
                                    for(const auto& part:job.burnedShadow.parts)batch.tornParts.emplace_back(part,Flags);
                                }
                                return S_OK;
                            }
                        }
                    }else{
                        painter_tear_modules::Cache modules;
                        auto recipe=painter_shadow_cast::capture(modules,name,pFunctionName,pTarget,Flags,pSrcData,SrcDataLen,defines,&inputs);
                        recipe.certified=certified;
                        Microsoft::WRL::ComPtr<ID3DBlob> compiled;
                        if(recipe.ready() && painter_shadow_cast::make(recipe,Flags,compiled.GetAddressOf())){
                            const auto* data=compiled->GetBufferPointer();const auto size=(u32)compiled->GetBufferSize();
                            _result=createForRequest(pTarget,(DWORD*)data,size,side.c_str(),result,o.disasm);
                            if(SUCCEEDED(_result)){
                                if(IWriter* saved=FS.w_open(side.c_str())){
                                    if(saved->valid()){saved->w_u32(source_crc);saved->w_u32(crc32(data,size));saved->w(data,size);}
                                    FS.w_close(saved);
                                }
                                return _result;
                            }
                        }
                    }
                }
            }
        }
    }
