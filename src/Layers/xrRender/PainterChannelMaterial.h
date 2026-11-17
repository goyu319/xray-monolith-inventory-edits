#pragma once
#ifdef USE_DX11
#include "ResourceManager.h"
#include "PainterSoftShader.h"
// Renderer-owner-thread adapter for the EXISTING anatomy material. It keeps
// every texture, state and constant handler, replaces only admitted programs,
// and adds immutable field bindings. Publication must still use the existing
// staged packet barrier; a false result requires same-candidate fallback.
namespace PainterChannelMaterial {
struct Bindings {
    ref_texture anatomy_pages, anatomy_data, cut_words, cut_profiles, primitive_owners;
    // The packet's removal field: on a separated piece it marks the other pieces' tissue, and the channel shader
    // drops wall pixels backed by it (painter_channel_tissue). Required when the program reflects it.
    ref_texture removal_pages, removal_data;
    bool ready() const { return anatomy_pages && anatomy_data && cut_words && cut_profiles && primitive_owners; }
    const ref_texture* find(LPCSTR name) const {
        if(!xr_strcmp(name,"s_anatomy_pages"))return &anatomy_pages;
        if(!xr_strcmp(name,"s_anatomy_data"))return &anatomy_data;
        if(!xr_strcmp(name,"s_painter_cut_words"))return &cut_words;
        if(!xr_strcmp(name,"s_painter_cut_profiles"))return &cut_profiles;
        if(!xr_strcmp(name,"s_painter_channel_owners"))return &primitive_owners;
        if(!xr_strcmp(name,"s_painter_removal_pages"))return &removal_pages;
        if(!xr_strcmp(name,"s_painter_removal_data"))return &removal_data;
        return nullptr;
    }
};
struct Programs {xr_string vs,ps;int sample=-1;};
inline bool programs(const SPass& p,Programs& result) {
    if(!p.vs||!p.ps||!p.vs->vs||!p.ps->ps||p.vs->skinning!=4)return false;
    if((p.gs&&p.gs->gs)||(p.hs&&p.hs->sh)||(p.ds&&p.ds->sh)||(p.cs&&p.cs->sh))return false;
    xr_string vs=*p.vs->cName,ps=*p.ps->cName;
    if(vs.size()<2||vs.substr(vs.size()-2)!="_4")return false;
    vs.resize(vs.size()-2);
    if(ps.size()>2&&ps[ps.size()-2]=='_'&&ps.back()>='0'&&ps.back()<='7'){
        result.sample=ps.back()-'0';ps.resize(ps.size()-2);
    }
    if(vs=="painter_anatomy"&&ps=="painter_anatomy"){
        result.vs="painter_anatomy";result.ps="painter_channel";return true;
    }
    if(vs=="shadow_direct_model"&&ps=="dumb"){
        result.vs="painter_channel_shadow";result.ps="painter_channel_shadow";return true;
    }
    return false;
}
inline bool clone(CResourceManager& resources,const ref_shader& source,const Bindings& fields,
    ref_shader& destination,xr_string& reason) {
    if(!source||!fields.ready()){reason="channel_missing_field";return false;}
    struct Restore {
        int skin=Engine.External.GetSkinningMode(),sample=::Render->m_MSAASample;
        ~Restore(){Engine.External.SetSkinningMode(skin);::Render->m_MSAASample=sample;}
    } restore;
    Shader assembled;unsigned passes=0;
    for(unsigned n=0;n<SHADER_ELEMENTS_MAX;++n){
        const auto&oldElement=source->E[n];if(!oldElement)continue;
        if(oldElement->flags.bStrictB2F){reason="channel_forward_material";return false;}
        ShaderElement element;element._copy(*oldElement);element.passes.clear();
        for(const auto&old:oldElement->passes){
            Programs names;if(!old||!programs(*old,names)){reason="channel_unsupported_pass";return false;}
            string_path path;
            strconcat(sizeof(path),path,"r3\\",names.vs.c_str(),".vs");if(!FS.exist("$game_shaders$",path)){reason="channel_missing_vs";return false;}
            strconcat(sizeof(path),path,"r3\\",names.ps.c_str(),".ps");if(!FS.exist("$game_shaders$",path)){reason="channel_missing_ps";return false;}
            Engine.External.SetSkinningMode(4);::Render->m_MSAASample=names.sample;
            SPass pass;pass.state=old->state;pass.C=old->C;pass.gs=old->gs;pass.hs=old->hs;pass.ds=old->ds;pass.cs=old->cs;
            {PainterSoftShaderGuard soft;pass.vs=resources._CreateVS(names.vs.c_str());pass.ps=resources._CreatePS(names.ps.c_str());}
            if(!pass.vs||!pass.ps||!pass.vs->vs||!pass.ps->ps){reason="channel_program_creation";return false;}
            R_constant_table constants;constants.merge(&pass.ps->constants);constants.merge(&pass.vs->constants);
            STextureList textures;unsigned attached=0;
            for(auto&c:constants.table){
                if(const auto* binding=fields.find(*c->name)){
                    if(c->type!=RC_dx10texture){reason="channel_resource_type";return false;}
                    if(!*binding){reason="channel_missing_removal";return false;}
                    textures.emplace_back(c->samp.index,*binding);++attached;continue;
                }
                auto*previous=old->constants?old->constants->get(*c->name):nullptr;
                if(!previous||previous->type!=c->type){reason="channel_constant_contract";return false;}
                c->handler=previous->handler;
                if(c->type==RC_sampler&&c->samp.index!=previous->samp.index){reason="channel_sampler_contract";return false;}
                if(c->type==RC_dx10texture){
                    bool found=false;if(old->T)for(const auto&t:*old->T)if(t.first==previous->samp.index){textures.emplace_back(c->samp.index,t.second);found=true;break;}
                    if(!found){reason="channel_original_texture_missing";return false;}
                }
            }
            // Both deferred and shadow programs require all five resources.
            if(attached!=5&&attached!=7){reason="channel_incomplete_reflection";return false;}
            pass.constants=resources._CreateConstantTable(constants);pass.T=resources._CreateTextureList(textures);
            element.passes.push_back(resources._CreatePass(pass));++passes;
        }
        assembled.E[n]=resources._CreateElement(element);
    }
    if(!passes){reason="channel_no_passes";return false;}
    ref_shader completed;completed=resources._CreateShader(&assembled);destination=completed;reason.clear();return true;
}
}
#endif
