#pragma once
#ifdef USE_DX11
#include "PainterBloodTextureWorker.h"
#include <memory>
#include <cstring>
#include <vector>
namespace PainterStaticBlood {
class DecoderCache {
    using Snapshot=PainterBloodTextureSource::SnapshotPtr;
    using Image=PainterBloodTextureCodec::Image;
    struct Entry { Snapshot source; std::shared_ptr<const Image> image; };
    std::unique_ptr<PainterBloodTextureWorker::Decoder> decoder;
    std::vector<Entry> entries;
    std::size_t bytes=0;
public:
    static constexpr std::size_t Limit=64u*1024u*1024u;
    // Own this object outside thread-local storage. Join the worker, then destroy the cache.
    std::shared_ptr<const Image> get(const Snapshot& s){
        if(!s||!s->supported||!s->source.p||!s->device.p)return {};
        for(const auto& e:entries)if(e.source->source.p==s->source.p&&e.source->device.p==s->device.p&&
             !std::memcmp(&e.source->texture,&s->texture,sizeof(s->texture))&&
             !std::memcmp(&e.source->view,&s->view,sizeof(s->view)))return e.image;
        const unsigned mip=s->view.Texture2D.MostDetailedMip;if(mip>=s->texture.MipLevels||mip>=32)return {};
        const std::size_t w=std::max(1u,s->texture.Width>>mip),h=std::max(1u,s->texture.Height>>mip);
        if(!w||w>Limit/sizeof(PainterBloodTextureCodec::Pixel)/h||s->source.p->byteCount>Limit)return {};
        const std::size_t wanted=w*h*sizeof(PainterBloodTextureCodec::Pixel);
        for(auto it=entries.begin();(bytes>Limit-wanted||entries.size()>=32)&&it!=entries.end();){
            if(it->image.use_count()==1){bytes-=it->image->pixels.size()*sizeof(PainterBloodTextureCodec::Pixel);it=entries.erase(it);}else ++it;
        }
        if(bytes>Limit-wanted||entries.size()>=32)return {};
        struct Reader {IReader* raw=nullptr;~Reader(){if(raw)FS.r_close(raw);}} in;
        in.raw=FS.r_open(s->source.p->path.c_str());
        if(!in.raw||in.raw->length()!=s->source.p->byteCount)return {};
        const auto* begin=static_cast<const unsigned char*>(in.raw->pointer());
        std::vector<unsigned char> original(begin,begin+in.raw->length());
        FS.r_close(in.raw);in.raw=nullptr;
        if(!decoder||!decoder->matches(s->device.p))decoder.reset(new PainterBloodTextureWorker::Decoder(s->device.p,PainterBloodTextureWorker::Channel::World));
        auto image=std::make_shared<Image>(decoder->decode(*s,original));
        if(image->pixels.size()!=w*h)return {};
        entries.push_back({s,image});bytes+=wanted;
        return image;
    }
    std::size_t resident()const{return bytes;}
};
}
#endif
