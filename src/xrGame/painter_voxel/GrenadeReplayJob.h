// Private visual replay consumes accepted stamps in order. Only the damage worker
// merges its result, after the stock physics and separation masks are final.
struct DisplayStroke {
 Vec origin,dir;double radius{},reach{},exitMult{},cavitation{},burst{},variation{};
 std::uint64_t seed{};GrenadeBite bite{};
};

struct GrenadeReplay {
 Grid grid;
 const Asset& asset;
 std::vector<GrenadeStockCut> stock;
 std::vector<ImpactResult> hits;
 GrenadeReplay(const Grid& previous,const Asset& source):grid(core::overlayOf(previous)),asset(source) {
  if(previous.visibleFloor)for(const auto& p:*previous.visibleFloor){
   auto& tile=grid.writeTile(p.first);
   for(unsigned j=0;j<8;++j)tile.removed[j]&=~p.second.removed[j];
  }
 }
 void stamp(DisplayStroke s){
  const std::size_t j=hits.size();hits.emplace_back();
  const Vec axis=unit(s.dir);
  const auto surface=firstSolidCylinder(grid,s.origin,axis,s.radius);
  if(!surface.found)return;
  const auto intact=firstSolidCylinder(asset.base,s.origin,axis,s.radius);
  s.bite.inlet=std::max(0.,dot(surface.entry-s.origin,axis));
  s.bite.depthLimit=intact.found?std::max(0.,s.reach+dot(intact.entry-surface.entry,axis)):1.;
  Tunnel cut{};s.bite.capture=&cut;
  std::shared_ptr<GrenadeStockCut::Bits> raster;
  const auto visual=eraseTunnel(grid,s.origin,s.dir,s.radius,s.reach,nullptr,&asset.envelope_tree,
   s.exitMult,nullptr,s.seed,s.cavitation,s.burst,nullptr,nullptr,nullptr,-1,&surface,s.variation,nullptr,&s.bite,false,&raster);
  if(visual.freshSolid&&cut.radius>0){hits[j].removed=true;stock.push_back({cut,s.bite.depthLimit,j,std::move(raster)});}
 }
 void finish(){
  resolveGrenadeSupport(grid,asset.closure_field.get(),stock,hits);
 }
};

// At most one streaming replay uses a spare thread in the existing region pool.
// Pool workers do their inner work inline and never wait for another pool task.
// Other simultaneous explosions take the synchronous path on their damage worker.
class GrenadeReplayJob {
 struct State {
  std::mutex lock;
  std::condition_variable signal;
  std::vector<DisplayStroke> strokes;
  bool closed{},abandon{},done{};
  std::exception_ptr failure;
  std::unique_ptr<GrenadeReplay> replay;
  std::shared_ptr<const Asset> asset;
  std::shared_ptr<const Grid> previous;
  void run() noexcept {
   AllocationScope scope;
   // This is a coordinator in one of the pool's two spare slots. Only one
   // can exist. The remaining pool threads never wait for other helpers.
   struct Coordinator {
    bool old=core::painterInlineParallel;
    Coordinator(){core::painterInlineParallel=false;}
    ~Coordinator(){core::painterInlineParallel=old;}
   } coordinator;
   try{
    replay=std::make_unique<GrenadeReplay>(*previous,*asset);
    std::size_t i=0;
    for(;;){
     DisplayStroke stroke;
     {
      std::unique_lock<std::mutex> guard(lock);
      signal.wait(guard,[&]{return abandon||closed||i<strokes.size();});
      if(abandon)break;
      if(i==strokes.size()){guard.unlock();replay->finish();break;}
      stroke=strokes[i++];
     }
     replay->stamp(stroke);
    }
   }catch(...){failure=std::current_exception();}
   {std::lock_guard<std::mutex> guard(lock);done=true;}
   active().store(false,std::memory_order_release);
   signal.notify_all();
  }
 };
 static std::atomic<bool>& active(){static std::atomic<bool> activeJob{false};return activeJob;}
 std::shared_ptr<State> state;
 bool attempted{};
public:
 void append(const DisplayStroke& s,const std::shared_ptr<const Asset>& asset,const std::shared_ptr<const Grid>& previous,std::size_t limit){
  if(!attempted){
   attempted=true;
   if(core::painterInlineParallel||core::worker_quit::quitting()||core::regionThreadCount()<2)return;
   bool expected=false;
   if(!active().compare_exchange_strong(expected,true,std::memory_order_acq_rel))return;
   try{
    auto own=std::make_shared<State>();own->strokes.reserve(limit);
    own->asset=asset;own->previous=previous;own->strokes.push_back(s);
    core::workerPool().submit([own]{own->run();});state=std::move(own);
   }catch(...){active().store(false,std::memory_order_release);}
   return;
  }
  if(state){
   {std::lock_guard<std::mutex> guard(state->lock);state->strokes.push_back(s);}
   state->signal.notify_one();
  }
 }
 void close(){
  if(state){
   {std::lock_guard<std::mutex> guard(state->lock);state->closed=true;}
   state->signal.notify_one();
  }
 }
 std::unique_ptr<GrenadeReplay> result(const Grid& previous,const Asset& asset,const std::vector<DisplayStroke>& strokes){
  if(!state){
   auto result=std::make_unique<GrenadeReplay>(previous,asset);
   for(const auto& s:strokes)result->stamp(s);
   result->finish();return result;
  }
  std::unique_lock<std::mutex> guard(state->lock);
  state->signal.wait(guard,[&]{return state->done;});
  if(state->failure)std::rethrow_exception(state->failure);
  return std::move(state->replay);
 }
 ~GrenadeReplayJob(){
  if(state){
   {std::lock_guard<std::mutex> guard(state->lock);state->abandon=true;}
   state->signal.notify_one();
  }
 }
};
