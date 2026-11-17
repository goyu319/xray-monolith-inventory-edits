#include "EngineRouteProof.h"
#include <algorithm>
#include <atomic>
#include <deque>
#include <map>
#include <set>
#include <cmath>
#include <mutex>
#include <tuple>
#include <limits>
namespace painter { namespace combat { namespace engine {
using namespace core;
namespace {
auto route_key(const RouteProofSource&q){return std::make_tuple(q.epoch,q.map,q.geometry,q.actor.id,q.actor.generation,q.owner,q.restrictions,q.origin_node,q.goal_node,q.goal.x,q.goal.y,q.goal.z);}
using Key=decltype(route_key(RouteProofSource{}));
bool policy_ok(const RouteProofPolicy&p){return p.queue_bytes&&p.job_overhead_bytes&&p.worker_work&&p.authority_work&&p.expansion_limit&&p.expansion_limit<=65536&&p.output_limit&&p.output_limit<=65536&&p.source_age_ms&&std::isfinite(p.endpoint_height_m)&&p.endpoint_height_m>0;}
bool same_static(const StaticRouteRequest&s,const RouteProofSource&q){return s.actor==q.actor&&s.level_epoch==q.epoch&&s.snapshot_seq==q.sequence&&s.geometry_revision==q.geometry&&s.ownership_generation==q.owner&&s.restrictions_revision==q.restrictions&&s.captured_at_ms==q.at&&s.origin_node==q.origin_node&&s.goal_node==q.goal_node;}
RouteProofStatus unfinished(StaticRouteStatus status){switch(status){case StaticRouteStatus::exhausted:return RouteProofStatus::budget;case StaticRouteStatus::cancelled:return RouteProofStatus::cancelled;case StaticRouteStatus::invalid:case StaticRouteStatus::failed:return RouteProofStatus::failed;default:return RouteProofStatus::pending;}}
}
struct EngineRouteProof::Impl {
 struct Validation {RouteProofSource source;RouteProofStatus status=RouteProofStatus::unknown;};
 // Keep exact authority captures for workers still processing those snapshots.
 // The static node corridor is shared; only validation stamps are retained.
 static constexpr std::size_t validation_charge=sizeof(Validation)*2+sizeof(void*)*4;
 struct Job {RouteProofSource requested;StaticRouteResult route;RouteProofReceipt receipt;std::deque<Validation> history;std::atomic<bool> ready{false};std::size_t charged=0,cursor=0;};
 RouteProofPolicy policy;Search search;std::mutex mutex;std::map<Key,std::shared_ptr<Job>> jobs;std::deque<Key> round;
 std::size_t bytes=0;
 Impl(RouteProofPolicy p,Search s):policy(p),search(s){}
};
EngineRouteProof::EngineRouteProof(RouteProofPolicy p,Search s):impl_(new Impl(p,s)){}
EngineRouteProof::~EngineRouteProof()=default;
RouteProofService EngineRouteProof::service(){RouteProofService out;out.lifetime=shared_from_this();out.context=this;out.work_per_update=impl_->policy.worker_work;
 out.query=[](void*context,const RouteProofSource&q,RouteProofBudget&budget,const std::function<bool()>&stop){return static_cast<EngineRouteProof*>(context)->query(q,budget,stop);};return out;}
RouteProofReceipt EngineRouteProof::query(const RouteProofSource&q,RouteProofBudget&budget,const std::function<bool()>&stop){
 RouteProofReceipt out;out.queried=q;auto& s=*impl_;if(!policy_ok(s.policy)||!s.search)return out;
 if(stop&&stop()){out.status=RouteProofStatus::cancelled;return out;}
 if(!budget.work_left){out.status=RouteProofStatus::budget;return out;}--budget.work_left;++budget.work_used;
 std::shared_ptr<Impl::Job> job;
 {
  // Workers never wait for live authority validation.
  std::unique_lock<std::mutex> lock(s.mutex,std::try_to_lock);if(!lock.owns_lock()){out.status=RouteProofStatus::pending;return out;}
  auto found=s.jobs.find(route_key(q));
  if(found!=s.jobs.end()){
   job=found->second;
   if(job->ready.load(std::memory_order_acquire)){const auto status=unfinished(job->route.status);if(status!=RouteProofStatus::pending){out.status=status;return out;}}
   if(same_route_capture(job->receipt.validated,q)){
    if(job->receipt.status==RouteProofStatus::complete){
     const auto copy_work=job->receipt.nodes.size()+job->receipt.points.size();
     if(copy_work>budget.work_left){out.status=RouteProofStatus::budget;return out;}
     budget.work_left-=copy_work;budget.work_used+=copy_work;return job->receipt;
    }
    out.validated=job->receipt.validated;out.status=job->receipt.status;return out;
   }
   for(auto it=job->history.rbegin();it!=job->history.rend();++it){
    if(!budget.work_left){out.status=RouteProofStatus::budget;return out;}--budget.work_left;++budget.work_used;
    if(!same_route_capture(it->source,q))continue;
    out.validated=it->source;out.queried=job->requested;out.status=it->status;
    if(out.status==RouteProofStatus::complete){
     const auto copy_work=job->route.nodes.size()+job->route.centres.size()+2;
     if(copy_work>budget.work_left){out.status=RouteProofStatus::budget;return out;}
     budget.work_left-=copy_work;budget.work_used+=copy_work;
     out.nodes=job->route.nodes;out.points.reserve(job->route.centres.size()+2);
     out.points.push_back(q.origin);out.points.insert(out.points.end(),job->route.centres.begin(),job->route.centres.end());out.points.push_back(q.goal);
    }
    return out;
   }
   out.status=RouteProofStatus::pending;return out;
  }
  // Charge worst-case vector capacity and caller-declared per-job allocator
  // overhead BEFORE worker search. Expansion scratch is bounded separately by
  // expansion_limit and exists only during this scheduled query call.
  const auto nodes=std::size_t(s.policy.output_limit);
  if(nodes>(std::numeric_limits<std::size_t>::max)()/64||s.policy.job_overhead_bytes>(std::numeric_limits<std::size_t>::max)()-nodes*64){out.status=RouteProofStatus::budget;return out;}
  const auto fixed=sizeof(Impl::Job)+2*sizeof(Key)+2*sizeof(std::shared_ptr<Impl::Job>);
  if(s.policy.job_overhead_bytes+nodes*64>(std::numeric_limits<std::size_t>::max)()-fixed){out.status=RouteProofStatus::budget;return out;}
  const auto charge=fixed+s.policy.job_overhead_bytes+nodes*64;
  if(charge>s.policy.queue_bytes||s.bytes>s.policy.queue_bytes-charge){out.status=RouteProofStatus::budget;return out;}
  job=std::make_shared<Impl::Job>();job->requested=q;job->charged=charge;job->receipt.queried=q;
  s.round.push_back(route_key(q));
  try{s.jobs.emplace(route_key(q),job);}catch(...){s.round.pop_back();throw;}
  s.bytes+=charge;
 }
 StaticRouteRequest native;native.actor=q.actor;native.level_epoch=q.epoch;native.snapshot_seq=q.sequence;native.geometry_revision=q.geometry;native.ownership_generation=q.owner;native.restrictions_revision=q.restrictions;native.captured_at_ms=q.at;native.origin_node=q.origin_node;native.goal_node=q.goal_node;native.expansion_limit=s.policy.expansion_limit;native.output_limit=s.policy.output_limit;
 StaticRouteQuota quota{budget.work_left,0};
 try{job->route=s.search(native,quota,stop);}catch(...){job->route.source=native;job->route.status=StaticRouteStatus::failed;}
 budget.work_left=quota.work_left;budget.work_used+=quota.work_used;
 job->ready.store(true,std::memory_order_release);out.status=RouteProofStatus::pending;return out;
}
RouteAdvance EngineRouteProof::advance(const WorldSnapshot&w,const RouteAuthority&a){
 RouteAdvance out;auto&s=*impl_;if(!policy_ok(s.policy)||!a.identity||!a.node||!a.attachment)return out;
 std::map<std::uint32_t,const ActorSnapshot*> actors;std::set<std::uint32_t> duplicate;
 for(const auto&actor:w.actors)if(!actors.emplace(actor.key.id,&actor).second)duplicate.insert(actor.key.id);
 std::unique_lock<std::mutex> lock(s.mutex,std::try_to_lock);if(!lock.owns_lock())return out;
 std::size_t idle=0;
 while(out.work<s.policy.authority_work&&!s.round.empty()&&idle<s.round.size()){
  const auto key=s.round.front();s.round.pop_front();auto found=s.jobs.find(key);if(found==s.jobs.end())continue;
  auto&job=*found->second;const auto&q=job.requested;++out.work;
  auto ai=actors.find(q.actor.id);const auto* actor=ai==actors.end()?nullptr:ai->second;
  const bool stale=!actor||duplicate.count(q.actor.id)||!actor->alive||actor->wounded||!actor->current_position_accessible||!(actor->flags&flag_admitted)||actor->yield_reasons||actor->key!=q.actor||actor->ownership_generation!=q.owner||actor->restrictions_revision!=q.restrictions||actor->level_epoch!=w.level_epoch||actor->snapshot_seq!=w.sequence||actor->observed_at_ms!=w.now_ms||w.level_epoch!=q.epoch||w.map_hash!=q.map||w.geometry_revision!=q.geometry||w.sequence<q.sequence||w.now_ms<q.at||w.now_ms-q.at>s.policy.source_age_ms||actor->node_id!=q.origin_node||!finite(actor->position);
  if(stale&&job.ready.load(std::memory_order_acquire)){s.bytes-=job.charged;s.jobs.erase(found);++out.rejected;idle=0;continue;}
  s.round.push_back(key);
  if(stale||!job.ready.load(std::memory_order_acquire)){++idle;continue;}
  // Incomplete worker searches prove no obstruction. Retire them once on the
  // authority pass; a later budgeted worker query may retry, never this loop.
  if(unfinished(job.route.status)!=RouteProofStatus::pending){s.round.pop_back();s.bytes-=job.charged;s.jobs.erase(found);idle=0;continue;}
  const auto current=route_proof_source(w,*actor,q.goal_node,q.goal);
  if(same_route_capture(job.receipt.validated,current)){++idle;continue;}
  // Never replace a completed capture before a lagging worker can consume it.
  // Retention is bounded by the source lifetime AND the existing byte budget.
  // No old receipt is retimestamped or used for another foot position/lease.
  while(!job.history.empty()&&w.now_ms>=job.history.front().source.at&&w.now_ms-job.history.front().source.at>s.policy.source_age_ms){
   job.history.pop_front();job.charged-=Impl::validation_charge;s.bytes-=Impl::validation_charge;
  }
  if(job.receipt.status==RouteProofStatus::complete||job.receipt.status==RouteProofStatus::blocked){
   if(job.history.empty()||!same_route_capture(job.history.back().source,job.receipt.validated)){
    if(s.bytes>s.policy.queue_bytes-std::min(s.policy.queue_bytes,Impl::validation_charge)&&!job.history.empty()){
     job.history.pop_front();job.charged-=Impl::validation_charge;s.bytes-=Impl::validation_charge;
    }
    if(Impl::validation_charge<=s.policy.queue_bytes&&s.bytes<=s.policy.queue_bytes-Impl::validation_charge){
     job.history.push_back({job.receipt.validated,job.receipt.status});job.charged+=Impl::validation_charge;s.bytes+=Impl::validation_charge;
    }
   }
  }
  idle=0;
  const auto&route=job.route;
  bool valid=a.identity(a.context,q,current)&&same_static(route.source,q)&&route.status==StaticRouteStatus::complete&&route.nodes.size()==route.centres.size()&&!route.nodes.empty()&&route.nodes.size()<=s.policy.output_limit&&route.nodes.front()==q.origin_node&&route.nodes.back()==q.goal_node;
  if(!valid){job.receipt.validated=current;job.receipt.status=RouteProofStatus::blocked;++out.rejected;continue;}
  if(job.cursor<route.nodes.size()){
   const auto at=job.cursor;const auto evidence=a.node(a.context,q,current,at?route.nodes[at-1]:invalid_node,route.nodes[at],route.centres[at]);
   if(evidence==Evidence::unknown){++idle;continue;}
   if(evidence!=Evidence::yes){job.receipt.validated=current;job.receipt.status=RouteProofStatus::blocked;job.cursor=0;++out.rejected;continue;}
   ++job.cursor;continue;
  }
  // One authority work unit performs one identity check plus at most one
  // node check, or a fixed pair of endpoint checks (three callbacks maximum).
  const auto source=a.attachment(a.context,q,current,true,route.centres.front(),s.policy.endpoint_height_m);
  const auto goal=a.attachment(a.context,q,current,false,route.centres.back(),s.policy.endpoint_height_m);
  if(source==Evidence::unknown||goal==Evidence::unknown){++idle;continue;}
  job.receipt.validated=current;job.receipt.queried=q;
  if(source!=Evidence::yes||goal!=Evidence::yes){job.receipt.status=RouteProofStatus::blocked;++out.rejected;continue;}
  job.receipt.nodes=route.nodes;job.receipt.points.clear();job.receipt.points.reserve(route.centres.size()+2);job.receipt.points.push_back(current.origin);job.receipt.points.insert(job.receipt.points.end(),route.centres.begin(),route.centres.end());job.receipt.points.push_back(q.goal);job.receipt.status=RouteProofStatus::complete;++out.completed;
 }
 out.queued_bytes=s.bytes;return out;
}
}}}

