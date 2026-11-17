#pragma once
// Separate original-anatomy cranial region. The neck disk remains the actual
// attachment test; upper skull/brain loss is measured here without pretending
// that the thin articulation disk covers the whole damaged head.
struct CranialMeasure {
 JointMeasure tissue,brain;bool valid{};
 CranialDamage evaluate(const Grid&grid)const {
  CranialDamage result;if(!valid)return result;
  const auto a=tissue.evaluate(grid),b=brain.evaluate(grid);result.valid=true;
  result.original_m3=a.original_m3;result.removed_m3=a.removed_m3;
  result.original_bone_m3=a.original_bone_m3;result.removed_bone_m3=a.removed_bone_m3;
  result.original_brain_m3=b.original_m3;result.removed_brain_m3=b.removed_m3;return result;
 }
 std::size_t bytes()const {std::size_t n=sizeof(*this);for(const auto*m:{&tissue,&brain}){
  n+=m->blocks.capacity()*sizeof(JointBlock);for(const auto&b:m->blocks)n+=b.spans.capacity()*sizeof(JointSpan);}return n;}
};
inline CranialMeasure prepareCranialMeasure(const Grid&grid){
 CranialMeasure result;if(!grid.anatomy||!grid.material_slots)return result;
 const auto&a=*grid.anatomy;const auto&slots=*grid.material_slots;
 int low[3]={INT_MAX,INT_MAX,INT_MAX},high[3]={INT_MIN,INT_MIN,INT_MIN};
 // Original brain labels are the authored cranial witness. No head geometry,
 // station dimensions, visual name or shot power substitutes for this anatomy.
 for(std::size_t col=0;col+1<a.offsets.size();++col)for(auto r=a.offsets[col];r<a.offsets[col+1];++r){const auto&run=a.runs[r];
  if(run.label>=slots.size()||slots[run.label]!=3)continue;
  const int p[3]={int(col%a.size[0]),int(col/a.size[0]),int(run.lo)},q[3]={p[0],p[1],int(run.hi)-1};
  for(unsigned k=0;k<3;++k){low[k]=std::min(low[k],p[k]);high[k]=std::max(high[k],q[k]);}}
 if(low[0]==INT_MAX)return result;
 const int grow=int(std::ceil(.025/a.pitch));for(unsigned k=0;k<3;++k){low[k]=std::max(0,low[k]-grow);high[k]=std::min(int(a.size[k])-1,high[k]+grow);}
 // Reject a malformed/noncranial organ table rather than sample a whole body.
 for(unsigned k=0;k<3;++k)if((high[k]-low[k]+1)*a.pitch>.40)return result;
 result.tissue.pitch=result.brain.pitch=a.pitch;result.tissue.origin=result.brain.origin=Vec{a.origin[0],a.origin[1],a.origin[2]};
 std::map<Cell,std::vector<JointSpan>>tissue,brain;const double volume=a.pitch*a.pitch*a.pitch;std::size_t visited=0;
 // 2026-10-20: one span per run segment inside an 8-voxel block, continuing the block's last span exactly where the old
 // per-voxel loop merged (same column, adjacent, same bone flag); the volumes add the same constant once per voxel, so the
 // measure is bit-identical and ~2.3x faster. Budget 16M: closed-helmet heads hold up to 10.6M occupied voxels in the box.
 auto append=[](auto&blocks,int x,int y,int z,int end,bool bone){while(z<end){const int stop=std::min(end,(z/8+1)*8);auto&spans=blocks[Cell{x/8,y/8,z/8}];
  if(!spans.empty()&&spans.back().x==x&&spans.back().y==y&&spans.back().hi==z&&spans.back().bone==bone)spans.back().hi=stop;
  else spans.push_back({x,y,z,stop,bone});z=stop;}};
 auto add=[volume](double&sum,int n){for(int i=0;i<n;++i)sum+=volume;};
 for(int x=low[0];x<=high[0];++x)for(int y=low[1];y<=high[1];++y){const auto col=std::size_t(y)*a.size[0]+x;
  for(auto r=a.offsets[col];r<a.offsets[col+1];++r){const auto&run=a.runs[r];need(run.label<slots.size(),"cranial_material_label");const auto slot=slots[run.label];
   const int z0=std::max(low[2],int(run.lo)),z1=std::min(high[2],int(run.hi)-1)+1;if(z1<=z0)continue;
   visited+=std::size_t(z1-z0);need(visited<=16000000,"cranial_measure_budget");
   append(tissue,x,y,z0,z1,slot==2);add(result.tissue.tissue,z1-z0);if(slot==2)add(result.tissue.skeleton,z1-z0);
   if(slot==3){append(brain,x,y,z0,z1,false);add(result.brain.tissue,z1-z0);}}}
 for(auto&b:tissue)result.tissue.blocks.push_back({b.first,std::move(b.second)});
 for(auto&b:brain)result.brain.blocks.push_back({b.first,std::move(b.second)});
 result.valid=result.brain.tissue>0.&&result.tissue.skeleton>0.;return result;
}
