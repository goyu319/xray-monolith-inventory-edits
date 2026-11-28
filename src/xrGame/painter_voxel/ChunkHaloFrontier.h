// Interior cells cannot grow the six-neighbour halo. Keep frontier order identical
// to the scalar walk, including across a brick face. Missing bricks are all zero.
template<class Tiles> inline void gatherHaloFrontier(const Tiles& tiles,std::vector<Cell>& cells){
 constexpr std::uint64_t first=0x0101010101010101ull,last=first<<7;
 static const core::Tile empty{};
 for(const auto& pair:tiles){
  const Cell key=pair.first;
  auto neighbour=[&](int dx,int dy,int dz)->const core::Tile&{
   const auto it=tiles.find(Cell{key.x+dx,key.y+dy,key.z+dz});
   return it==tiles.end()?empty:it->second;
  };
  const auto& xm=neighbour(-1,0,0);const auto& xp=neighbour(1,0,0);
  const auto& ym=neighbour(0,-1,0);const auto& yp=neighbour(0,1,0);
  const auto& zm=neighbour(0,0,-1);const auto& zp=neighbour(0,0,1);
  const auto& words=pair.second.removed;
  for(int x=0;x<8;++x){
   const std::uint64_t w=words[x];
   const auto alongX=(x==0?xm.removed[7]:words[x-1])&(x==7?xp.removed[0]:words[x+1]);
   const auto alongY=((w<<8)|(ym.removed[x]>>56))&((w>>8)|(yp.removed[x]<<56));
   const auto alongZ=(((w<<1)&~first)|((zm.removed[x]>>7)&first))&
                     (((w>>1)&~last)|((zp.removed[x]<<7)&last));
   auto frontier=w&~(alongX&alongY&alongZ);
   while(frontier){
    const auto bit=lowBit64(frontier);frontier&=frontier-1;
    cells.push_back(Cell{key.x*8+x,key.y*8+int(bit>>3),key.z*8+int(bit&7)});
   }
  }
 }
}
