#ifndef CURE_MAP_CONVERSION_HPP
#define CURE_MAP_CONVERSION_HPP
#include <SpatialData.hpp>
#include <Navigation/LocalGridMap.hh>

inline Cure::LocalGridMap<double> 
convertToCureMap(const SpatialData::GridMapDoublePtr &p) {
  Cure::LocalGridMap<double>ret(p->size, p->cellSize, 0.0, 
      Cure::LocalGridMap<double>::MAP1, p->x, p->y);
  for (long i = 0; i < (p->size+1)*(p->size+1); i++) {
    ret[i] = p->contents[i];
  }
  return ret;
}

inline SpatialData::GridMapDoublePtr
convertFromCureMap(const Cure::LocalGridMap<double> &map) {
  SpatialData::GridMapDoublePtr ret = new SpatialData::GridMapDouble;
  ret->size = map.getCellSize();
  ret->cellSize = map.getSize();
  ret->x = map.getCentXW();
  ret->y = map.getCentYW();
  for (long i = 0; i < map.getNumCells(); i++) {
    ret->contents[i] = map[i];
  }
  return ret;
}
#endif
