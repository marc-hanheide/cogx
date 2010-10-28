#ifndef CURE_MAP_CONVERSION_HPP
#define CURE_MAP_CONVERSION_HPP
#include <FrontierInterface.hpp>
#include <Navigation/LocalGridMap.hh>

inline void
convertToCureMap(const FrontierInterface::GridMapDoublePtr &p,
    Cure::LocalGridMap<double> &ret) {
  for (long i = 0; i < (2*p->size+1)*(2*p->size+1); i++) {
    ret[i] = p->contents[i];
  }
}

inline FrontierInterface::GridMapDoublePtr
convertFromCureMap(const Cure::LocalGridMap<double> &map) {
  FrontierInterface::GridMapDoublePtr ret = new FrontierInterface::GridMapDouble;
  ret->size = map.getSize();
  ret->cellSize = map.getCellSize();
  ret->x = map.getCentXW();
  ret->y = map.getCentYW();
  for (long i = 0; i < map.getNumCells(); i++) {
    ret->contents.push_back(map[i]);
  }
  return ret;
}
#endif
