#ifndef GRIDMAPWRAPPER_H
#define GRIDMAPWRAPPER_H

#include <cmath>
#include <iterator>

namespace SpatialGridMap {

/**
 Class GridMapWrapper, wraps any type in a format
 compatible with SpatialGridMap
 */
template<class Data>
struct GridMapWrapper {
  Data data;

  GridMapWrapper() {
  }
  ;
  GridMapWrapper(Data data) :
    data(data) {
  }
  ;

  // Equal if obs is same, and prob within threshold
  bool operator==(GridMapWrapper & other) {
    return data == other.data;
  }

  // Sets this data to be a merge of n other data instances
  template<class Iterator>
  void merge(Iterator bloxels, int n) {
    //Since all values are equal, just take the first one
    data = bloxels->data;
  }
};

template<class Data>
inline std::ostream& operator <<(std::ostream& stream, const GridMapWrapper<
    Data> & wrapper) {
  stream << wrapper.data << " ";
  return stream;
}

template<class Data>
inline std::istream& operator >>(std::istream& stream,
    GridMapWrapper<Data> & wrapper) {
  stream >> wrapper.data;
  return stream;
}

}
; // end of namespace

#endif
