#ifndef BINDING_COMPARATOR_INDEX_H_
#define BINDING_COMPARATOR_INDEX_H_
#include <ext/hash_map>

namespace Binding {
/// used to index comparators
/*struct ComparatorIndex {
  ComparatorIndex(unsigned int _proxyFeatureTypeNumber, 
		 unsigned int _unionFeatureTypeNumber) 
    :   proxyFeatureTypeNumber(_proxyFeatureTypeNumber),
	unionFeatureTypeNumber(_unionFeatureTypeNumber) {}
  unsigned int proxyFeatureTypeNumber;
  unsigned int unionFeatureTypeNumber;
};*/

/*
struct CI_eq {
  bool operator()(const ComparatorIndex& _indx1, const ComparatorIndex& _indx2) const {
    return _indx1.proxyFeatureTypeNumber == _indx2.proxyFeatureTypeNumber &&
           _indx1.unionFeatureTypeNumber == _indx2.unionFeatureTypeNumber;
  }
};

struct CI_hash {
  /// based on a reasonable assumption that the number of features is less than 256^2... :-)
  int operator()(const ComparatorIndex& _indx1) const {
    return _indx1.proxyFeatureTypeNumber * 65536 + _indx1.unionFeatureTypeNumber;
  }
};

template<typename T>
struct ComparatorIndexMap {
  typedef __gnu_cxx::hash_map<const ComparatorIndex, T, CI_hash, CI_eq> map;
};
*/



} // namespace Binding

#endif // BINDING_COMPARATOR_INDEX_H_
