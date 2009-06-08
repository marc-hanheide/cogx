#ifndef BINDING_TYPE_MAP_H_
#define BINDING_TYPE_MAP_H_

#include <balt/core/TypeMap.hpp>

//moved it...
/*
namespace Binding {

struct TI_eq {
  bool operator()(const std::type_info* _p, const std::type_info* _q) const {return *_p==*_q;}
};

struct TI_hash {
  int operator()(const std::type_info* _p) const {return reinterpret_cast<int>(_p); }
};

/// see Stroustrup 15.4.4.1
template<typename TypeInfoT>
struct TypeMap {
  typedef __gnu_cxx::hash_map<const std::type_info*, TypeInfoT, TI_hash, TI_eq> map;
};


} // namespace Binding

*/
#endif // BINDING_TYPE_MAP_H_

