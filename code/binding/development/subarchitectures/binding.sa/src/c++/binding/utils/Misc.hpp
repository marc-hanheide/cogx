#ifndef BINDING_UTILS_MISC_H_
#define BINDING_UTILS_MISC_H_
#include <algorithm>
#include <set>

namespace Binding {

/// neat util for lazy users
template<class T, class F>
inline
F for_all(T& _container, F _functor) {
  if(!_container.empty())
    return for_each(_container.begin(),_container.end(),_functor);
  return _functor;
}


/// calls getID and iserts the result in a set
template <typename T> 
struct insert_getID
{
  std::set<std::string> m_set;
  void operator()(const T& _t) {m_set.insert(_t->getID());}
};

/// calls inserts the first of a pair in a set
template <typename T> 
struct insert_first
{
  std::set<std::string> m_set;
  void operator()(const T& _t) {m_set.insert(_t.first);}
};


template<typename SetT>
bool 
contains_element(const SetT& _set, const typename SetT::key_type& _element)
{
  return std::binary_search(_set.begin(),_set.end(),_element);
}


} // namespace Binding

#endif // BINDING_UTILS_H_
