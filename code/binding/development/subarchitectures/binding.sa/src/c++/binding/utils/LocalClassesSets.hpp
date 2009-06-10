#ifndef BINDING_LOCAL_CLASSES_SETS_H_
#define BINDING_LOCAL_CLASSES_SETS_H_
#include <binding/utils/LocalClasses.hpp>
#include <iostream>
#include <iterator>
#include <boost/foreach.hpp>
#ifndef foreach
#define foreach BOOST_FOREACH
#endif //foreach

namespace Binding {

template<typename LocalDataPtrT>
struct LocalDataSets {
  typedef std::map<std::string,LocalDataPtrT> set;
};

typedef LocalDataSets<ProxyPtr>::set ProxySet;

inline
ProxySet& 
operator+=(ProxySet& _set1, const ProxySet& _set2){
  _set1.insert(_set2.begin(),_set2.end());
  return _set1;
}

inline
ProxySet 
operator+(const ProxySet& _set1, const ProxySet& _set2){
  ProxySet ret(_set1);
  ret += _set2;
  return ret;
}

inline
ProxySet&
operator-=(ProxySet& _set1, const ProxySet& _set2){
  ProxySet ret;  
  set_difference(_set1.begin(),_set1.end(),
		 _set2.begin(),_set2.end(),
		 std::insert_iterator<ProxySet>(ret,ret.begin()));
  _set1=ret;
  return _set1;
}

inline
ProxySet 
operator-(ProxySet& _set1, const ProxySet& _set2){
  ProxySet ret(_set1);
  ret -= _set2;
  return ret;
}

/// returns a subset matching a property
template<typename Predicate>
ProxySet 
subset(const ProxySet& _set, const Predicate& _predicate){
  ProxySet ret;
  std::insert_iterator<ProxySet> inserter = std::inserter(ret,ret.begin());
  foreach(ProxySet::value_type p, _set) {
    if(_predicate(p))
      inserter = p;
  }
  return ret;
}

template<typename Predicate>
ProxySet 
operator|(const ProxySet& _set, const Predicate& _predicate)
{
  return subset(_set,_predicate);
}
 

inline
std::ostream& 
operator<<(std::ostream& _out, const ProxySet& _set)
{
  ProxySet::const_iterator i = _set.begin();
  _out << "{";
  while(i != _set.end()) {
    _out << (i++)->first ;
    if(i != _set.end())
      _out << ", ";
  }
  _out << "}";
  return _out;
}

typedef LocalDataSets<UnionPtr>::set UnionSet;


inline
UnionSet& 
operator+=(UnionSet& _set1, const UnionSet& _set2){
  _set1.insert(_set2.begin(),_set2.end());
  return _set1;
}

inline
UnionSet 
operator+(const UnionSet& _set1, const UnionSet& _set2){
  UnionSet ret(_set1);
  ret += _set2;
  return ret;
}

inline
UnionSet& 
operator-=(UnionSet& _set1, const UnionSet& _set2){
  UnionSet ret;  
  set_difference(_set1.begin(),_set1.end(),
		 _set2.begin(),_set2.end(),
		 std::insert_iterator<UnionSet>(ret,ret.begin()));
  _set1=ret;
  return _set1;
}

inline
UnionSet 
operator-(UnionSet& _set1, const UnionSet& _set2){
  UnionSet ret(_set1);
  ret -= _set2;
  return ret;
}

/// returns a subset matching a property
template<typename Predicate>
UnionSet 
subset(const UnionSet& _set, const Predicate& _predicate){
  UnionSet ret;
  std::insert_iterator<UnionSet> inserter = std::inserter(ret,ret.begin());
  foreach(UnionSet::value_type p, _set) {
    if(_predicate(p))
      inserter = p;
  }
  return ret;
}

template<typename Predicate>
UnionSet 
operator|(const UnionSet& _set, const Predicate& _predicate)
{
  return subset(_set,_predicate);
}

inline
std::ostream& 
operator<<(std::ostream& _out, const UnionSet& _set)
{
  UnionSet::const_iterator i = _set.begin();
  _out << "{";
  while(i != _set.end()) {
    _out << (i++)->first ;
    if(i != _set.end())
      _out << ", ";
  }
  _out << "}";
  return _out;
}


} // namespace Binding 
#endif // BINDING_LOCAL_CLASSES_SETS_H_
