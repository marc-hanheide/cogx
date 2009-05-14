#ifndef BINDING_BASIC_EXTRACTORS_HPP_
#define BINDING_BASIC_EXTRACTORS_HPP_
#include <binding/utils/BindingUtils.hpp>
#include <binding/utils/LocalClasses.hpp>
#include <binding/utils/LocalClassesSets.hpp>
#include <binding/abstr/AbstractBindingWMRepresenter.hpp>
#include <functional>
#include <deque>
#include <boost/function.hpp>
#include <boost/regex.hpp>
#include <boost/ref.hpp>
#include <map>
#include <boost/foreach.hpp>
#ifndef foreach
#define foreach BOOST_FOREACH
#endif //foreach

namespace Binding {

/// extracts ids from proxies or unions
template<class LocalBindingDataPtrT>
struct AbstractExtractor 
  : public std::unary_function<const LocalBindingDataPtrT&, const std::set<std::string> >
{
  typedef LocalBindingDataPtrT local_ptr_type;
  virtual std::set<std::string> operator()(const LocalBindingDataPtrT&) const = 0;
protected:
  virtual ~AbstractExtractor(){}
};

/// returns the combined result from two extractors
template<class LocalBindingDataPtrT>
struct InclusiveOrExtractor 
  : public AbstractExtractor<LocalBindingDataPtrT> {
  const AbstractExtractor<LocalBindingDataPtrT>& m_extractor_1;
  const AbstractExtractor<LocalBindingDataPtrT>& m_extractor_2;
  InclusiveOrExtractor(const AbstractExtractor<LocalBindingDataPtrT>& _extractor_1,  
		       const AbstractExtractor<LocalBindingDataPtrT>& _extractor_2) 
    : m_extractor_1(_extractor_1), m_extractor_2(_extractor_2) {}
  
  virtual std::set<std::string> operator()(const LocalBindingDataPtrT& _ptr) const {
    const std::set<std::string>& ids1(m_extractor_1(_ptr));
    const std::set<std::string>& ids2(m_extractor_2(_ptr));
    std::set<std::string> ret;
    std::set_union(ids1.begin(),ids1.end(),
		   ids2.begin(),ids2.end(),
		   std::insert_iterator<std::set<std::string> >(ret,ret.begin()));
    return ret;
  }
};

/// factory functor for convenience
inline
InclusiveOrExtractor<ProxyPtr>
operator||(const AbstractExtractor<ProxyPtr>& _extractor_1,
	   const AbstractExtractor<ProxyPtr>& _extractor_2) {
  return InclusiveOrExtractor<ProxyPtr>(_extractor_1, _extractor_2);
}
/// factory functor for convenience
inline
InclusiveOrExtractor<UnionPtr>
operator||(const AbstractExtractor<UnionPtr>& _extractor_1,
	   const AbstractExtractor<UnionPtr>& _extractor_2) {
  return InclusiveOrExtractor<UnionPtr>(_extractor_1, _extractor_2);
}

/// returns the \i common result of two extractors, i.e. the intersection
template<class LocalBindingDataPtrT>
struct AndExtractor 
  : public AbstractExtractor<LocalBindingDataPtrT> {
  const AbstractExtractor<LocalBindingDataPtrT>& m_extractor_1;
  const AbstractExtractor<LocalBindingDataPtrT>& m_extractor_2;
  AndExtractor(const AbstractExtractor<LocalBindingDataPtrT>& _extractor_1,  
	       const AbstractExtractor<LocalBindingDataPtrT>& _extractor_2) 
    : m_extractor_1(_extractor_1), m_extractor_2(_extractor_2) {}
  
  virtual std::set<std::string> operator()(const LocalBindingDataPtrT& _ptr) const {
    const std::set<std::string>& ids1(m_extractor_1(_ptr));
    const std::set<std::string>& ids2(m_extractor_2(_ptr));
    std::set<std::string> ret;
    std::set_intersection(ids1.begin(),ids1.end(),
			  ids2.begin(),ids2.end(),
			  std::insert_iterator<std::set<std::string> >(ret,ret.begin()));
    return ret;
  }
};

/// factory functor for convenience
inline
AndExtractor<ProxyPtr>
operator&&(const AbstractExtractor<ProxyPtr>& _extractor_1,
	   const AbstractExtractor<ProxyPtr>& _extractor_2) {
  return AndExtractor<ProxyPtr>(_extractor_1, _extractor_2);
}
/// factory functor for convenience
inline
AndExtractor<UnionPtr>
operator&&(const AbstractExtractor<UnionPtr>& _extractor_1,
	   const AbstractExtractor<UnionPtr>& _extractor_2) {
  return AndExtractor<UnionPtr>(_extractor_1, _extractor_2);
}

/// call an extractor until no more IDs found
template<class LocalBindingDataPtrT>
struct RecursiveExtractor 
  : public AbstractExtractor<LocalBindingDataPtrT> {
  const AbstractExtractor<LocalBindingDataPtrT>& m_extractor;
  /// should refer to the handler's loading function
  boost::function<LocalBindingDataPtrT& (boost::reference_wrapper<const std::string>)> m_loader;
  
  RecursiveExtractor(const AbstractExtractor<LocalBindingDataPtrT>& _extractor,
		     boost::function<LocalBindingDataPtrT& (boost::reference_wrapper<const std::string>)> _loader) : m_extractor(_extractor),
														     m_loader(_loader) {}
														     
  virtual std::set<std::string> operator()(const LocalBindingDataPtrT& _ptr) const {
    std::set<std::string> ids; 
    std::deque<LocalBindingDataPtrT> queue;
    queue.push_back(_ptr);
    while(!queue.empty()) {
      std::set<std::string> new_ids = m_extractor(queue.front());
      for(std::set<std::string>::const_iterator itr = new_ids.begin(); 
	  itr != new_ids.end() ; 
	  ++itr) {
	if(ids.insert(*itr).second) { // if insertion was successful, i.e. it was not already expanded
	  LocalBindingDataPtrT target = m_loader(boost::cref(*itr));
	  queue.push_back(target); // put onto queue for recursive processing
	}
      }
      queue.pop_front(); 
    }
    return ids;
  }
};

} // namespace Binding
#endif //BINDING_BASIC_EXTRACTORS_HPP_

