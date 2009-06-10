#ifndef BINDING_PREDICATES_HPP_
#define BINDING_PREDICATES_HPP_
#include <binding/utils/BasicPredicates.hpp>
#include <binding/utils/Extractors.hpp>
#include <binding/utils/GraphLoader.hpp>
#include <binding/utils/Misc.hpp>
#include <binding/idl/BindingQueries.hh>
#include <algorithm>
#include <functional>
#include <boost/foreach.hpp>
#ifndef foreach
#define foreach BOOST_FOREACH
#endif //foreach

namespace Binding {

/// checks the state and returns true if matching
struct ProxyStateChecker : public AbstractPredicate<ProxyPtr> {
  /// the desied state
  BindingData::BindingProxyState m_state;
  ProxyStateChecker(BindingData::BindingProxyState _state) : m_state(_state) {}
  bool test(const ProxyPtr& _ptr) const {
    return (*_ptr)->m_proxyState == m_state;
  }
  virtual abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

/// function object type that returns true iff the proxy's m_unionID
/// matches the desired
struct ProxyUnionIDChecker : public AbstractPredicate<ProxyPtr> {
  /// the desired unionID (not copied, so keep it in scope)
  const std::string m_unionID;
  ProxyUnionIDChecker(const std::string& _uid) : m_unionID(_uid) {}
  bool test(const ProxyPtr& _ptr) const {    
    return std::string((*_ptr)->m_unionID) == m_unionID;
  }
  virtual abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};
  
/// functor for checking the number of proxies in a union
struct UnionProxyCountCheck : public AbstractPredicate<UnionPtr>{
  /// desired number of proxies
  const unsigned int m_proxies;
  UnionProxyCountCheck(unsigned int _proxies) : m_proxies(_proxies) {}
  bool test(const UnionPtr& _ptr) const {
    return _ptr->proxyIDs().size() == m_proxies;
  }
  virtual abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

/// functor for checking the number of proxies bound to a proxy (without
/// having to load the union)
template<typename Comparison = std::equal_to<unsigned int> ///< The comparison that should be made. the lhs will be the actual number of proxies, and the rhs\p m_proxies
>
struct ProxyBindingsCountCheck : public AbstractPredicate<ProxyPtr>{
  /// the compared number of proxies (default is an equal comparison)
  const unsigned int m_proxies;
  ProxyBindingsCountCheck(unsigned int _proxies) : m_proxies(_proxies) {}
  bool test(const ProxyPtr& _ptr) const {
    return Comparison()((*_ptr)->m_proxyIDs.length(),m_proxies);
  }
  virtual abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

/// functor for checking the number of proxies bound to a proxy (Loads the union, which is inefficient)
/// \remark, implemented to test \p ProxyBindingsCountCheck
template<typename Comparison = std::equal_to<unsigned int> ///< The comparison that should be made. the lhs will be the actual number of proxies, and the rhs\p m_proxies
>
struct ProxyBindingsCountCheckViaUnion : public AbstractPredicate<ProxyPtr>{
  /// the compared number of proxies (default is an equal comparison)
  const unsigned int m_proxies;
  /// for loading the union
  BindingGraphHandler& m_handler; 
  ProxyBindingsCountCheckViaUnion(unsigned int _proxies, BindingGraphHandler& _handler) 
    : m_proxies(_proxies), 
      m_handler(_handler) {}
  bool test(const ProxyPtr& _ptr) const {
    if(_ptr->bindingUnionID() == "") // not bound at all, compare to 0
      return (Comparison()(0, m_proxies));
    UnionSet unions = m_handler.extractUnionsFromProxies(_ptr);
    assert(unions.size() == 1);
    UnionPtr the_union(unions.begin()->second);
    return Comparison()(the_union->proxyIDs().size(),m_proxies);
  }
  virtual abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

/// functor for checking the existence of features in unions or proxies
template<typename LocalBindingDataPtrT>
struct HasFeatureCheck : public AbstractPredicate<LocalBindingDataPtrT> {
  const std::string m_feature_type;
  /// if 0, any number of the feature type results in true, otherwise,
  /// the number must match
  const int m_feature_count;
  HasFeatureCheck(const std::string _feature_type, 
		  int _feature_count = 0) : m_feature_type(_feature_type), m_feature_count(_feature_count) {}
  bool test(const LocalBindingDataPtrT& _ptr) const {
    const FeatureSetWithRepetitions& fset(_ptr->featureSetWithRepetitions());
    
    if(!m_feature_count) { // if 0, ignore the actual number
      assert((fset.find(m_feature_type) != fset.end()) == _ptr->hasFeature(m_feature_type));
      return fset.find(m_feature_type) != fset.end();
    }
    
    int count = 0;
    typedef const std::pair<const std::string,OneTypeOfFeaturesWithRepetitions> P;
    foreach(P& p, fset) {
      if(p.first == m_feature_type)
	count++;
    }
    return count == m_feature_count;
  }
  virtual typename AbstractPredicate<LocalBindingDataPtrT>::abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

/// convenience function for HasFeatureCheck
template<typename LocalBindingDataPtrT, typename FeatureT>
HasFeatureCheck<LocalBindingDataPtrT> 
hasFeature(const int _n = 0)
{
  return HasFeatureCheck<LocalBindingDataPtrT>(cast::typeName<FeatureT>(), _n);
}

/// functor for checking if a proxy or union has a feature which is
/// comparable to a given feature
template<typename LocalBindingDataPtrT>
struct ComparableToFeature : public AbstractPredicate<LocalBindingDataPtrT> {
  const std::string m_feature_type;
  const std::set<std::string>& m_externally_comparable_set;
  const std::set<std::string>& m_internally_comparable_set;
  
  ComparableToFeature(const std::string& _feature_type) 
    : m_feature_type(_feature_type),
      m_externally_comparable_set(BindingFeatureOntology::construct().comparableExternally(_feature_type)),
      m_internally_comparable_set(BindingFeatureOntology::construct().comparableInternally(_feature_type)){}
  bool test(const LocalBindingDataPtrT& _ptr) const {
    const FeatureSetWithRepetitions& fset(_ptr->featureSetWithRepetitions());
    
    typedef const std::pair<const std::string,OneTypeOfFeaturesWithRepetitions> P;
    foreach(P& p, fset) {
      if(contains_element(m_externally_comparable_set,p.first))
	return true;
      if(contains_element(m_internally_comparable_set,p.first))
	return true;
    }
    return false;
  }
  virtual typename AbstractPredicate<LocalBindingDataPtrT>::abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};


/// checks if a union is bound to a proxy with a particular ID
struct UnionIsBoundToProxy : public AbstractPredicate<UnionPtr> {
  /// the ID of the proxy the union should be bound to
  const std::string m_proxyID;
  UnionIsBoundToProxy(const std::string& _proxyID) : m_proxyID(_proxyID) {}
  /// initiates \p m_proxyID by \p _proxyptr->id()
  UnionIsBoundToProxy(const ProxyPtr& _proxyptr) : m_proxyID(_proxyptr->id()) {}
  bool test(const UnionPtr& _ptr) const {
    const std::set<std::string>& pset(_ptr->proxyIDs());
    return pset.find(m_proxyID) !=  pset.end();
  }
  virtual abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

/// checks if a union is bound to all of a set of proxies
struct UnionIsBoundToAllOfProxies : public AbstractPredicate<UnionPtr> {
  /// the IDs of the proxies the union should be bound to
  const std::set<std::string> m_proxyIDs;
  UnionIsBoundToAllOfProxies(const std::set<std::string>& _proxyIDs) : m_proxyIDs(_proxyIDs) {}
  /// initiates \p m_proxyID by \p _proxyptr->id()
  UnionIsBoundToAllOfProxies(const ProxySet& _proxy_set) 
    : m_proxyIDs(for_all(_proxy_set,insert_first<std::pair<const std::string,ProxyPtr> >()).m_set) {}
  bool test(const UnionPtr& _ptr) const {
    const std::set<std::string>& pset(_ptr->proxyIDs());
    // all of the tested ids must be included in the actual proxy ids
    return std::includes(pset.begin(),pset.end(),
			 m_proxyIDs.begin(),m_proxyIDs.end());
  }
  virtual abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};


/// checks if a union is bound to one of a set of proxies
struct UnionIsBoundToOneOfProxies : public AbstractPredicate<UnionPtr> {
  /// the IDs of the proxies the union should be bound to
  const std::set<std::string> m_proxyIDs;
  UnionIsBoundToOneOfProxies(const std::set<std::string>& _proxyIDs) : m_proxyIDs(_proxyIDs) {}
  /// initiates \p m_proxyID by \p _proxyptr->id()
  UnionIsBoundToOneOfProxies(const ProxySet& _proxy_set) 
    : m_proxyIDs(for_all(_proxy_set,insert_first<std::pair<const std::string,ProxyPtr> >()).m_set) {}
  bool test(const UnionPtr& _ptr) const {
    const std::set<std::string>& pset(_ptr->proxyIDs());
    // one of the tested ids must be included in the actual proxy ids
    foreach(std::string str, m_proxyIDs) {
      if(contains_element(pset,str))
	return true;
    }
    return false;
  }
  virtual abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

/// checks if an entry is consistent with WM (needs a reference to the component to work this out)
template<typename LocalBindingDataPtrT>
struct ConsistencyCheck : public AbstractPredicate<LocalBindingDataPtrT> {
  const AbstractBindingWMRepresenter& m_component;
  ConsistencyCheck(const AbstractBindingWMRepresenter& _component) : m_component(_component) {}
  
  bool test(const LocalBindingDataPtrT& _ptr) const {
    unsigned int current_version = m_component.component().getVersionNumber(_ptr->id(),_ptr->subarchitectureID());
    return _ptr->version() == current_version;
  }
  virtual typename AbstractPredicate<LocalBindingDataPtrT>::abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

//checks if the proxy/union is basic/group/relation
template<typename LocalBindingDataPtrT>
struct TypeCheck : public AbstractPredicate<LocalBindingDataPtrT> {
  const BindingData::BindingProxyType m_type;
  TypeCheck(const BindingData::BindingProxyType _type) : m_type(_type) {}
  
  bool test(const LocalBindingDataPtrT& _ptr) const {
    return (*_ptr)->m_type == m_type;
  }
  virtual typename AbstractPredicate<LocalBindingDataPtrT>::abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}
};

/// matches the m_concept of a BindingFeatures::Concept with a regex
struct ConceptRegexMatcher : public std::unary_function<BindingFeatures::Concept,bool>{
  const boost::regex m_regex;
  ConceptRegexMatcher(const boost::regex& _regex) : m_regex(_regex){}
  ConceptRegexMatcher(const std::string& _regex) : m_regex(_regex){}
  bool operator()(const BindingFeatures::Concept& _f) const {
    return boost::regex_match(std::string(_f.m_concept), 
			      m_regex);
  }
};

  
/// matches the m_label of a BindingFeatures::RelationLabel with a regex
struct RelationLabelRegexMatcher : public std::unary_function<BindingFeatures::RelationLabel,bool>{
  const boost::regex m_regex;
  RelationLabelRegexMatcher(const boost::regex& _regex) : m_regex(_regex){}
  RelationLabelRegexMatcher(const std::string& _regex) : m_regex(_regex){}
  bool operator()(const BindingFeatures::RelationLabel& _f) const {
    return boost::regex_match(std::string(_f.m_label), 
			      m_regex);
  }
};

/// matches the m_colour of a BindingFeatures::Colour with a regexp
struct ColourRegexMatcher : public std::unary_function<BindingFeatures::Colour,bool>{
  const boost::regex m_regex;
  ColourRegexMatcher(const boost::regex& _regex) : m_regex(_regex){}
  ColourRegexMatcher(const std::string& _regex) : m_regex(_regex){}
  bool operator()(const BindingFeatures::Colour& _f) const {
    return boost::regex_match(std::string(_f.m_colour), 
			      m_regex);
  }
};

/// checks that a \p ThisProxyID compares (using \p Compare) to its \p m_immediateProxyID (or a specific proxyID provided)
template<typename Compare = std::equal_to<std::string> ///< the Comparison which will be done (lhs is the m_proxyID or the m_parent.m_immediateProxyID and rhs the m_thisProxyID of the feature)
	 >
struct ThisProxyIDConsistencyChecker : public std::unary_function<BindingFeatures::ThisProxyID,bool>{
  /// the proxy_ID that will be compared (if empty, m_immediateProxyID will be caompared to instead)
  const std::string m_proxyID;
  ThisProxyIDConsistencyChecker(const std::string& _proxyID) : m_proxyID(_proxyID){}
  ThisProxyIDConsistencyChecker() {}
  bool operator()(const BindingFeatures::ThisProxyID& _f) const {
    if(m_proxyID.empty())
      return Compare()(std::string(_f.m_parent.m_immediateProxyID),std::string(_f.m_thisProxyID));
    else
      return  
	Compare()(m_proxyID,std::string(_f.m_thisProxyID));
  }
};




//checks if at least one of a feature of a particular type using a functor (\p MatcherT) returns true
template<typename LocalBindingDataPtrT, ///< ProxyPtr or UnionPtr
	 typename FeatureT, ///< the feature type
	 typename MatcherT  ///< should be a functor that takes an FeatureT and return a bool
	 >
struct FeatureChecker : public AbstractPredicate<LocalBindingDataPtrT> {
  const MatcherT m_matcher;
  FeatureChecker(const MatcherT& _matcher) : m_matcher(_matcher) {}
  
  bool test(const LocalBindingDataPtrT& _ptr) const {
    const FeatureSetWithRepetitions& fset(_ptr->featureSetWithRepetitions());
    const FeatureSetWithRepetitions::const_iterator feat = fset.find(cast::typeName<FeatureT>());
    if(feat == fset.end()) // feature is not even a member
      return false;
    foreach(OneTypeOfFeaturesWithRepetitions::value_type f, feat->second) { // f is a shared_ptr to an AbstractFeature
      if(m_matcher(extractIDLFeature<FeatureT>(*f)))
	return true;
    }
    return false;
  }
   virtual typename AbstractPredicate<LocalBindingDataPtrT>::abstract_predicate_ptr
   clone() const {return clone_predicate(*this);}

};

/// convenience function to get a FeatureChecker
template<typename LocalBindingDataPtrT, ///< ProxyPtr or UnionPtr
	 typename FeatureT, ///< the feature type
	 typename MatcherT  ///< should be a functor that takes an FeatureT and return a bool
	 >
FeatureChecker<LocalBindingDataPtrT,FeatureT,MatcherT> 
featureCheck(const MatcherT& _matcher) 
{
  return FeatureChecker<LocalBindingDataPtrT,FeatureT,MatcherT>(_matcher);
}

/// Should return true if the proxy is a singular, and is not bound to
/// any non-singular
struct FreeSingular : public AbstractPredicate<ProxyPtr> 
{
  /// for loading proxies
  BindingGraphHandler& m_handler; 
  FreeSingular(BindingGraphHandler& _handler) : m_handler(_handler) {}
  bool test(const ProxyPtr& _ptr) const {
    if(!hasFeature<ProxyPtr,BindingFeatures::Singular>().test(_ptr))
      return false;
    const BindingData::WorkingMemoryIDList& proxies((*_ptr)->m_proxyIDs);
    assert(proxies.length() > 0);
    //    if(proxies.length() == 1)
    //      return true;
    for(unsigned int i = 0 ; i < proxies.length() ; ++i) {
      ProxyPtr boundproxy = m_handler.loadProxy(std::string(proxies[i]));
      if(boundproxy->id() != _ptr->id())
	if(!hasFeature<ProxyPtr,BindingFeatures::Singular>().test(boundproxy))
	  return false; // it's enough with one nonsingular to deem that it's not free.
    }
    return true;
  }
  virtual abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};


//Returns true if the comparison between m_ports and the number of inports in the proxy returns true
  template <typename LocalBindingDataPtrT, 
	    typename Comparison = std::equal_to<unsigned int> ///< The comparison that should be made. the lhs will be the actual number of ports, and the rhs \p m_ports>
  >
struct InportsCountCheck : public AbstractPredicate<LocalBindingDataPtrT> 
{
  const unsigned int m_ports;
  InportsCountCheck(unsigned int _ports) : m_ports(_ports) {}
  bool test(const LocalBindingDataPtrT& _ptr) const {
    /// the number to compare the number of ports to
    const BindingData::ProxyPorts& ports = _ptr->inPorts();
    return Comparison()(ports.m_ports.length(), m_ports);
  }
  virtual typename AbstractPredicate<LocalBindingDataPtrT>::abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

///Returns true if the comparison between m_ports and the number of outports in the proxy returns true
template <typename LocalBindingDataPtrT, 
	  typename Comparison = std::equal_to<unsigned int> ///< The comparison that should be made. the lhs will be the actual number of ports, and the rhs \p m_ports>
>
struct OutportsCountCheck : public AbstractPredicate<LocalBindingDataPtrT> 
{
  const unsigned int m_ports;
  OutportsCountCheck(unsigned int _ports) : m_ports(_ports) {}
  bool test(const LocalBindingDataPtrT& _ptr) const {
    /// the number to compare the number of ports to
    const BindingData::ProxyPorts& ports = _ptr->rawOutPorts();
    return Comparison()(ports.m_ports.length(), m_ports);
  }
  virtual typename AbstractPredicate<LocalBindingDataPtrT>::abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

// fwd decl
class ComparatorCache;
/// uses a \p ComparatorCache to return a result
/// \remark only intended for using inside the \p BindingScorer, any other use may result in undefined behaviour
/// \remark  implemented in \link ComparatorCache.cpp \endlink due to linking issues
struct ComparatorCacheCheck : public AbstractPredicate<ProxyPtr> 
{
private:
  mutable ComparatorCache& m_comparator;
  /// the feature to compare with
  const AbstractFeature& m_feature;
    /// the resulting value desired
  const boost::tribool m_desiredTribool;
  /// the query parameters
  const BindingQueries::QueryParameters& m_queryParameters;
  /// the comparisons that are awaiting answering by external
  /// comparator 
  mutable std::set<std::string> m_featureComparisonIDs;
  /// contains last answer;
  mutable boost::logic::tribool m_answer;
public:
  ComparatorCacheCheck(ComparatorCache& _comparator, 
		       const AbstractFeature& _feature, 
		       const boost::tribool& _desiredTribool,
		       const BindingQueries::QueryParameters& _queryParameters) 
    : m_comparator(_comparator), 
      m_feature(_feature),
      m_desiredTribool(_desiredTribool),
      m_queryParameters(_queryParameters) {  }
public: 
  bool test(const ProxyPtr& _ptr) const;
  /// returns \p m_answer
  boost::logic::tribool answer() const {return m_answer;}
  
  /// returns the set of externally compared featurecomparisons (thrown by the \p ComparatorCache)
  const std::set<std::string>& featureComparisonIDs() const {return m_featureComparisonIDs;}
  bool cacheReady() const {return featureComparisonIDs().empty();}
  virtual  abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

template<typename Comparison = std::equal_to<unsigned int> >
struct BestListCountChecker {
  unsigned int m_count;
  BestListCountChecker(unsigned int _count) : m_count(_count) {}
  bool operator()(const ProxyPtr& _ptr) const {
    return Comparison()(_ptr->bestUnionsForProxy().m_unionIDs.length(), m_count);
  }
};

template<typename Comparison = std::equal_to<unsigned int> >
struct NonMatchingListCountChecker {
  unsigned int m_count;
  NonMatchingListCountChecker(unsigned int _count) : m_count(_count) {}
  bool operator()(const ProxyPtr& _ptr) const {
    return Comparison()(_ptr->nonMatchingUnions().m_nonMatchingUnionIDs.length(), m_count);
  }
};

/// checks stuff of the  using e.g. the BestListChecker...
template <typename Checker>
struct ProxyCheck : public AbstractPredicate<ProxyPtr> 
{
  Checker m_checker;
  ProxyCheck(const Checker& _checker) : m_checker(_checker) {}
  bool test(const ProxyPtr& _ptr) const {
    return m_checker(_ptr);
  }
  virtual typename AbstractPredicate<ProxyPtr>::abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}
};

/// convenience fct
template<typename Checker>
ProxyCheck<Checker> proxyCheck(const Checker& _checker) {
  return ProxyCheck<Checker>(_checker);
}
  
} // namespace Binding

#endif //BINDING_PREDICATES_HPP_
