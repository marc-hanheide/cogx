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
  BindingData::BindingProxyState state;
  ProxyStateChecker(BindingData::BindingProxyState _state) : state(_state) {}
  bool test(const ProxyPtr& _ptr) const {
    return (*_ptr)->proxyState == state;
  }
  virtual abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

/// function object type that returns true iff the proxy's unionID
/// matches the desired
struct ProxyUnionIDChecker : public AbstractPredicate<ProxyPtr> {
  /// the desired unionID (not copied, so keep it in scope)
  const std::string unionID;
  ProxyUnionIDChecker(const std::string& _uid) : unionID(_uid) {}
  bool test(const ProxyPtr& _ptr) const {    
    return std::string((*_ptr)->unionID) == unionID;
  }
  virtual abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};
  
/// functor for checking the number of proxies in a union
struct UnionProxyCountCheck : public AbstractPredicate<UnionPtr>{
  /// desired number of proxies
  const unsigned int proxies;
  UnionProxyCountCheck(unsigned int _proxies) : proxies(_proxies) {}
  bool test(const UnionPtr& _ptr) const {
    return _ptr->proxyIDs().size() == proxies;
  }
  virtual abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

/// functor for checking the number of proxies bound to a proxy (without
/// having to load the union)
template<typename Comparison = std::equal_to<unsigned int> ///< The comparison that should be made. the lhs will be the actual number of proxies, and the rhs\p proxies
>
struct ProxyBindingsCountCheck : public AbstractPredicate<ProxyPtr>{
  /// the compared number of proxies (default is an equal comparison)
  const unsigned int proxies;
  ProxyBindingsCountCheck(unsigned int _proxies) : proxies(_proxies) {}
  bool test(const ProxyPtr& _ptr) const {
    return Comparison()((*_ptr)->proxyIDs.length(),proxies);
  }
  virtual abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

/// functor for checking the number of proxies bound to a proxy (Loads the union, which is inefficient)
/// \remark, implemented to test \p ProxyBindingsCountCheck
template<typename Comparison = std::equal_to<unsigned int> ///< The comparison that should be made. the lhs will be the actual number of proxies, and the rhs\p proxies
>
struct ProxyBindingsCountCheckViaUnion : public AbstractPredicate<ProxyPtr>{
  /// the compared number of proxies (default is an equal comparison)
  const unsigned int proxies;
  /// for loading the union
  BindingGraphHandler& handler; 
  ProxyBindingsCountCheckViaUnion(unsigned int _proxies, BindingGraphHandler& _handler) 
    : proxies(_proxies), 
      handler(_handler) {}
  bool test(const ProxyPtr& _ptr) const {
    if(_ptr->bindingUnionID() == "") // not bound at all, compare to 0
      return (Comparison()(0, proxies));
    UnionSet unions = handler.extractUnionsFromProxies(_ptr);
    assert(unions.size() == 1);
    UnionPtr the_union(unions.begin()->second);
    return Comparison()(the_union->proxyIDs().size(),proxies);
  }
  virtual abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

/// functor for checking the existence of features in unions or proxies
template<typename LocalBindingDataPtrT>
struct HasFeatureCheck : public AbstractPredicate<LocalBindingDataPtrT> {
  const std::string feature_type;
  /// if 0, any number of the feature type results in true, otherwise,
  /// the number must match
  const int feature_count;
  HasFeatureCheck(const std::string _feature_type, 
		  int _feature_count = 0) : feature_type(_feature_type), feature_count(_feature_count) {}
  bool test(const LocalBindingDataPtrT& _ptr) const {
    const FeatureSetWithRepetitions& fset(_ptr->featureSetWithRepetitions());
    
    if(!feature_count) { // if 0, ignore the actual number
      assert((fset.find(feature_type) != fset.end()) == _ptr->hasFeature(feature_type));
      return fset.find(feature_type) != fset.end();
    }
    
    int count = 0;
    typedef const std::pair<const std::string,OneTypeOfFeaturesWithRepetitions> P;
    foreach(P& p, fset) {
      if(p.first == feature_type)
	count++;
    }
    return count == feature_count;
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
  const std::string feature_type;
  const std::set<std::string>& externally_comparable_set;
  const std::set<std::string>& internally_comparable_set;
  
  ComparableToFeature(const std::string& _feature_type) 
    : feature_type(_feature_type),
      externally_comparable_set(BindingFeatureOntology::construct().comparableExternally(_feature_type)),
      internally_comparable_set(BindingFeatureOntology::construct().comparableInternally(_feature_type)){}
  bool test(const LocalBindingDataPtrT& _ptr) const {
    const FeatureSetWithRepetitions& fset(_ptr->featureSetWithRepetitions());
    
    typedef const std::pair<const std::string,OneTypeOfFeaturesWithRepetitions> P;
    foreach(P& p, fset) {
      if(contains_element(externally_comparable_set,p.first))
	return true;
      if(contains_element(internally_comparable_set,p.first))
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
  const std::string proxyID;
  UnionIsBoundToProxy(const std::string& _proxyID) : proxyID(_proxyID) {}
  /// initiates \p proxyID by \p _proxyptr->id()
  UnionIsBoundToProxy(const ProxyPtr& _proxyptr) : proxyID(_proxyptr->id()) {}
  bool test(const UnionPtr& _ptr) const {
    const std::set<std::string>& pset(_ptr->proxyIDs());
    return pset.find(proxyID) !=  pset.end();
  }
  virtual abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

/// checks if a union is bound to all of a set of proxies
struct UnionIsBoundToAllOfProxies : public AbstractPredicate<UnionPtr> {
  /// the IDs of the proxies the union should be bound to
  const std::set<std::string> proxyIDs;
  UnionIsBoundToAllOfProxies(const std::set<std::string>& _proxyIDs) : proxyIDs(_proxyIDs) {}
  /// initiates \p proxyID by \p _proxyptr->id()
  UnionIsBoundToAllOfProxies(const ProxySet& _proxy_set) 
    : proxyIDs(for_all(_proxy_set,insert_first<std::pair<const std::string,ProxyPtr> >()).set) {}
  bool test(const UnionPtr& _ptr) const {
    const std::set<std::string>& pset(_ptr->proxyIDs());
    // all of the tested ids must be included in the actual proxy ids
    return std::includes(pset.begin(),pset.end(),
			 proxyIDs.begin(),proxyIDs.end());
  }
  virtual abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};


/// checks if a union is bound to one of a set of proxies
struct UnionIsBoundToOneOfProxies : public AbstractPredicate<UnionPtr> {
  /// the IDs of the proxies the union should be bound to
  const std::set<std::string> proxyIDs;
  UnionIsBoundToOneOfProxies(const std::set<std::string>& _proxyIDs) : proxyIDs(_proxyIDs) {}
  /// initiates \p proxyID by \p _proxyptr->id()
  UnionIsBoundToOneOfProxies(const ProxySet& _proxy_set) 
    : proxyIDs(for_all(_proxy_set,insert_first<std::pair<const std::string,ProxyPtr> >()).set) {}
  bool test(const UnionPtr& _ptr) const {
    const std::set<std::string>& pset(_ptr->proxyIDs());
    // one of the tested ids must be included in the actual proxy ids
    foreach(std::string str, proxyIDs) {
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
  const AbstractBindingWMRepresenter& component;
  ConsistencyCheck(const AbstractBindingWMRepresenter& _component) : component(_component) {}
  
  bool test(const LocalBindingDataPtrT& _ptr) const {
    unsigned int current_version = component.component().getVersionNumber(_ptr->id(),_ptr->subarchitectureID());
    return _ptr->version() == current_version;
  }
  virtual typename AbstractPredicate<LocalBindingDataPtrT>::abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

//checks if the proxy/union is basic/group/relation
template<typename LocalBindingDataPtrT>
struct TypeCheck : public AbstractPredicate<LocalBindingDataPtrT> {
  const BindingData::BindingProxyType type;
  TypeCheck(const BindingData::BindingProxyType _type) : type(_type) {}
  
  bool test(const LocalBindingDataPtrT& _ptr) const {
    return (*_ptr)->type == type;
  }
  virtual typename AbstractPredicate<LocalBindingDataPtrT>::abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}
};

/// matches the concept of a BindingFeatures::Concept with a regex
struct ConceptRegexMatcher : public std::unary_function<BindingFeatures::Concept,bool>{
  const boost::regex regex;
  ConceptRegexMatcher(const boost::regex& _regex) : regex(_regex){}
  ConceptRegexMatcher(const std::string& _regex) : regex(_regex){}
  bool operator()(const BindingFeatures::Concept& _f) const {
    return boost::regex_match(std::string(_f.concept), 
			      regex);
  }
};

  
/// matches the label of a BindingFeatures::RelationLabel with a regex
struct RelationLabelRegexMatcher : public std::unary_function<BindingFeatures::RelationLabel,bool>{
  const boost::regex regex;
  RelationLabelRegexMatcher(const boost::regex& _regex) : regex(_regex){}
  RelationLabelRegexMatcher(const std::string& _regex) : regex(_regex){}
  bool operator()(const BindingFeatures::RelationLabel& _f) const {
    return boost::regex_match(std::string(_f.label), 
			      regex);
  }
};

/// matches the colour of a BindingFeatures::Colour with a regexp
struct ColourRegexMatcher : public std::unary_function<BindingFeatures::Colour,bool>{
  const boost::regex regex;
  ColourRegexMatcher(const boost::regex& _regex) : regex(_regex){}
  ColourRegexMatcher(const std::string& _regex) : regex(_regex){}
  bool operator()(const BindingFeatures::Colour& _f) const {
    return boost::regex_match(std::string(_f.colour), 
			      regex);
  }
};

/// checks that a \p ThisProxyID compares (using \p Compare) to its \p immediateProxyID (or a specific proxyID provided)
template<typename Compare = std::equal_to<std::string> ///< the Comparison which will be done (lhs is the proxyID or the parent.immediateProxyID and rhs the thisProxyID of the feature)
	 >
struct ThisProxyIDConsistencyChecker : public std::unary_function<BindingFeatures::ThisProxyID,bool>{
  /// the proxy_ID that will be compared (if empty, immediateProxyID will be caompared to instead)
  const std::string proxyID;
  ThisProxyIDConsistencyChecker(const std::string& _proxyID) : proxyID(_proxyID){}
  ThisProxyIDConsistencyChecker() {}
  bool operator()(const BindingFeatures::ThisProxyID& _f) const {
    if(proxyID.empty())
      return Compare()(std::string(_f.parent.immediateProxyID),std::string(_f.thisProxyID));
    else
      return  
	Compare()(proxyID,std::string(_f.thisProxyID));
  }
};




//checks if at least one of a feature of a particular type using a functor (\p MatcherT) returns true
template<typename LocalBindingDataPtrT, ///< ProxyPtr or UnionPtr
	 typename FeatureT, ///< the feature type
	 typename MatcherT  ///< should be a functor that takes an FeatureT and return a bool
	 >
struct FeatureChecker : public AbstractPredicate<LocalBindingDataPtrT> {
  const MatcherT matcher;
  FeatureChecker(const MatcherT& _matcher) : matcher(_matcher) {}
  
  bool test(const LocalBindingDataPtrT& _ptr) const {
    const FeatureSetWithRepetitions& fset(_ptr->featureSetWithRepetitions());
    const FeatureSetWithRepetitions::const_iterator feat = fset.find(cast::typeName<FeatureT>());
    if(feat == fset.end()) // feature is not even a member
      return false;
    foreach(OneTypeOfFeaturesWithRepetitions::value_type f, feat->second) { // f is a shared_ptr to an AbstractFeature
      if(matcher(extractIDLFeature<FeatureT>(*f)))
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
  BindingGraphHandler& handler; 
  FreeSingular(BindingGraphHandler& _handler) : handler(_handler) {}
  bool test(const ProxyPtr& _ptr) const {
    if(!hasFeature<ProxyPtr,BindingFeatures::Singular>().test(_ptr))
      return false;
    const BindingData::WorkingMemoryIDList& proxies((*_ptr)->proxyIDs);
    assert(proxies.length() > 0);
    //    if(proxies.length() == 1)
    //      return true;
    for(unsigned int i = 0 ; i < proxies.length() ; ++i) {
      ProxyPtr boundproxy = handler.loadProxy(std::string(proxies[i]));
      if(boundproxy->id() != _ptr->id())
	if(!hasFeature<ProxyPtr,BindingFeatures::Singular>().test(boundproxy))
	  return false; // it's enough with one nonsingular to deem that it's not free.
    }
    return true;
  }
  virtual abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};


//Returns true if the comparison between ports and the number of inports in the proxy returns true
  template <typename LocalBindingDataPtrT, 
	    typename Comparison = std::equal_to<unsigned int> ///< The comparison that should be made. the lhs will be the actual number of ports, and the rhs \p ports>
  >
struct InportsCountCheck : public AbstractPredicate<LocalBindingDataPtrT> 
{
  const unsigned int ports;
  InportsCountCheck(unsigned int _ports) : ports(_ports) {}
  bool test(const LocalBindingDataPtrT& _ptr) const {
    /// the number to compare the number of ports to
    const BindingData::ProxyPorts& ports = _ptr->inPorts();
    return Comparison()(ports.ports.length(), ports);
  }
  virtual typename AbstractPredicate<LocalBindingDataPtrT>::abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

///Returns true if the comparison between ports and the number of outports in the proxy returns true
template <typename LocalBindingDataPtrT, 
	  typename Comparison = std::equal_to<unsigned int> ///< The comparison that should be made. the lhs will be the actual number of ports, and the rhs \p ports>
>
struct OutportsCountCheck : public AbstractPredicate<LocalBindingDataPtrT> 
{
  const unsigned int ports;
  OutportsCountCheck(unsigned int _ports) : ports(_ports) {}
  bool test(const LocalBindingDataPtrT& _ptr) const {
    /// the number to compare the number of ports to
    const BindingData::ProxyPorts& ports = _ptr->rawOutPorts();
    return Comparison()(ports.ports.length(), ports);
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
  mutable ComparatorCache& comparator;
  /// the feature to compare with
  const AbstractFeature& feature;
    /// the resulting value desired
  const boost::tribool desiredTribool;
  /// the query parameters
  const BindingQueries::QueryParameters& queryParameters;
  /// the comparisons that are awaiting answering by external
  /// comparator 
  mutable std::set<std::string> featureComparisonIDs;
  /// contains last answer;
  mutable boost::logic::tribool answer;
public:
  ComparatorCacheCheck(ComparatorCache& _comparator, 
		       const AbstractFeature& _feature, 
		       const boost::tribool& _desiredTribool,
		       const BindingQueries::QueryParameters& _queryParameters) 
    : comparator(_comparator), 
      feature(_feature),
      desiredTribool(_desiredTribool),
      queryParameters(_queryParameters) {  }
public: 
  bool test(const ProxyPtr& _ptr) const;
  /// returns \p answer
  boost::logic::tribool answer() const {return answer;}
  
  /// returns the set of externally compared featurecomparisons (thrown by the \p ComparatorCache)
  const std::set<std::string>& featureComparisonIDs() const {return featureComparisonIDs;}
  bool cacheReady() const {return featureComparisonIDs().empty();}
  virtual  abstract_predicate_ptr
  clone() const {return clone_predicate(*this);}

};

template<typename Comparison = std::equal_to<unsigned int> >
struct BestListCountChecker {
  unsigned int count;
  BestListCountChecker(unsigned int _count) : count(_count) {}
  bool operator()(const ProxyPtr& _ptr) const {
    return Comparison()(_ptr->bestUnionsForProxy().unionIDs.length(), count);
  }
};

template<typename Comparison = std::equal_to<unsigned int> >
struct NonMatchingListCountChecker {
  unsigned int count;
  NonMatchingListCountChecker(unsigned int _count) : count(_count) {}
  bool operator()(const ProxyPtr& _ptr) const {
    return Comparison()(_ptr->nonMatchingUnions().nonMatchingUnionIDs.length(), count);
  }
};

/// checks stuff of the  using e.g. the BestListChecker...
template <typename Checker>
struct ProxyCheck : public AbstractPredicate<ProxyPtr> 
{
  Checker checker;
  ProxyCheck(const Checker& _checker) : checker(_checker) {}
  bool test(const ProxyPtr& _ptr) const {
    return checker(_ptr);
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
