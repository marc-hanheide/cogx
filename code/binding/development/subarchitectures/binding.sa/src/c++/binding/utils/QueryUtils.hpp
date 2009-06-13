#ifndef BINDING_QUERY_UTILS_HPP_
#define BINDING_QUERY_UTILS_HPP_
#include <binding/idl/BindingQueries.hh>
#include <binding/ontology/BindingFeatureOntology.hpp>
#include <iostream>

namespace Binding {
/// helps creating queries
class QueryUtils {
  cast::PrivilegedManagedProcess& component;
  const std::string bindingSubarchID;
  const std::string& bindingSubarchID() {return bindingSubarchID;}
public:
  QueryUtils(cast::PrivilegedManagedProcess& _component,
	     const std::string& _bindingSubarchID) 
    : component(_component),
      bindingSubarchID(_bindingSubarchID){}

  static const BindingQueries::QueryParameters& 
  defaultQueryParameters() {
    static bool flag(false);
    static BindingQueries::QueryParameters par;
    if(!flag) {
      par.boundProxyInclusion = BindingQueries::INCLUDE_BOUND;
    }
    return par;
  }
  
  BindingQueries::QueryParameters
  constructQueryParameters(BindingQueries::IncludeBoundProxies _inc) {
    BindingQueries::QueryParameters par;
    par.boundProxyInclusion = _inc;
    return par;
  }
  
    
public:
  
  /// asks a query based on an instantiated feature, e.g. is the
  /// \p BindingFeatures::Colour of a particular proxy (or its union) "red'.
  /// returns the WM ID of the stored query
  template<class FeatureT>  
  std::string
  askBasicQuery(const FeatureT& _feature,
		const std::string& _proxyID,
		const BindingQueries::QueryParameters& _par = 
		defaultQueryParameters(),
		BindingFeaturesCommon::TruthValue _truthValue 
		= BindingFeaturesCommon::POSITIVE) 
  {
    BindingData::FeaturePointer ptr = 
      addQueryFeature(_feature,_truthValue);
    return addBasicQueryToWM(ptr, _proxyID, _par);
  }

  /// asks a basic query without an instantiated feature, i.e. it asks
  /// if a proxy (or the proxy's union) has the feature or not.
  template<class FeatureT>  
  std::string
  askBasicQuery(const std::string& _proxyID,
		const BindingQueries::QueryParameters& _par = 
		defaultQueryParameters()) 
  {
    BindingData::FeaturePointer ptr = dummyQueryFeature<FeatureT>();
    return addBasicQueryToWM(ptr, _proxyID, _par);
  }

  /// asks a query based on an instantiated feature, e.g. what proxies
  /// (or corresponding unions) has a \p BindingFeatures::Colour that
  /// is "red" (or not). Returns the WM ID of the stored query
  template<class FeatureT>  
  std::string
  askAdvancedQuery(const FeatureT& _feature,
		   const BindingQueries::QueryParameters& _par = 
		   defaultQueryParameters(),
		   BindingFeaturesCommon::TruthValue _truthValue =
		   BindingFeaturesCommon::POSITIVE) 
  {
    BindingData::FeaturePointer ptr = 
      addQueryFeature(_feature,_truthValue);
    return addAdvancedQueryToWM(ptr, _par);
  }

  /// asks a basic query without an instantiated feature, i.e. it asks
  /// what proxies (or the proxies' unions) that has the feature.
  template<class FeatureT>  
  std::string
  askAdvancedQuery(const BindingQueries::QueryParameters& _par = 
		   defaultQueryParameters()) 
  {
    BindingData::FeaturePointer ptr = dummyQueryFeature<FeatureT>();
    return addAdvancedQueryToWM(ptr, _par);
  }

private:
  std::string 
  addBasicQueryToWM(const BindingData::FeaturePointer& _ptr,
		    const std::string& _proxyID,
		    const BindingQueries::QueryParameters& _par) {
    BindingQueries::BasicQuery* query = new BindingQueries::BasicQuery;
    query->parameters = _par;
    query->proxyID = CORBA::string_dup(_proxyID.c_str());
    query->featurePointer = _ptr;
    query->processed = false;
    query->answer = BindingData::INDETERMINATETB;
    std::string qid(component.newDataID());
    component.addToWorkingMemory(qid, bindingSubarchID(), query);
    return qid;
  }
  
  std::string 
  addAdvancedQueryToWM(const BindingData::FeaturePointer& _ptr,
		       const BindingQueries::QueryParameters& _par) {
    BindingQueries::AdvancedQuery* query = new BindingQueries::AdvancedQuery;
    query->parameters = _par;
    query->featurePointer = _ptr;
    query->hasTheFeatureProxyIDs.length(0);
    query->hasTheFeatureUnionIDs.length(0);
    query->matchingProxyIDs.length(0);
    query->matchingUnionIDs.length(0);
    query->nonMatchingProxyIDs.length(0);
    query->nonMatchingUnionIDs.length(0);
    query->processed = false;
    std::string qid(component.newDataID());
    component.addToWorkingMemory(qid, bindingSubarchID(), query, cast::cdl::BLOCKING);
    //cout << "Advanced query added: " << qid << endl;
    return qid;
  }
    
  template<class FeatureT>
  BindingData::FeaturePointer
  addQueryFeature(const FeatureT& _feature, 
		  BindingFeaturesCommon::TruthValue _truthValue) {
    BindingData::FeaturePointer ret;
    FeatureT* feature_ptr = new FeatureT(_feature);
    feature_ptr->parent.immediateProxyID = CORBA::string_dup("");
    feature_ptr->parent.truthValue = _truthValue;
    ret.address = CORBA::string_dup(component.newDataID().c_str());
    static const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
    
    ret.type    = CORBA::string_dup(ontology.featureName(typeid(FeatureT)).c_str());
    ret.immediateProxyID = CORBA::string_dup("");
    component.addToWorkingMemory(std::string(ret.address), 
				   bindingSubarchID, 
				   feature_ptr);
    
    return ret;
  }

  template<class FeatureT>
  BindingData::FeaturePointer
  dummyQueryFeature() {
    BindingData::FeaturePointer ptr;
    ptr.immediateProxyID = CORBA::string_dup("");
    ptr.address= CORBA::string_dup("");
    static const BindingFeatureOntology& 
      ontology(BindingFeatureOntology::construct());
    ptr.type = 
      CORBA::string_dup(ontology.featureName(typeid(FeatureT)).c_str());
    return ptr;
  }
};

std::ostream&
operator<<(std::ostream& _out, const BindingData::WorkingMemoryIDList& _l) 
{
  _out << "{";
  if(_l.length()) {
    _out << _l[0];
    for(unsigned int i = 1 ; i < _l.length() ; ++i) {
      _out << ", " << _l[i];
    }
  }
  _out << "}";
  return _out;
}

std::ostream&
operator<<(std::ostream& _out, const BindingQueries::AdvancedQuery& _q) {
  _out << "AdvancedQuery[" 
       << ((_q.parameters.boundProxyInclusion == 
	    BindingQueries::INCLUDE_BOUND)?"INCLUDE_BOUND":"EXCLUDE_BOUND") 
       << "]{\n"
       << "  feature: " << _q.featurePointer.address << " (" 
       << _q.featurePointer.type << ")\n"
       << "  hasTheFeatureProxyIDs: " << _q.hasTheFeatureProxyIDs << "\n"
       << "  hasTheFeatureUnionIDs: " << _q.hasTheFeatureUnionIDs << "\n"
       << "  matchingProxyIDs" << _q.matchingProxyIDs  << "\n"
       << "  matchingUnionIDs" << _q.matchingUnionIDs  << "\n"
       << "  nonMatchingProxyIDs" << _q.nonMatchingProxyIDs  << "\n"
       << "  nonMatchingUnionIDs" << _q.nonMatchingUnionIDs << "\n"
       << "}\n";
    return _out;
}
} // namespace Binding

#endif //BINDING_QUERY_UTILS_HPP_
