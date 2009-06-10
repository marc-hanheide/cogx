#ifndef BINDING_QUERY_UTILS_HPP_
#define BINDING_QUERY_UTILS_HPP_
#include <binding/idl/BindingQueries.hh>
#include <binding/ontology/BindingFeatureOntology.hpp>
#include <iostream>

namespace Binding {
/// helps creating queries
class QueryUtils {
  cast::PrivilegedManagedProcess& m_component;
  const std::string m_bindingSubarchID;
  const std::string& bindingSubarchID() {return m_bindingSubarchID;}
public:
  QueryUtils(cast::PrivilegedManagedProcess& _component,
	     const std::string& _bindingSubarchID) 
    : m_component(_component),
      m_bindingSubarchID(_bindingSubarchID){}

  static const BindingQueries::QueryParameters& 
  defaultQueryParameters() {
    static bool flag(false);
    static BindingQueries::QueryParameters par;
    if(!flag) {
      par.m_boundProxyInclusion = BindingQueries::INCLUDE_BOUND;
    }
    return par;
  }
  
  BindingQueries::QueryParameters
  constructQueryParameters(BindingQueries::IncludeBoundProxies _inc) {
    BindingQueries::QueryParameters par;
    par.m_boundProxyInclusion = _inc;
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
    query->m_parameters = _par;
    query->m_proxyID = CORBA::string_dup(_proxyID.c_str());
    query->m_featurePointer = _ptr;
    query->m_processed = false;
    query->m_answer = cast::cdl::triIndeterminate;
    std::string qid(m_component.newDataID());
    m_component.addToWorkingMemory(qid, bindingSubarchID(), query);
    return qid;
  }
  
  std::string 
  addAdvancedQueryToWM(const BindingData::FeaturePointer& _ptr,
		       const BindingQueries::QueryParameters& _par) {
    BindingQueries::AdvancedQuery* query = new BindingQueries::AdvancedQuery;
    query->m_parameters = _par;
    query->m_featurePointer = _ptr;
    query->m_hasTheFeatureProxyIDs.length(0);
    query->m_hasTheFeatureUnionIDs.length(0);
    query->m_matchingProxyIDs.length(0);
    query->m_matchingUnionIDs.length(0);
    query->m_nonMatchingProxyIDs.length(0);
    query->m_nonMatchingUnionIDs.length(0);
    query->m_processed = false;
    std::string qid(m_component.newDataID());
    m_component.addToWorkingMemory(qid, bindingSubarchID(), query, cast::cdl::BLOCKING);
    //cout << "Advanced query added: " << qid << endl;
    return qid;
  }
    
  template<class FeatureT>
  BindingData::FeaturePointer
  addQueryFeature(const FeatureT& _feature, 
		  BindingFeaturesCommon::TruthValue _truthValue) {
    BindingData::FeaturePointer ret;
    FeatureT* feature_ptr = new FeatureT(_feature);
    feature_ptr->m_parent.m_immediateProxyID = CORBA::string_dup("");
    feature_ptr->m_parent.m_truthValue = _truthValue;
    ret.m_address = CORBA::string_dup(m_component.newDataID().c_str());
    static const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
    
    ret.m_type    = CORBA::string_dup(ontology.featureName(typeid(FeatureT)).c_str());
    ret.m_immediateProxyID = CORBA::string_dup("");
    m_component.addToWorkingMemory(std::string(ret.m_address), 
				   m_bindingSubarchID, 
				   feature_ptr);
    
    return ret;
  }

  template<class FeatureT>
  BindingData::FeaturePointer
  dummyQueryFeature() {
    BindingData::FeaturePointer ptr;
    ptr.m_immediateProxyID = CORBA::string_dup("");
    ptr.m_address= CORBA::string_dup("");
    static const BindingFeatureOntology& 
      ontology(BindingFeatureOntology::construct());
    ptr.m_type = 
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
       << ((_q.m_parameters.m_boundProxyInclusion == 
	    BindingQueries::INCLUDE_BOUND)?"INCLUDE_BOUND":"EXCLUDE_BOUND") 
       << "]{\n"
       << "  feature: " << _q.m_featurePointer.m_address << " (" 
       << _q.m_featurePointer.m_type << ")\n"
       << "  hasTheFeatureProxyIDs: " << _q.m_hasTheFeatureProxyIDs << "\n"
       << "  hasTheFeatureUnionIDs: " << _q.m_hasTheFeatureUnionIDs << "\n"
       << "  m_matchingProxyIDs" << _q.m_matchingProxyIDs  << "\n"
       << "  m_matchingUnionIDs" << _q.m_matchingUnionIDs  << "\n"
       << "  m_nonMatchingProxyIDs" << _q.m_nonMatchingProxyIDs  << "\n"
       << "  m_nonMatchingUnionIDs" << _q.m_nonMatchingUnionIDs << "\n"
       << "}\n";
    return _out;
}
} // namespace Binding

#endif //BINDING_QUERY_UTILS_HPP_
