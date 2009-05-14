#include "LocalClasses.hpp"
#include "binding/feature-utils/AbstractFeature.hpp"
#include "binding/feature-utils/FeatureExtractor.hpp"
#include "binding/abstr/AbstractBinder.hpp"
//#include "ontology/BindingLocalOntology.hpp"
#include "binding/ontology/BindingFeatureOntology.hpp"
#include "binding/utils/Misc.hpp"
//#include "binding/abstr/AbstractBindingWMRepresenter.hpp"
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <functional>
#include <boost/function.hpp>
#include <boost/ref.hpp>

#include <boost/foreach.hpp>
#ifndef foreach
#define foreach BOOST_FOREACH
#endif //foreach




namespace Binding {
using namespace boost;
using namespace std;
using namespace cast;


LBindingProxy::LBindingProxy(const LBindingProxy& _lprox) 
  : LocalBindingData(_lprox.bindingWMRepresenter(),
		     _lprox.m_proxy->m_proxyFeatures,
		     _lprox.id(), 
		     _lprox.subarchitecture_id(),
		     _lprox.version()),
    m_proxy(_lprox.m_proxy),
    m_bestUnionsCache(_lprox.component(),string(_lprox.m_proxy->m_bestUnionsForProxyID)),
    m_nonMatchingUnionsCache(_lprox.component(),string(_lprox.m_proxy->m_nonMatchingUnionID)),
    m_inPortsCache(_lprox.component(),string(_lprox.m_proxy->m_inPortsID))
{ 
  //    cout << "LBindingProxy copy constructor called\n"; cout.flush();
  //    cout << "_lprox.m_proxy.get(): " << _lprox.m_proxy.get() << endl;
  shared_ptr<string> str = shared_ptr<string>(new string(subarchitectureID()));
  m_bestUnionsCache.setSubarchitectureID(str);
  m_nonMatchingUnionsCache.setSubarchitectureID(str);
  m_inPortsCache.setSubarchitectureID(str);
}
  
LBindingProxy::LBindingProxy(AbstractBindingWMRepresenter& _bindingWMRepresenter,
			     const shared_ptr<const BindingData::BindingProxy>& _proxy,
			     const string& _id,
			     const string& _subarchitecture_id,
			     unsigned int _version)
  : LocalBindingData(_bindingWMRepresenter,
		     _proxy->m_proxyFeatures,
		     _id, 
		     _subarchitecture_id,
		     _version),
    m_proxy(_proxy),
    m_bestUnionsCache(component(),string(_proxy->m_bestUnionsForProxyID)),
    m_nonMatchingUnionsCache(component(),string(_proxy->m_nonMatchingUnionID)),
    m_inPortsCache(component(),string(_proxy->m_inPortsID)) 
{ 
  shared_ptr<string> str = shared_ptr<string>(new string(subarchitectureID()));
  m_bestUnionsCache.setSubarchitectureID(str);
  m_nonMatchingUnionsCache.setSubarchitectureID(str);
  m_inPortsCache.setSubarchitectureID(str);

}
  

const FeatureSet& 
LocalBindingData::featureSet() const 
{
  if(m_featureSet.get() == NULL) {
    m_featureSet = auto_ptr<FeatureSet>(new FeatureSet(_toFeatureSet(m_features)));
  }
  return *m_featureSet;  
}

const FeatureSet& 
LocalBindingData::comparableFeatureSet() const
{
  if(m_comparableFeatureSet.get() == NULL) {
    m_comparableFeatureSet = auto_ptr<FeatureSet>(new FeatureSet(_toComparableFeatureSet(m_features)));
  }
  return *m_comparableFeatureSet;  
}

const FeatureSetWithRepetitions& 
LocalBindingData::featureSetWithRepetitions() const
{
  if(m_featureSetWithRepetitions.get() == NULL) {
    m_featureSetWithRepetitions = 
      auto_ptr<FeatureSetWithRepetitions>(new FeatureSetWithRepetitions(_toFeatureSetWithRepetitions(m_features)));
  }
  return *m_featureSetWithRepetitions;  
}

const FeatureSetWithRepetitions& 
LocalBindingData::comparableFeatureSetWithRepetitions() const
{
  if(m_comparableFeatureSetWithRepetitions.get() == NULL) {
    m_comparableFeatureSetWithRepetitions = 
      auto_ptr<FeatureSetWithRepetitions>(new FeatureSetWithRepetitions(_toComparableFeatureSetWithRepetitions(m_features)));
  }
  return *m_comparableFeatureSetWithRepetitions;    
}

/*const OptimizedFeatureSet& 
LocalBindingData::optimizedComparableFeatureSet() const
{
  if(m_optimizedComparableFeatureSet.get() == NULL) {
    m_optimizedComparableFeatureSet = auto_ptr<OptimizedFeatureSet>(new OptimizedFeatureSet(_toOptimizedComparableFeatureSet(m_features)));
  }
  return *m_optimizedComparableFeatureSet;  
}


const OptimizedFeatureSetWithRepetitions& 
LocalBindingData::optimizedComparableFeatureSetWithRepetitions() const
{
  if(m_optimizedComparableFeatureSetWithRepetitions.get() == NULL) {
    m_optimizedComparableFeatureSetWithRepetitions = 
      auto_ptr<OptimizedFeatureSetWithRepetitions>(new OptimizedFeatureSetWithRepetitions(_toOptimizedComparableFeatureSetWithRepetitions(m_features)));
  }
  return *m_optimizedComparableFeatureSetWithRepetitions;    
}
*/

 
FeatureSet 
LocalBindingData::_toFeatureSet(const BindingData::FeaturePointers& features) const
{
  FeatureSet fset;
  for(unsigned int i = 0; i < features.length() ; ++i) {
    //    log(string("fset generation: ") + lexical_cast<string>(i) + " : "+ lexical_cast<string>(features.leng.hpp()));
    //    log(string("features[i].m_type: ") + string(features[i].m_type));
    OneTypeOfFeatures& fvalues(fset[string(features[i].m_type)]);
    shared_ptr<AbstractFeature> ptr(m_bindingWMRepresenter.toFeature(features[i]));
    fvalues.insert(ptr);
  }  
  return fset; 
}

FeatureSetWithRepetitions
LocalBindingData::_toFeatureSetWithRepetitions(const BindingData::FeaturePointers& features) const
{
  FeatureSetWithRepetitions fset;
  for(unsigned int i = 0; i < features.length() ; ++i) {
    //    log(string("fset generation: ") + lexical_cast<string>(i) + " : "+ lexical_cast<string>(features.leng.hpp()));
    //    log(string("features[i].m_type: ") + string(features[i].m_type));
    OneTypeOfFeaturesWithRepetitions& fvalues(fset[string(features[i].m_type)]);
    shared_ptr<AbstractFeature> ptr(m_bindingWMRepresenter.toFeature(features[i]));
    fvalues.insert(ptr);
  }  
  return fset; 
}

FeatureSet 
LocalBindingData::_toComparableFeatureSet(const BindingData::FeaturePointers& features) const
{
  const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  FeatureSet fset;
  for(unsigned int i = 0; i < features.length() ; ++i) {
    if(ontology.comparable(string(features[i].m_type))) {
      //log(string("fset comparable generation: ") + lexical_cast<string>(i) + " : "+ lexical_cast<string>(features.length()));
      //log(string("features[i].m_type: ") + string(features[i].m_type));
      OneTypeOfFeatures& fvalues(fset[string(features[i].m_type)]);
      shared_ptr<AbstractFeature> ptr(m_bindingWMRepresenter.toFeature(features[i]));
      fvalues.insert(ptr);
    }
  }  
  return fset; 
}

FeatureSetWithRepetitions
LocalBindingData::_toComparableFeatureSetWithRepetitions(const BindingData::FeaturePointers& features) const
{
  const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  FeatureSetWithRepetitions fset;
  for(unsigned int i = 0; i < features.length() ; ++i) {
    if(ontology.comparable(string(features[i].m_type))) {
      //log(string("fset (comparable) generation: ") + lexical_cast<string>(i) + " : "+ lexical_cast<string>(features.length()));
      //log(string("features[i].m_type: ") + string(features[i].m_type));
      OneTypeOfFeaturesWithRepetitions& fvalues(fset[string(features[i].m_type)]);
      shared_ptr<AbstractFeature> ptr(m_bindingWMRepresenter.toFeature(features[i]));
      fvalues.insert(ptr);
    }
  }  
  return fset; 
}


/// only includes features that can be compared with some other feature (sorted)
/*OptimizedFeatureSet 
LocalBindingData::_toOptimizedComparableFeatureSet(const BindingData::FeaturePointers& features) const
{
  const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  OptimizedFeatureSet fset;
  for(unsigned int i = 0; i < features.length() ; ++i) {
    if(ontology.comparable(string(features[i].m_type))) {
      //log(string("fset comparable generation: ") + lexical_cast<string>(i) + " : "+ lexical_cast<string>(features.length()));
      //log(string("features[i].m_type: ") + string(features[i].m_type));
      OptimizedOneTypeOfFeatures& fvalues(fset[string(features[i].m_type)]);
      shared_ptr<AbstractFeature> ptr(m_bindingWMRepresenter.toFeature(features[i]));
      fvalues.insert(ptr);
    }
  }  
  return fset; 
}

OptimizedFeatureSetWithRepetitions 
LocalBindingData::_toOptimizedComparableFeatureSetWithRepetitions(const BindingData::FeaturePointers& features) const
{
  const BindingFeatureOntology& ontology(BindingFeatureOntology::construct());
  OptimizedFeatureSetWithRepetitions fset;
  for(unsigned int i = 0; i < features.length() ; ++i) {
    if(ontology.comparable(string(features[i].m_type))) {
      //log(string("fset (comparable) generation: ") + lexical_cast<string>(i) + " : "+ lexical_cast<string>(features.length()));
      //log(string("features[i].m_type: ") + string(features[i].m_type));
      OptimizedOneTypeOfFeaturesWithRepetitions& fvalues(fset[string(features[i].m_type)]);
      shared_ptr<AbstractFeature> ptr(m_bindingWMRepresenter.toFeature(features[i]));
      fvalues.insert(ptr);
    }
  }  
  return fset; 
}
*/

const LBindingUnion& 
LBindingProxy::bindingUnion() const throw(BindingException)
{
  if(bindingUnionID().empty()) {
    throw(BindingException("LBindingProxy::bindingUnion() ... !bound()"));
  }
  return bindingWMRepresenter().m_unionLocalCache[string(m_proxy->m_unionID)];
}

const string& 
LBindingProxy::bindingUnionID() const
{
  if(m_bindingUnionID.empty()) 
    m_bindingUnionID = m_proxy->m_unionID;
  return m_bindingUnionID;
}

const std::set<std::string>& 
LBindingProxy::bestUnionsForProxySet() const
{
  const BindingData::BestUnionsForProxy& best(bestUnionsForProxy()); // updates the cache + cache version number
  if(!m_bestUnionsForProxySet.get() || 
     m_bestUnionsForProxySetLocalVersion != m_bestUnionsCache.getVersion()) {
    m_bestUnionsForProxySetLocalVersion = m_bestUnionsCache.getVersion();
    m_bestUnionsForProxySet = std::auto_ptr<std::set<std::string> >(new std::set<std::string>); // auto_ptr deletes old... if any
    const BindingData::WorkingMemoryIDList& ids = best.m_unionIDs;
    assert(string(best.m_proxyID) == id());
    std::insert_iterator<std::set<std::string> > inserter = 
      std::inserter(*m_bestUnionsForProxySet, m_bestUnionsForProxySet->begin());
    for(unsigned int i = 0; i < ids.length() ; ++i) {
      inserter = std::string(ids[i]);
    }
  }
  return *m_bestUnionsForProxySet;
}

const std::set<std::string>& 
LBindingProxy::nonMatchingUnionsSet() const
{
  const BindingData::NonMatchingUnions& nonmatch(nonMatchingUnions()); // updates the cache + cache version number
  if(!m_nonMatchingUnionsSet.get() || 
     m_nonMatchingUnionsSetLocalVersion != m_nonMatchingUnionsCache.getVersion()) {
    m_nonMatchingUnionsSetLocalVersion = m_nonMatchingUnionsCache.getVersion();
    m_nonMatchingUnionsSet = std::auto_ptr<std::set<std::string> >(new std::set<std::string>); // auto_ptr deletes old... if any
    assert(string(nonmatch.m_proxyID) == id());
    const BindingData::WorkingMemoryIDList& ids = nonmatch.m_nonMatchingUnionIDs;
    std::insert_iterator<std::set<std::string> > inserter = 
      std::inserter(*m_nonMatchingUnionsSet, m_nonMatchingUnionsSet->begin());
    for(unsigned int i = 0; i < ids.length() ; ++i) {
      inserter = std::string(ids[i]);
    }
  }
  return *m_nonMatchingUnionsSet;
}


WorkingMemoryReaderProcess& 
LocalBindingData::component() const {
  return m_bindingWMRepresenter.component();
}

bool
proxyPortLess::operator()(const BindingData::ProxyPort& _port1,const BindingData::ProxyPort& _port2) const 
{
  assert(std::string(_port1.m_label) == std::string(_port1.m_label)); // since this is the way it will be in the \p PortMap
  if(_port1.m_type != _port2.m_type) {
    return _port1.m_type < _port2.m_type;
  } else if(string(_port1.m_ownerProxyID) != string(_port2.m_ownerProxyID)) {
    return string(_port1.m_ownerProxyID) < string(_port2.m_ownerProxyID);
  } else if(string(_port1.m_proxyID) != string(_port2.m_proxyID)) {
    return string(_port1.m_proxyID) < string(_port2.m_proxyID);
  } 
  return false; // i.e. identical
}


PortMap
toPortMap(const BindingData::ProxyPorts& _ports) {
  PortMap ret;
  for(unsigned int i = 0; i < _ports.m_ports.length() ; ++i) {
    ret[std::string(_ports.m_ports[i].m_label)].insert(_ports.m_ports[i]);
  }
  return ret;
}

const PortMap& 
LocalBindingData::
outPorts() const { 
  if(!m_outPortMap.get())
    m_outPortMap = std::auto_ptr<const PortMap>(new PortMap(toPortMap(rawOutPorts())));
  return *m_outPortMap;
}

const std::set<std::string>&
LBindingUnion::proxyIDs() const {
  if(!m_proxyIDs.get()) {
    m_proxyIDs = auto_ptr<set<string> >(new set<string>);
    for(unsigned int i = 0; i < m_union->m_proxyIDs.length(); ++i) 
      m_proxyIDs->insert(string(m_union->m_proxyIDs[i]));
  }  
  return *m_proxyIDs;
}

LBindingUnion::LBindingUnion(const LBindingUnion& _lunion) 
    : LocalBindingData(_lunion.bindingWMRepresenter(),
		       _lunion.m_union->m_unionFeatures,
		       _lunion.id(), 
		       _lunion.subarchitecture_id(),
		       _lunion.version()),
      m_union(_lunion.m_union) { }

LBindingUnion::LBindingUnion(AbstractBindingWMRepresenter& _bindingWMRepresenter, 
		const boost::shared_ptr<const BindingData::BindingUnion>& _union,
		const std::string& _id,
		const std::string& _subarchitecture_id,
		unsigned int _version) 
    : LocalBindingData(_bindingWMRepresenter, 
		       _union->m_unionFeatures, 
		       _id, 
		       _subarchitecture_id,
		       _version),
      m_union(_union) { }

LocalBindingData::LocalBindingData(AbstractBindingWMRepresenter& _bindingWMRepresenter,
				   const BindingData::FeaturePointers& _features,
				   const std::string& _id,
				   const std::string& _subarchitecture_id,
				   unsigned int _version)
  : m_bindingWMRepresenter(_bindingWMRepresenter),
    m_features(_features),
    m_id(_id),
    m_subarchitecture_id(_subarchitecture_id),
    m_version(_version) {}
  

std::string 
LocalBindingData::getGroupDetailsID() const {
  const FeatureSet& fset(featureSet());
  assert(this->type() == BindingData::GROUP);
  FeatureSet::const_iterator itr = fset.find(typeName<BindingFeatures::Group>());
  assert(itr != fset.end());
  assert(!itr->second.empty());
  return string(extractIDLFeature<BindingFeatures::Group>(*(itr->second.begin())).m_groupDetailsID);
}

const BindingFeatures::Singular&
LocalBindingData::getSingularFeature() const 
{
  const FeatureSet& fset(featureSet());
  assert(this->type() != BindingData::GROUP);
  FeatureSet::const_iterator itr = fset.find(typeName<BindingFeatures::Singular>());
  assert(itr != fset.end());
  assert(!itr->second.empty());
  return extractIDLFeature<BindingFeatures::Singular>(*(itr->second.begin()));
}

const BindingFeatures::Group&
LocalBindingData::getGroupFeature() const 
{
  const FeatureSet& fset(featureSet());
  assert(this->type() == BindingData::GROUP);
  FeatureSet::const_iterator itr = fset.find(typeName<BindingFeatures::Group>());
  assert(itr != fset.end());
  assert(!itr->second.empty());
  return extractIDLFeature<BindingFeatures::Group>(*(itr->second.begin()));
}

const std::set<std::string>&
LocalBindingData::getGroupMemberIDs() const 
{
  if(!m_groupMemberIDs.get()) {
    m_groupMemberIDs = auto_ptr<std::set<std::string> >(new std::set<std::string>());
  }
  const BindingFeatures::details::GroupDetails& details(getGroupDetails());
  assert(string(details.m_groupProxyID) == id());
  *m_groupMemberIDs = set<string>();
  for(unsigned int i = 0 ; i < details.m_groupMemberProxyIDs.length(); ++i) {
    m_groupMemberIDs->insert(string(details.m_groupMemberProxyIDs[i]));
  }
  return *m_groupMemberIDs;
}

const BindingFeatures::details::GroupDetails&
LocalBindingData::getGroupDetails() const 
{
  if(!m_groupDetailsCache.get()) {
    m_groupDetailsCache = auto_ptr<CachedCASTData<BindingFeatures::details::GroupDetails> >
      (new CachedCASTData<BindingFeatures::details::GroupDetails>(dynamic_cast<cast::WorkingMemoryReaderProcess&>(m_bindingWMRepresenter),
								  getGroupDetailsID(),
								  shared_ptr<string>(new string(subarchitecture_id())))
       );
  }
  return **m_groupDetailsCache;
}


bool 
LocalBindingData::hasFeature(const std::string& _typeName) const 
{
  FeatureSet::const_iterator itr = featureSet().find(_typeName);
  return itr != featureSet().end();
}

bool 
LocalBindingData::hasFeature(const std::type_info& _type) const
{
  static const BindingFeatureOntology& ont(BindingFeatureOntology::construct());
  return hasFeature(ont.featureName(_type));
}

BindingData::Ambiguity* 
LBindingProxy::ambiguity() const {
  std::auto_ptr<BindingData::Ambiguity> m_ambiguity;
  if(!shouldHaveAmbiguity()) {
    m_ambiguity = auto_ptr<BindingData::Ambiguity>(NULL);
    return NULL;
  }
  if(!bindingWMRepresenter().component().existsOnWorkingMemory(id()+ambiguityIDPostFix())) {
    m_ambiguity = auto_ptr<BindingData::Ambiguity>(NULL);
    return NULL;    
  }
  try {
    shared_ptr<const BindingData::Ambiguity> amb = 
      bindingWMRepresenter().loadBindingDataFromWM<BindingData::Ambiguity>(id()+ambiguityIDPostFix());
    m_ambiguity = 
      auto_ptr<BindingData::Ambiguity>(new BindingData::Ambiguity(*amb));
  } catch (const DoesNotExistOnWMException&) {
    m_ambiguity = auto_ptr<BindingData::Ambiguity>(NULL);
    return NULL;    
  }
  assert(m_ambiguity.get());
  return m_ambiguity.get();
}


const FeatureSet 
LocalBindingData::getFeatureSetComparableTo(const string& _type) const {
  const FeatureSet& fset(featureSet());
  FeatureSet ret;
  const static BindingFeatureOntology&
    ont(BindingFeatureOntology::construct());
  const std::set<std::string>& 
    comparableInternally(ont.comparableInternally(_type));
  const std::set<std::string>& 
    comparableExternally(ont.comparableExternally(_type));
  typedef pair<string,OneTypeOfFeatures> Element;
  foreach(const Element& e, fset) {
    if(contains_element(comparableExternally,e.first) ||
       contains_element(comparableInternally,e.first)) {
      ret.insert(e);
    }
  }
  return ret;
}

} // namespace Binding

