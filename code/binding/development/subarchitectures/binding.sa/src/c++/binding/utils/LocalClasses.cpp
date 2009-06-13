#include "LocalClasses.hpp"
#include "../feature-utils/AbstractFeature.hpp"
#include "../feature-utils/FeatureExtractor.hpp"
#include "../abstr/AbstractBinder.hpp"
//#include "ontology/BindingLocalOntology.hpp"
#include "../ontology/BindingFeatureOntology.hpp"
#include "../utils/Misc.hpp"
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
		     _lprox.proxy->proxyFeatures,
		     _lprox.id(),
		     _lprox.subarchitecture_id(),
		     _lprox.version()),
    proxy(_lprox.proxy),
    bestUnionsCache(_lprox.component(),string(_lprox.proxy->bestUnionsForProxyID)),
    nonMatchingUnionsCache(_lprox.component(),string(_lprox.proxy->nonMatchingUnionID)),
    inPortsCache(_lprox.component(),string(_lprox.proxy->inPortsID))
{
  //    cout << "LBindingProxy copy constructor called\n"; cout.flush();
  //    cout << "_lprox.proxy.get(): " << _lprox.proxy.get() << endl;
  shared_ptr<string> str = shared_ptr<string>(new string(subarchitectureID()));
  bestUnionsCache.setSubarchitectureID(str);
  nonMatchingUnionsCache.setSubarchitectureID(str);
  inPortsCache.setSubarchitectureID(str);
}

LBindingProxy::LBindingProxy(AbstractBindingWMRepresenter& _bindingWMRepresenter,
			     const shared_ptr<const BindingData::BindingProxy>& _proxy,
			     const string& _id,
			     const string& _subarchitecture_id,
			     unsigned int _version)
  : LocalBindingData(_bindingWMRepresenter,
		     _proxy->proxyFeatures,
		     _id,
		     _subarchitecture_id,
		     _version),
    proxy(_proxy),
    bestUnionsCache(component(),string(_proxy->bestUnionsForProxyID)),
    nonMatchingUnionsCache(component(),string(_proxy->nonMatchingUnionID)),
    inPortsCache(component(),string(_proxy->inPortsID))
{
  shared_ptr<string> str = shared_ptr<string>(new string(subarchitectureID()));
  bestUnionsCache.setSubarchitectureID(str);
  nonMatchingUnionsCache.setSubarchitectureID(str);
  inPortsCache.setSubarchitectureID(str);

}


const FeatureSet&
LocalBindingData::featureSet() const
{
  if(featureSet.get() == NULL) {
    featureSet = auto_ptr<FeatureSet>(new FeatureSet(_toFeatureSet(features)));
  }
  return *featureSet;
}

const FeatureSet&
LocalBindingData::comparableFeatureSet() const
{
  if(comparableFeatureSet.get() == NULL) {
    comparableFeatureSet = auto_ptr<FeatureSet>(new FeatureSet(_toComparableFeatureSet(features)));
  }
  return *comparableFeatureSet;
}

const FeatureSetWithRepetitions&
LocalBindingData::featureSetWithRepetitions() const
{
  if(featureSetWithRepetitions.get() == NULL) {
    featureSetWithRepetitions =
      auto_ptr<FeatureSetWithRepetitions>(new FeatureSetWithRepetitions(_toFeatureSetWithRepetitions(features)));
  }
  return *featureSetWithRepetitions;
}

const FeatureSetWithRepetitions&
LocalBindingData::comparableFeatureSetWithRepetitions() const
{
  if(comparableFeatureSetWithRepetitions.get() == NULL) {
    comparableFeatureSetWithRepetitions =
      auto_ptr<FeatureSetWithRepetitions>(new FeatureSetWithRepetitions(_toComparableFeatureSetWithRepetitions(features)));
  }
  return *comparableFeatureSetWithRepetitions;
}

/*const OptimizedFeatureSet&
LocalBindingData::optimizedComparableFeatureSet() const
{
  if(optimizedComparableFeatureSet.get() == NULL) {
    optimizedComparableFeatureSet = auto_ptr<OptimizedFeatureSet>(new OptimizedFeatureSet(_toOptimizedComparableFeatureSet(features)));
  }
  return *optimizedComparableFeatureSet;
}


const OptimizedFeatureSetWithRepetitions&
LocalBindingData::optimizedComparableFeatureSetWithRepetitions() const
{
  if(optimizedComparableFeatureSetWithRepetitions.get() == NULL) {
    optimizedComparableFeatureSetWithRepetitions =
      auto_ptr<OptimizedFeatureSetWithRepetitions>(new OptimizedFeatureSetWithRepetitions(_toOptimizedComparableFeatureSetWithRepetitions(features)));
  }
  return *optimizedComparableFeatureSetWithRepetitions;
}
*/


FeatureSet
LocalBindingData::_toFeatureSet(const BindingData::FeaturePointers& features) const
{
  FeatureSet fset;
  for(unsigned int i = 0; i < features.length() ; ++i) {
    //    log(string("fset generation: ") + lexical_cast<string>(i) + " : "+ lexical_cast<string>(features.leng.hpp()));
    //    log(string("features[i].type: ") + string(features[i].type));
    OneTypeOfFeatures& fvalues(fset[string(features[i].type)]);
    shared_ptr<AbstractFeature> ptr(bindingWMRepresenter.toFeature(features[i]));
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
    //    log(string("features[i].type: ") + string(features[i].type));
    OneTypeOfFeaturesWithRepetitions& fvalues(fset[string(features[i].type)]);
    shared_ptr<AbstractFeature> ptr(bindingWMRepresenter.toFeature(features[i]));
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
    if(ontology.comparable(string(features[i].type))) {
      //log(string("fset comparable generation: ") + lexical_cast<string>(i) + " : "+ lexical_cast<string>(features.length()));
      //log(string("features[i].type: ") + string(features[i].type));
      OneTypeOfFeatures& fvalues(fset[string(features[i].type)]);
      shared_ptr<AbstractFeature> ptr(bindingWMRepresenter.toFeature(features[i]));
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
    if(ontology.comparable(string(features[i].type))) {
      //log(string("fset (comparable) generation: ") + lexical_cast<string>(i) + " : "+ lexical_cast<string>(features.length()));
      //log(string("features[i].type: ") + string(features[i].type));
      OneTypeOfFeaturesWithRepetitions& fvalues(fset[string(features[i].type)]);
      shared_ptr<AbstractFeature> ptr(bindingWMRepresenter.toFeature(features[i]));
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
    if(ontology.comparable(string(features[i].type))) {
      //log(string("fset comparable generation: ") + lexical_cast<string>(i) + " : "+ lexical_cast<string>(features.length()));
      //log(string("features[i].type: ") + string(features[i].type));
      OptimizedOneTypeOfFeatures& fvalues(fset[string(features[i].type)]);
      shared_ptr<AbstractFeature> ptr(bindingWMRepresenter.toFeature(features[i]));
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
    if(ontology.comparable(string(features[i].type))) {
      //log(string("fset (comparable) generation: ") + lexical_cast<string>(i) + " : "+ lexical_cast<string>(features.length()));
      //log(string("features[i].type: ") + string(features[i].type));
      OptimizedOneTypeOfFeaturesWithRepetitions& fvalues(fset[string(features[i].type)]);
      shared_ptr<AbstractFeature> ptr(bindingWMRepresenter.toFeature(features[i]));
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
  return bindingWMRepresenter().unionLocalCache[string(proxy->unionID)];
}

const string&
LBindingProxy::bindingUnionID() const
{
  if(bindingUnionID.empty())
    bindingUnionID = proxy->unionID;
  return bindingUnionID;
}

const std::set<std::string>&
LBindingProxy::bestUnionsForProxySet() const
{
  const BindingData::BestUnionsForProxy& best(bestUnionsForProxy()); // updates the cache + cache version number
  if(!bestUnionsForProxySet.get() ||
     bestUnionsForProxySetLocalVersion != bestUnionsCache.getVersion()) {
    bestUnionsForProxySetLocalVersion = bestUnionsCache.getVersion();
    bestUnionsForProxySet = std::auto_ptr<std::set<std::string> >(new std::set<std::string>); // auto_ptr deletes old... if any
    const BindingData::WorkingMemoryIDList& ids = best.unionIDs;
    assert(string(best.proxyID) == id());
    std::insert_iterator<std::set<std::string> > inserter =
      std::inserter(*bestUnionsForProxySet, bestUnionsForProxySet->begin());
    for(unsigned int i = 0; i < ids.length() ; ++i) {
      inserter = std::string(ids[i]);
    }
  }
  return *bestUnionsForProxySet;
}

const std::set<std::string>&
LBindingProxy::nonMatchingUnionsSet() const
{
  const BindingData::NonMatchingUnions& nonmatch(nonMatchingUnions()); // updates the cache + cache version number
  if(!nonMatchingUnionsSet.get() ||
     nonMatchingUnionsSetLocalVersion != nonMatchingUnionsCache.getVersion()) {
    nonMatchingUnionsSetLocalVersion = nonMatchingUnionsCache.getVersion();
    nonMatchingUnionsSet = std::auto_ptr<std::set<std::string> >(new std::set<std::string>); // auto_ptr deletes old... if any
    assert(string(nonmatch.proxyID) == id());
    const BindingData::WorkingMemoryIDList& ids = nonmatch.nonMatchingUnionIDs;
    std::insert_iterator<std::set<std::string> > inserter =
      std::inserter(*nonMatchingUnionsSet, nonMatchingUnionsSet->begin());
    for(unsigned int i = 0; i < ids.length() ; ++i) {
      inserter = std::string(ids[i]);
    }
  }
  return *nonMatchingUnionsSet;
}


WorkingMemoryReaderProcess&
LocalBindingData::component() const {
  return bindingWMRepresenter.component();
}

bool
proxyPortLess::operator()(const BindingData::ProxyPort& _port1,const BindingData::ProxyPort& _port2) const
{
  assert(std::string(_port1.label) == std::string(_port1.label)); // since this is the way it will be in the \p PortMap
  if(_port1.type != _port2.type) {
    return _port1.type < _port2.type;
  } else if(string(_port1.ownerProxyID) != string(_port2.ownerProxyID)) {
    return string(_port1.ownerProxyID) < string(_port2.ownerProxyID);
  } else if(string(_port1.proxyID) != string(_port2.proxyID)) {
    return string(_port1.proxyID) < string(_port2.proxyID);
  }
  return false; // i.e. identical
}


PortMap
toPortMap(const BindingData::ProxyPorts& _ports) {
  PortMap ret;
  for(unsigned int i = 0; i < _ports.ports.length() ; ++i) {
    ret[std::string(_ports.ports[i].label)].insert(_ports.ports[i]);
  }
  return ret;
}

const PortMap&
LocalBindingData::
outPorts() const {
  if(!outPortMap.get())
    outPortMap = std::auto_ptr<const PortMap>(new PortMap(toPortMap(rawOutPorts())));
  return *outPortMap;
}

const std::set<std::string>&
LBindingUnion::proxyIDs() const {
  if(!proxyIDs.get()) {
    proxyIDs = auto_ptr<set<string> >(new set<string>);
    for(unsigned int i = 0; i < union->proxyIDs.length(); ++i)
      proxyIDs->insert(string(union->proxyIDs[i]));
  }
  return *proxyIDs;
}

LBindingUnion::LBindingUnion(const LBindingUnion& _lunion)
    : LocalBindingData(_lunion.bindingWMRepresenter(),
		       _lunion.union->unionFeatures,
		       _lunion.id(),
		       _lunion.subarchitecture_id(),
		       _lunion.version()),
      union(_lunion.union) { }

LBindingUnion::LBindingUnion(AbstractBindingWMRepresenter& _bindingWMRepresenter,
		const boost::shared_ptr<const BindingData::BindingUnion>& _union,
		const std::string& _id,
		const std::string& _subarchitecture_id,
		unsigned int _version)
    : LocalBindingData(_bindingWMRepresenter,
		       _union->unionFeatures,
		       _id,
		       _subarchitecture_id,
		       _version),
      union(_union) { }

LocalBindingData::LocalBindingData(AbstractBindingWMRepresenter& _bindingWMRepresenter,
				   const BindingData::FeaturePointers& _features,
				   const std::string& _id,
				   const std::string& _subarchitecture_id,
				   unsigned int _version)
  : bindingWMRepresenter(_bindingWMRepresenter),
    features(_features),
    id(_id),
    subarchitecture_id(_subarchitecture_id),
    version(_version) {}


std::string
LocalBindingData::getGroupDetailsID() const {
  const FeatureSet& fset(featureSet());
  assert(this->type() == BindingData::GROUP);
  FeatureSet::const_iterator itr = fset.find(typeName<BindingFeatures::Group>());
  assert(itr != fset.end());
  assert(!itr->second.empty());
  return string(extractIDLFeature<BindingFeatures::Group>(*(itr->second.begin())).groupDetailsID);
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
  if(!groupMemberIDs.get()) {
    groupMemberIDs = auto_ptr<std::set<std::string> >(new std::set<std::string>());
  }
  const BindingFeatures::details::GroupDetails& details(getGroupDetails());
  assert(string(details.groupProxyID) == id());
  *groupMemberIDs = set<string>();
  for(unsigned int i = 0 ; i < details.groupMemberProxyIDs.length(); ++i) {
    groupMemberIDs->insert(string(details.groupMemberProxyIDs[i]));
  }
  return *groupMemberIDs;
}

const BindingFeatures::details::GroupDetails&
LocalBindingData::getGroupDetails() const
{
  if(!groupDetailsCache.get()) {
    groupDetailsCache = auto_ptr<CachedCASTData<BindingFeatures::details::GroupDetails> >
      (new CachedCASTData<BindingFeatures::details::GroupDetails>(dynamic_cast<cast::WorkingMemoryReaderProcess&>(bindingWMRepresenter),
								  getGroupDetailsID(),
								  shared_ptr<string>(new string(subarchitecture_id())))
       );
  }
  return **groupDetailsCache;
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
  std::auto_ptr<BindingData::Ambiguity> ambiguity;
  if(!shouldHaveAmbiguity()) {
    ambiguity = auto_ptr<BindingData::Ambiguity>(NULL);
    return NULL;
  }
  if(!bindingWMRepresenter().component().existsOnWorkingMemory(id()+ambiguityIDPostFix())) {
    ambiguity = auto_ptr<BindingData::Ambiguity>(NULL);
    return NULL;
  }
  try {
    shared_ptr<const BindingData::Ambiguity> amb =
      bindingWMRepresenter().loadBindingDataFromWM<BindingData::Ambiguity>(id()+ambiguityIDPostFix());
    ambiguity =
      auto_ptr<BindingData::Ambiguity>(new BindingData::Ambiguity(*amb));
  } catch (const DoesNotExistOnWMException&) {
    ambiguity = auto_ptr<BindingData::Ambiguity>(NULL);
    return NULL;
  }
  assert(ambiguity.get());
  return ambiguity.get();
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

