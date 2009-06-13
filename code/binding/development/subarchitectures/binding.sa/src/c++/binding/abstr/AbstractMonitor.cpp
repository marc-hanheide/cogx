#include <balt/interface/BALTTimer.hpp>
#include <boost/lexical_cast.hpp>
#include "AbstractMonitor.hpp"
#include "cast/architecture/ChangeFilterFactory.hpp"
#include "binding/utils/BindingScoreUtils.hpp"
#include "binding/ontology/BindingFeatureOntology.hpp"
#include "binding/feature-specialization/helpers/SalienceHelper.hpp"
#include <BindingFeaturesCommon.hpp>
#include <boost/regex.hpp>
#include <boost/foreach.hpp>
#ifndef foreach
#define foreach BOOST_FOREACH
#endif //foreach

namespace Binding {
using namespace boost;
using namespace std;
using namespace cast;
using namespace cast::cdl;
using namespace BindingFeaturesCommon;
using namespace BindingFeatures;


AbstractMonitor::AbstractMonitor(const string& _id) 
  : WorkingMemoryAttachedComponent(_id),
    AbstractBinder(_id),
    currentlyBuiltProxy(NULL),
    binderReady(false),
    startCalled(false)
{
  queueBehaviour = cdl::QUEUE;
}


void 
AbstractMonitor::configure(map<string,string> & _config) {

  // first let the base class configure itself
  AbstractBinder::configure(_config); // sets bindingSA

  bindingSA = bindingSubarchID();
}

void
AbstractMonitor::start() {
  PrivilegedManagedProcess::start();
  startCalled = true;
  addChangeFilter(createGlobalTypeFilter<BindingData::BinderToken>(cdl::ADD),
		  new MemberFunctionChangeReceiver<AbstractMonitor>(this,
								    &AbstractMonitor::binderReadySignal));
  addChangeFilter(createGlobalTypeFilter<BindingQueries::MakeProxyUnavailable>(cdl::ADD),
		  new MemberFunctionChangeReceiver<AbstractMonitor>(this,
								    &AbstractMonitor::makeProxyUnavailableAdded));
}


void
AbstractMonitor::binderReadySignal(const cdl::WorkingMemoryChange & _wmc)
{
  binderReady = true;
}

void 
AbstractMonitor::startNewGroupProxy(unsigned int _groupSize) {
  _startNewProxy(BindingData::GROUP);
  BindingFeatures::Group pl;
  pl.size = _groupSize;
  pl.groupDetailsID = CORBA::string_dup(newDataID().c_str());
  BindingData::FeaturePointer ptr = addFeatureToCurrentProxy(pl);
  BindingFeatures::details::GroupDetails* groupdetails = new BindingFeatures::details::GroupDetails;
  groupdetails->groupFeatureID = CORBA::string_dup(string(ptr.address).c_str());
  groupdetails->groupProxyID = CORBA::string_dup(currentProxyID.c_str());
  addToWorkingMemory(string(pl.groupDetailsID), 
		     getBindingSA(),
		     groupdetails,
		     BLOCKING);
}

void 
AbstractMonitor::makeCurrentProxyHypothetical() {
  if (updatingProxy) {
    throw BindingException("An existing proxy can not be made hypothetical");
  }
  currentlyBuiltProxy->hypothetical = true;
}


void 
AbstractMonitor::_startNewProxy(const BindingData::BindingProxyType _proxyType, const string& _insistedID) {
  assert(startCalled);
  assert(binderReady);
  if(currentlyBuiltProxy.get() != NULL) {
    throw BindingException("A new proxy can not be started until the previous one is stored or cancelled.");
  }

  if(_insistedID == "") {
     currentProxyID = newDataID();
  } else {
    log("Warning, using insisted ID when starting proxy: " + _insistedID);
    currentProxyID = _insistedID;
  }
  
  updatingProxy = false;

  currentlyBuiltProxy = 
    std::auto_ptr<BindingData::BindingProxy>(new BindingData::BindingProxy());
  // make sure to eliminate nasty CORBA null pointer
  currentlyBuiltProxy->unionID = CORBA::string_dup("");
  // make sure to eliminate nasty CORBA null pointer
  
  string bestID(newDataID());
  BindingData::BestUnionsForProxy* 
    new_best(new BindingData::BestUnionsForProxy());
  new_best->proxyID = CORBA::string_dup(currentProxyID.c_str());
  new_best->unionIDs.length(0);
  new_best->score = defaultBindingScore();
  new_best->score.sticky = -1;
  new_best->proxyUpdatesWhenThisComputed = -1;
  
  addToWorkingMemory(bestID, 
		     bindingSA,
		     //BindingLocalOntology::BEST_UNIONS_FOR_PROXY_TYPE, 
		     new_best, 
		     BLOCKING);
  
  string nonmatchingID(newDataID());
  BindingData::NonMatchingUnions* 
    new_nonmatching(new BindingData::NonMatchingUnions());
  new_nonmatching->proxyID = CORBA::string_dup(currentProxyID.c_str());
  new_nonmatching->nonMatchingUnionIDs.length(0);
  addToWorkingMemory(nonmatchingID, 
		     bindingSA,
		     //BindingLocalOntology::NON_MATCHING_UNIONS_TYPE, 
		     new_nonmatching,
		     BLOCKING);

  currentlyBuiltProxy->proxyIDs.length(0);
  currentlyBuiltProxy->sourceID = CORBA::string_dup(subarchitectureID.c_str());
  currentlyBuiltProxy->bestUnionsForProxyID = CORBA::string_dup(bestID.c_str());
  currentlyBuiltProxy->nonMatchingUnionID = CORBA::string_dup(nonmatchingID.c_str());
  currentlyBuiltProxy->type = _proxyType;
  currentlyBuiltProxy->hypothetical = false;
  currentlyBuiltProxy->updates = 0;
  currentlyBuiltProxy->bindingCount = 0;

  string inPortsID = newDataID();
  
  BindingData::ProxyPorts* inPorts = new BindingData::ProxyPorts();
  inPorts->ownerProxyID = CORBA::string_dup(currentProxyID.c_str());
  
  addToWorkingMemory(inPortsID, 
		     bindingSA,
		     //BindingLocalOntology::PROXY_PORTS_TYPE,
		     inPorts,
		     BLOCKING);
  
  currentlyBuiltProxy->inPortsID = CORBA::string_dup(inPortsID.c_str());

//     if(currentlyBuiltProxy->hypothetical) {
//       log("_startNewProxy, currentlyBuiltProxy is hypothetical");
//     }


}

void 
AbstractMonitor::changeExistingProxy(const string& _proxyAddr, const set<string>& _deleteTheseFeatureTypes)
{
  if(currentlyBuiltProxy.get() != NULL) {
    throw BindingException("An existing proxy can not be loaded until the previous one is stored or cancelled");
  }
  
  log("will acquire token for updating");
  if(!hasBindTheseProxiesMonitorToken()) {
    acquireBindTheseProxiesMonitorToken();
  }
//   if(!hasUpdateProxyMonitorToken()) {
//     acquireUpdateProxyMonitorToken();
//   }
  if(!hasBinderToken()) {
    acquireBinderToken();
  }

  log("acquired token for updating");
  
  updatingProxy = true;
  
  currentProxyID = _proxyAddr;
  
  shared_ptr<const BindingData::BindingProxy> 
    proxy(getWorkingMemoryEntry<BindingData::BindingProxy>(_proxyAddr,bindingSA)->getData());

  if(string(proxy->unionID) == "" && // i.e. unbound proxy
     unboundProxyAddresses.find(currentProxyID) != unboundProxyAddresses.end()) {// i.e., it's been added to bindtheseproxies, or it been cancelled
    cerr << "ILLEGAL!!! Attempt to update a proxy while binder is working on it (or one that has been cancelled). ProxyID: " << currentProxyID << endl;
  
  }

  // create new binding proxy locally
  currentlyBuiltProxy = 
    std::auto_ptr<BindingData::BindingProxy>(new BindingData::BindingProxy(*proxy));

  // reset the features
  currentlyBuiltProxy->proxyFeatures.length(0);
  currentlyBuiltProxy->updates = proxy->updates + 1;  
  
//   if(currentlyBuiltProxy->hypothetical) {
//     log("changeExistingProxy, proxy is hypothetical");
//   }

  const BindingFeatureOntology& ont(BindingFeatureOntology::construct());
  
  // then, store the existing features for deletion and copy features
  // we want to delete
  for(unsigned int i = 0 ; i < proxy->proxyFeatures.length() ; ++i ) {
    if(_deleteTheseFeatureTypes.find(string(proxy->proxyFeatures[i].type)) 
       != _deleteTheseFeatureTypes.end()) {
      featuresToDelete.insert(proxy->proxyFeatures[i]);
    } else { // if not deleted, then we need to add it to the new proxy
      currentlyBuiltProxy->proxyFeatures.length(currentlyBuiltProxy->proxyFeatures.length() + 1); 
      currentlyBuiltProxy->proxyFeatures[currentlyBuiltProxy->proxyFeatures.length() - 1] = 
	proxy->proxyFeatures[i];
    }
  }
  for(unsigned int i = 0 ; i < proxy->proxyFeatures.length() ; ++i ) {
    if(string(proxy->proxyFeatures[i].type) == ont.featureName(typeid(BindingFeatures::SourceID))) { //BindingFeatureOntology::SOURCE_ID_TYPE
      sourceIDAddress = proxy->proxyFeatures[i].address;
    }
  }
}

void
AbstractMonitor::cancelCurrentProxy()
{
  if(currentlyBuiltProxy.get() == NULL) 
    return;

  FeaturePointerSet delete_these;

  updatingProxy = false;
  currentProxyID = "";

  for(unsigned int i = 0 ; 
      i < currentlyBuiltProxy->proxyFeatures.length() ; 
      ++i ) {
    delete_these.insert(currentlyBuiltProxy->proxyFeatures[i]);
  }
  
  _deleteFeatures(delete_these);

  currentlyBuiltProxy = std::auto_ptr<BindingData::BindingProxy>(NULL);
  sourceIDAddress = "";
}


BindingData::FeaturePointer
AbstractMonitor::addSourceIDToCurrentProxy() {
  BindingFeatures::SourceID sourceID;
  if(sourceID == "") { // locally stored source ID
    throw BindingException("AbstractMonitor::sourceID not initialized. This should be done in the implemented monitor.");
  }
  sourceID.sourceID = CORBA::string_dup(sourceID.c_str());
  sourceID.monitorID = CORBA::string_dup(getProcessIdentifier().c_str());
  return addFeatureToCurrentProxy(sourceID);
}

BindingData::FeaturePointer 
AbstractMonitor::addOtherSourceIDToCurrentProxy(string _id, BindingFeaturesCommon::TruthValue _truthValue)
{
  BindingFeatures::OtherSourceID otherSourceID;
  otherSourceID.otherSourceID = CORBA::string_dup(_id.c_str());
  BindingData::FeaturePointer ret = addFeatureToCurrentProxy(otherSourceID,_truthValue);
  return ret;
}


BindingData::FeaturePointer
AbstractMonitor::addCreationTimeToCurrentProxy() 
{
  BindingFeatures::CreationTime creationTime;
  creationTime.creationTime = BALTTimer::getBALTTime();
  return addFeatureToCurrentProxy(creationTime);
}

string 
AbstractMonitor::storeCurrentProxy(bool _storeSystemFeatures) 	
{

  if(_storeSystemFeatures && !updatingProxy) {
    // 1st, store the SourceID since it's required. The inherited
    // method from AbstractMonitor can be used.
    addSourceIDToCurrentProxy().address;
    
    // Another simple thing to add is the creation time. Very
    // useful for debugging. Could be used to eliminate "old"
    // proxies too.
    addCreationTimeToCurrentProxy();
  }
  //log("AbstractMonitor::storeCurrentProxy");
  
  if(!updatingProxy) {
    //log("AbstractMonitor::storeCurrentProxy adding ThisProxyID");

    BindingFeatures::ThisProxyID thisProxyID;
    thisProxyID.thisProxyID = CORBA::string_dup(currentProxyID.c_str());
    addFeatureToCurrentProxy(thisProxyID);
  }


  //log("AbstractMonitor::storeCurrentProxy storing on wm");
  // store proxy on WM
  BindingData::BindingProxy* proxy_ptr = 
    new BindingData::BindingProxy(*currentlyBuiltProxy);
  
//   if(proxy_ptr->hypothetical) {
//     log("storeCurrentProxy, proxy_ptr is hypothetical");
//   }

  set<string> featureIDs;

  for(unsigned int i = 0; i < proxy_ptr->proxyFeatures.length() ; ++i) {
    featureIDs.insert(string(proxy_ptr->proxyFeatures[i].address));
  }
  string signature;
  foreach(string str, featureIDs) {
    signature += str;
  }
  proxy_ptr->featureSignature = CORBA::string_dup(signature.c_str());
  if(!hasBindTheseProxiesMonitorToken())
    acquireBindTheseProxiesMonitorToken();
  if(!hasBinderToken())
    acquireBinderToken();
  if(!updatingProxy) {
    proxy_ptr->proxyState = BindingData::NEW;
    addToWorkingMemory(currentProxyID, 
		       bindingSA(), 
		       //BindingLocalOntology::BINDING_PROXY_TYPE, 
		       proxy_ptr, 
		       BLOCKING);
  } else {
    
    bool overwritten = false;
    while(!overwritten) {
      try {
	proxy_ptr->proxyState = BindingData::UPDATED;
	overwriteWorkingMemory(currentProxyID, 
			       bindingSA(), 
			       //BindingLocalOntology::BINDING_PROXY_TYPE, 
			       proxy_ptr, 
			       BLOCKING);
	overwritten = true; // only reach if there is no exception
      } catch(const ConsistencyException _e) {
	getWorkingMemoryEntry<BindingData::BindingProxy>(currentProxyID,bindingSA());
      }
    }
  }
  // Now set the inports of the proxies that are related (if any)
  if(currentlyBuiltProxy->outPorts.ports.length() > 0)
    updateInports(currentlyBuiltProxy->outPorts);
  
  //log(string("Current proxy feature count: ") + lexical_cast<string>(currentlyBuiltProxy->proxyFeatures.length()));
  
  currentlyBuiltProxy = std::auto_ptr<BindingData::BindingProxy>(NULL);
  
  unboundProxyAddresses.insert(currentProxyID);
    
  string addr = currentProxyID;

  currentProxyID = "";

#warning testing to call bindNewProxies everytime an updated proxy is stored
//#warning NO... now call bindNewProxies everytime any proxy is stored
  if(updatingProxy) {
    bindNewProxies();
  }

  return addr;
}

string 
AbstractMonitor::addSimpleRelation(const string& _from, 
				   const string& _to, 
				   const string& _label) 
{
  assert(currentlyBuiltProxy.get() == NULL);
  
  // Now, try to add relation as a proxy too:
  startNewRelationProxy();
  RelationLabel label;
  label.label = CORBA::string_dup(_label.c_str());
  addFeatureToCurrentProxy(label);
  
  DebugString info;
  info.debugString = CORBA::string_dup(("obj1: " +  _from + " obj2: " + _to).c_str());
  addFeatureToCurrentProxy(info);
  
  addOutPortToCurrentProxy(_from, "from");
  addOutPortToCurrentProxy(_to, "to");
  string relationID = storeCurrentProxy();
  return relationID;
}


void 
AbstractMonitor::addOutPortToCurrentProxy(const std::string _proxyID, 
					  const std::string _portLabel)
{
  assert(currentlyBuiltProxy->type == BindingData::RELATION); // only allow outports from relations, for now...
  BindingData::ProxyPort port;
  port.type = BindingData::PROXY; // for now... ok...
  port.label = CORBA::string_dup(_portLabel.c_str());
  port.proxyID = CORBA::string_dup(_proxyID.c_str());
  port.ownerProxyID = CORBA::string_dup(currentProxyID.c_str());
  currentlyBuiltProxy->outPorts.ports.length(currentlyBuiltProxy->outPorts.ports.length() + 1);
  currentlyBuiltProxy->outPorts.ports[currentlyBuiltProxy->outPorts.ports.length() - 1] = port;
}
  


void 
AbstractMonitor::updateInports(const BindingData::ProxyPorts& _ports)
{
  // disabled updateInports, this is now done in the Binder
  //  return;
  //for(unsigned int i = 0 ; i < _ports.ports.length() ; ++i) {
  //  string proxyID(_ports.ports[i].proxyID);
  //  log("updating inports of " + proxyID);
  //  try {
  //    map<string,string>::const_iterator inportsID_itr(proxyID2inportsID.find(proxyID));
  //    string inportsID;
  //    if(inportsID_itr == proxyID2inportsID.end()) {
  //shared_ptr<const CASTData<BindingData::BindingProxy> > data_ptr(getWorkingMemoryEntry<BindingData::BindingProxy>(proxyID,bindingSA));
  //shared_ptr<const BindingData::BindingProxy> froproxy(data_ptr->getData());
  //inportsID = froproxy->inPortsID;
  //proxyID2inportsID[proxyID] = inportsID;
  //    } else {
  //inportsID = inportsID_itr->second;
  //    }
  //    log("inportsID: " + inportsID);
  //          
  //    //      lockEntry(inportsID, bindingSA, cast::cdl::LOCKED_ODR);
  //    shared_ptr<const CASTData<BindingData::ProxyPorts> > 
  //inportsData = 
  //getWorkingMemoryEntry<BindingData::ProxyPorts>(inportsID, bindingSA);
  //    shared_ptr<const BindingData::ProxyPorts> inports = inportsData->getData();
  //    BindingData::ProxyPorts* new_inports = new BindingData::ProxyPorts(*inports);
  //    
  //    //ProxyPort[] new_inports = new ProxyPort[inports.ports.length + 1];
  //    //System.arraycopy(inports.ports, 0, new_inports, 0, inports.ports.length);
  //    //new_inports[inports.ports.length] = new ProxyPort(_portType,"",_relationID);
  //    new_inports->ports.length(inports->ports.length() + 1);
  //    BindingData::ProxyPort new_port;
  //    new_port.type = _ports.ports[i].type;
  //    new_port.label = CORBA::string_dup(_ports.ports[i].label);
  //    new_port.proxyID = CORBA::string_dup(_ports.ports[i].ownerProxyID);
  //    //cout << "new_port.proxyID " << new_port.proxyID << endl;
  //    new_port.ownerProxyID = CORBA::string_dup(_ports.ports[i].proxyID);
  //    
  //    new_inports->ports[inports->ports.length()] = new_port;
  //    
  //    overwriteWorkingMemory(inportsID, 
  //		     bindingSA,
  //		     //BindingLocalOntology::PROXY_PORTS_TYPE,
  //		     new_inports,
  //		     BLOCKING);
  //    //insert the proxy to the unbound list.
  //    unboundProxyAddresses.insert(proxyID);
  //  } catch(const DoesNotExistOnWMException& _e) {
  //    log("could not update inports of " + proxyID + " : " + _e.what());
  //  }
  //}
}

void 
AbstractMonitor::bindNewProxies()
{
  if(unboundProxyAddresses.empty()) {
    log("AbstractMonitor::bindNewProxies: unboundProxyAddresses.empty()");
    return;
  }
  // token only for bindNewProxies
  if(!hasBindTheseProxiesMonitorToken())
    acquireBindTheseProxiesMonitorToken();
  // token for the whole architecture
  if(!hasBinderToken())
    acquireBinderToken();
  releaseBinderToken(); // give it back to binder right away
  try{
    BindingData::BindTheseProxies* bindThese = new BindingData::BindTheseProxies();
    bindThese->proxyIDs.length(unboundProxyAddresses.size());
    set<string>::const_iterator unbound_itr = unboundProxyAddresses.begin();
    for(unsigned int i = 0 ; i < bindThese->proxyIDs.length() ; ++i,++unbound_itr) {
      ownedProxyIDs.insert(*unbound_itr);
      bindThese->proxyIDs[i] = CORBA::string_dup(unbound_itr->c_str());
      //log("bindThese->proxyIDs[" +lexical_cast<string>(i)+ "]: " + string(bindThese->proxyIDs[i]));
    }
    //  log("bindThese.proxyIDs.length(): " + lexical_cast<string>(bindThese->proxyIDs.length()));
    addToWorkingMemory(newDataID(), 
		       bindingSA, 
		       bindThese, 
		       BLOCKING);
    unboundProxyAddresses.clear();
    
    // now delete any features that have been replaced/removed
    _deleteFeatures(featuresToDelete);
    featuresToDelete.clear();
    
    // make sure the token is released even at the event of an exception
  }
  catch(...) {
    if(hasBinderToken())
      releaseBinderToken();
    releaseBindTheseProxiesMonitorToken();
    if(hasUpdateProxyMonitorToken())
      releaseUpdateProxyMonitorToken();
    throw;
  }
  if(hasBinderToken())
    releaseBinderToken();
  releaseBindTheseProxiesMonitorToken();
  if(hasUpdateProxyMonitorToken()) {
    releaseUpdateProxyMonitorToken();
  }
}

void 
AbstractMonitor::deleteExistingProxy(const string& _proxyAddr) {

  set<string>::iterator owner_itr = ownedProxyIDs.find(_proxyAddr);
  assert(owner_itr != ownedProxyIDs.end()); // only delete what is owned by this proxy
  
  shared_ptr<const BindingData::BindingProxy> 
    proxy(getWorkingMemoryEntry<BindingData::BindingProxy>(_proxyAddr,bindingSA)->getData());

  //FeaturePointerSet delete_these;
  
  /*  for(unsigned int i = 0 ; i < proxy->proxyFeatures.length() ; ++i ) {
      delete_these.insert(proxy->proxyFeatures[i]);
      }*/

  // should be renamed:
  if(!hasBindTheseProxiesMonitorToken())
    acquireBindTheseProxiesMonitorToken();
  if(!hasBinderToken())
    acquireBinderToken();
  //releaseBinderToken(); // give it back to binder right away


  try{
    // 1st, delete proxy
    BindingData::BindingProxyDeletionTask* del_task = 
      new BindingData::BindingProxyDeletionTask();
    del_task->proxyID = CORBA::string_dup(_proxyAddr.c_str());
    
    string newID(newDataID());
    addToWorkingMemory(newID, 
		       bindingSA,
		       //BindingLocalOntology::BINDING_PROXY_DELETION_TASK_TYPE, 
		       del_task);
    
    ownedProxyIDs.erase(owner_itr);
    
    log("Proxy deletion task added for: " + _proxyAddr + " ( task ID: " + newID + ")");
    
    //deleteFromWorkingMemory(_proxyAddr);
    
    // 2nd, delete features, not any more... let someone else do it properly
    // _deleteFeatures(delete_these);
  }
  catch(...) {
    if(hasBinderToken())
      releaseBinderToken();
    if(hasBindTheseProxiesMonitorToken())
      releaseBindTheseProxiesMonitorToken();
    throw;
  }
  if(hasBinderToken())
    releaseBinderToken();
  if(hasBindTheseProxiesMonitorToken())
    releaseBindTheseProxiesMonitorToken();
};

void 
AbstractMonitor::_deleteFeatures(const FeaturePointerSet& features)
{
  for(FeaturePointerSet::const_iterator feat = features.begin(); 
      feat != features.end() ; 
      ++feat) {
    //log(string("Deleting feature task added for feature: " + string(feat->address)));
    //deleteFromWorkingMemory(string(feat->address));
    BindingData::ExplicitFeatureDeletionTask* del_task = 
      new BindingData::ExplicitFeatureDeletionTask();
    
    del_task->featureID = CORBA::string_dup(string(feat->address).c_str());
    
    assert(boost::regex_match(string(feat->address),boost::regex(".+:.+")));
    assert(string(feat->address) != "");
    
    addToWorkingMemory(newDataID(),
		       bindingSA, 
		       //BindingLocalOntology::BINDING_FEATURE_POINTER_DELETION_TASK_TYPE, 
		       del_task, 
		       cast::cdl::BLOCKING);
  }
}

void
AbstractMonitor::deleteFeatureFromCurrentProxy(const BindingData::FeaturePointer& _ptr)
{
  if(currentlyBuiltProxy.get() == NULL) {
    throw(BindingException("Attempt to delete feature from proxyFeatures before initiating it with startNewProxy"));
  }
  FeaturePointerSet deleted;
  unsigned int j = 0;
  for(unsigned int i = 0; i < currentlyBuiltProxy->proxyFeatures.length(); ++i) {
    if(string(currentlyBuiltProxy->proxyFeatures[i].address) == string(_ptr.address)) {
      assert(string(currentlyBuiltProxy->proxyFeatures[i].type) == string(_ptr.type));
      deleted.insert(_ptr);
    } else {
      currentlyBuiltProxy->proxyFeatures[j] = currentlyBuiltProxy->proxyFeatures[i];
      ++j;
    }
  }
  if(deleted.empty()) {
    throw(BindingException("Attempting to delete a feature that was not on the proxy"));
  }
  _deleteFeatures(deleted);
  currentlyBuiltProxy->proxyFeatures.length(currentlyBuiltProxy->proxyFeatures.length() - 1);
}


BindingData::FeaturePointer 
AbstractMonitor::addSalienceToCurrentProxy(const BindingFeaturesCommon::StartTime& _start,
					   const BindingFeaturesCommon::EndTime& _end)
{
  BindingFeatures::Salience salience;
  salience.start = _start;
  salience.end = _end;
  return addFeatureToCurrentProxy(salience);  
}


BindingData::FeaturePointer 
AbstractMonitor::addSalienceToCurrentProxy(const FrameworkBasics::BALTTime& _time) 
{ 
  return addSalienceToCurrentProxy(startTime(_time),endTime(_time));
}

BindingData::FeaturePointer 
AbstractMonitor::addSalienceToCurrentProxy()
{
  return addSalienceToCurrentProxy(BALTTimer::getBALTTime());
}

ParentFeature 
AbstractMonitor::defaultParentFeature() const {
  ParentFeature parent;
  parent.truthValue = BindingFeaturesCommon::POSITIVE;
  parent.immediateProxyID = CORBA::string_dup(currentProxyID.c_str());
  return parent;
}



void 
AbstractMonitor::acquireBindTheseProxiesMonitorToken() 
{
  acquireToken(_bindTheseProxiesMonitorTokenAddress());
}
void 
AbstractMonitor::releaseBindTheseProxiesMonitorToken() 
{
  assert(!hasBinderToken());
  //  acquireBinderTokenToken();
  acquireBinderLockToken(); // the next parts must be atomic
  releaseToken(_bindTheseProxiesMonitorTokenAddress());
  // releaseBinderTokenToken();
  releaseBinderLockToken();
}
  

cast::cdl::WorkingMemoryAddress 
AbstractMonitor::_bindTheseProxiesMonitorTokenAddress() const {
  cast::cdl::WorkingMemoryAddress a;
  a.subarchitecture = CORBA::string_dup(getBindingSA().c_str());
  a.id = CORBA::string_dup(BindingData::bindTheseProxiesMonitorTokenID);
  return a;  
}

void 
AbstractMonitor::acquireUpdateProxyMonitorToken() 
{
  acquireToken(_updateProxyMonitorTokenAddress());
}

void 
AbstractMonitor::releaseUpdateProxyMonitorToken() 
{
  releaseToken(_updateProxyMonitorTokenAddress());
}

cast::cdl::WorkingMemoryAddress 
AbstractMonitor::_updateProxyMonitorTokenAddress() const {
  cast::cdl::WorkingMemoryAddress a;
  a.subarchitecture = CORBA::string_dup(getBindingSA().c_str());
  a.id = CORBA::string_dup(BindingData::updateProxyMonitorTokenID);
  return a;
}

/*void 
AbstractMonitor::makeProxiesUnavailableAdded(const cast::cdl::WorkingMemoryChange & _wmc)
{
  try{
    try{
      // mutex the entry first:
      lockEntry(_wmc.address,cast::cdl::LOCKED_ODR);
    } catch(const DoesNotExistOnWMException _e) { // ok, it was removed before this monitor could touch it
      return;
    }
    shared_ptr<const BindingQueries::MakeProxiesUnavailable> unavail = loadBindingDataFromWM<BindingQueries::MakeProxiesUnavailable>(_wmc);
    vector<BindingQueries::MakeProxyUnavailable> remaining;
    for(unsigned int i = 0; i < unavail->proxies.length() ; ++i) {
      if(string(unavail->proxies[i].ownerMonitorID) == getProcessIdentifier()) // or proxy should be deleted, aha!
	try{
	  makeProxyUnavailable(unavail->proxies[i]);
	} catch(DoesNotExistOnWMException _e) {
	  if(string(_e.address().id) == string(unavail->proxies[i].proxyID)) // ah, it was our proxy that was gone... it was already deleted, no probs
	    ; // noop
	  else // not our proxy, it was sth else that was rotten
	    throw(_e);
	}
      else
	remaining.push_back(unavail->proxies[i]);
    }
    if(remaining.size() == 0) { // all job is done
      deleteFromWorkingMemory(_wmc.address);
      return;
    }
    if(remaining.size() != unavail->proxies.length()) { // i.e. sth was processed
      BindingQueries::MakeProxiesUnavailable* new_unavail = new BindingQueries::MakeProxiesUnavailable();
      new_unavail->proxies.length(remaining.size());
      unsigned int j = 0;
      foreach(BindingQueries::MakeProxyUnavailable unav, remaining) {
	new_unavail->proxies[j++] = unav;
      }
      overwriteWorkingMemory(_wmc.address,new_unavail,cast::cdl::BLOCKING);
    }
  } catch(...) { // just in case
    unlockEntry(_wmc.address);
    throw;
  }
  unlockEntry(_wmc.address);
}
*/

void
AbstractMonitor::makeProxyUnavailableAdded(const cast::cdl::WorkingMemoryChange & _wmc)
{
  log("makeProxyUnavailableAdded");

  shared_ptr<const BindingQueries::MakeProxyUnavailable> unavail;
  try{
    unavail = loadBindingDataFromWM<BindingQueries::MakeProxyUnavailable>(_wmc);
  } catch(DoesNotExistOnWMException& _e) {
    // noop. this is ok... it was someone else's struct
    return;
  }
  string proxyID(unavail->proxyID);
  if(ownedProxyIDs.find(proxyID) == ownedProxyIDs.end()) // not my proxy, ignore
    return;
  // ok, it is my proxy, deal with it:
  // 1st, delete the entry...
  deleteFromWorkingMemory(_wmc.address);
  // 2nd, deal with the entry
  try{
    makeProxyUnavailable(*unavail);
  } catch(DoesNotExistOnWMException _e) {
    if(strcmp(_e.address().id,proxyID.c_str()) == 0) // ah, it was our proxy that was gone... it was already deleted, no probs
      /*noop*/;
    else // not our proxy, it was sth else that was rotten
      throw(_e);
  }
}

void 
AbstractMonitor::makeProxyUnavailable(const BindingQueries::MakeProxyUnavailable& _makeProxyUnavailable)
{
  deleteExistingProxy(string(_makeProxyUnavailable.proxyID));
}

} // namespace Binding
