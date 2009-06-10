#include <balt/interface/BALTTimer.hpp>
#include <boost/lexical_cast.hpp>
#include "AbstractMonitor.hpp"
#include "cast/architecture/ChangeFilterFactory.hpp"
#include "binding/utils/BindingScoreUtils.hpp"
#include "binding/ontology/BindingFeatureOntology.hpp"
#include "binding/feature-specialization/helpers/SalienceHelper.hpp"
#include <binding/idl/BindingFeaturesCommon.hh>
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
    m_currentlyBuiltProxy(NULL),
    m_binderReady(false),
    m_startCalled(false)
{
  m_queueBehaviour = cdl::QUEUE;
}


void 
AbstractMonitor::configure(map<string,string> & _config) {

  // first let the base class configure itself
  AbstractBinder::configure(_config); // sets m_bindingSA

  m_bindingSA = bindingSubarchID();
}

void
AbstractMonitor::start() {
  PrivilegedManagedProcess::start();
  m_startCalled = true;
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
  m_binderReady = true;
}

void 
AbstractMonitor::startNewGroupProxy(unsigned int _groupSize) {
  _startNewProxy(BindingData::GROUP);
  BindingFeatures::Group pl;
  pl.m_size = _groupSize;
  pl.m_groupDetailsID = CORBA::string_dup(newDataID().c_str());
  BindingData::FeaturePointer ptr = addFeatureToCurrentProxy(pl);
  BindingFeatures::details::GroupDetails* groupdetails = new BindingFeatures::details::GroupDetails;
  groupdetails->m_groupFeatureID = CORBA::string_dup(string(ptr.m_address).c_str());
  groupdetails->m_groupProxyID = CORBA::string_dup(m_currentProxyID.c_str());
  addToWorkingMemory(string(pl.m_groupDetailsID), 
		     getBindingSA(),
		     groupdetails,
		     BLOCKING);
}

void 
AbstractMonitor::makeCurrentProxyHypothetical() {
  if (m_updatingProxy) {
    throw BindingException("An existing proxy can not be made hypothetical");
  }
  m_currentlyBuiltProxy->m_hypothetical = true;
}


void 
AbstractMonitor::_startNewProxy(const BindingData::BindingProxyType _proxyType, const string& _insistedID) {
  assert(m_startCalled);
  assert(m_binderReady);
  if(m_currentlyBuiltProxy.get() != NULL) {
    throw BindingException("A new proxy can not be started until the previous one is stored or cancelled.");
  }

  if(_insistedID == "") {
     m_currentProxyID = newDataID();
  } else {
    log("Warning, using insisted ID when starting proxy: " + _insistedID);
    m_currentProxyID = _insistedID;
  }
  
  m_updatingProxy = false;

  m_currentlyBuiltProxy = 
    std::auto_ptr<BindingData::BindingProxy>(new BindingData::BindingProxy());
  // make sure to eliminate nasty CORBA null pointer
  m_currentlyBuiltProxy->m_unionID = CORBA::string_dup("");
  // make sure to eliminate nasty CORBA null pointer
  
  string bestID(newDataID());
  BindingData::BestUnionsForProxy* 
    new_best(new BindingData::BestUnionsForProxy());
  new_best->m_proxyID = CORBA::string_dup(m_currentProxyID.c_str());
  new_best->m_unionIDs.length(0);
  new_best->m_score = defaultBindingScore();
  new_best->m_score.m_sticky = -1;
  new_best->m_proxyUpdatesWhenThisComputed = -1;
  
  addToWorkingMemory(bestID, 
		     m_bindingSA,
		     //BindingLocalOntology::BEST_UNIONS_FOR_PROXY_TYPE, 
		     new_best, 
		     BLOCKING);
  
  string nonmatchingID(newDataID());
  BindingData::NonMatchingUnions* 
    new_nonmatching(new BindingData::NonMatchingUnions());
  new_nonmatching->m_proxyID = CORBA::string_dup(m_currentProxyID.c_str());
  new_nonmatching->m_nonMatchingUnionIDs.length(0);
  addToWorkingMemory(nonmatchingID, 
		     m_bindingSA,
		     //BindingLocalOntology::NON_MATCHING_UNIONS_TYPE, 
		     new_nonmatching,
		     BLOCKING);

  m_currentlyBuiltProxy->m_proxyIDs.length(0);
  m_currentlyBuiltProxy->m_sourceID = CORBA::string_dup(m_subarchitectureID.c_str());
  m_currentlyBuiltProxy->m_bestUnionsForProxyID = CORBA::string_dup(bestID.c_str());
  m_currentlyBuiltProxy->m_nonMatchingUnionID = CORBA::string_dup(nonmatchingID.c_str());
  m_currentlyBuiltProxy->m_type = _proxyType;
  m_currentlyBuiltProxy->m_hypothetical = false;
  m_currentlyBuiltProxy->m_updates = 0;
  m_currentlyBuiltProxy->m_bindingCount = 0;

  string inPortsID = newDataID();
  
  BindingData::ProxyPorts* inPorts = new BindingData::ProxyPorts();
  inPorts->m_ownerProxyID = CORBA::string_dup(m_currentProxyID.c_str());
  
  addToWorkingMemory(inPortsID, 
		     m_bindingSA,
		     //BindingLocalOntology::PROXY_PORTS_TYPE,
		     inPorts,
		     BLOCKING);
  
  m_currentlyBuiltProxy->m_inPortsID = CORBA::string_dup(inPortsID.c_str());

//     if(m_currentlyBuiltProxy->m_hypothetical) {
//       log("_startNewProxy, m_currentlyBuiltProxy is hypothetical");
//     }


}

void 
AbstractMonitor::changeExistingProxy(const string& _proxyAddr, const set<string>& _deleteTheseFeatureTypes)
{
  if(m_currentlyBuiltProxy.get() != NULL) {
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
  
  m_updatingProxy = true;
  
  m_currentProxyID = _proxyAddr;
  
  shared_ptr<const BindingData::BindingProxy> 
    proxy(getWorkingMemoryEntry<BindingData::BindingProxy>(_proxyAddr,m_bindingSA)->getData());

  if(string(proxy->m_unionID) == "" && // i.e. unbound proxy
     m_unboundProxyAddresses.find(m_currentProxyID) != m_unboundProxyAddresses.end()) {// i.e., it's been added to bindtheseproxies, or it been cancelled
    cerr << "ILLEGAL!!! Attempt to update a proxy while binder is working on it (or one that has been cancelled). ProxyID: " << m_currentProxyID << endl;
  
  }

  // create new binding proxy locally
  m_currentlyBuiltProxy = 
    std::auto_ptr<BindingData::BindingProxy>(new BindingData::BindingProxy(*proxy));

  // reset the features
  m_currentlyBuiltProxy->m_proxyFeatures.length(0);
  m_currentlyBuiltProxy->m_updates = proxy->m_updates + 1;  
  
//   if(m_currentlyBuiltProxy->m_hypothetical) {
//     log("changeExistingProxy, proxy is hypothetical");
//   }

  const BindingFeatureOntology& ont(BindingFeatureOntology::construct());
  
  // then, store the existing features for deletion and copy features
  // we want to delete
  for(unsigned int i = 0 ; i < proxy->m_proxyFeatures.length() ; ++i ) {
    if(_deleteTheseFeatureTypes.find(string(proxy->m_proxyFeatures[i].m_type)) 
       != _deleteTheseFeatureTypes.end()) {
      m_featuresToDelete.insert(proxy->m_proxyFeatures[i]);
    } else { // if not deleted, then we need to add it to the new proxy
      m_currentlyBuiltProxy->m_proxyFeatures.length(m_currentlyBuiltProxy->m_proxyFeatures.length() + 1); 
      m_currentlyBuiltProxy->m_proxyFeatures[m_currentlyBuiltProxy->m_proxyFeatures.length() - 1] = 
	proxy->m_proxyFeatures[i];
    }
  }
  for(unsigned int i = 0 ; i < proxy->m_proxyFeatures.length() ; ++i ) {
    if(string(proxy->m_proxyFeatures[i].m_type) == ont.featureName(typeid(BindingFeatures::SourceID))) { //BindingFeatureOntology::SOURCE_ID_TYPE
      m_sourceIDAddress = proxy->m_proxyFeatures[i].m_address;
    }
  }
}

void
AbstractMonitor::cancelCurrentProxy()
{
  if(m_currentlyBuiltProxy.get() == NULL) 
    return;

  FeaturePointerSet delete_these;

  m_updatingProxy = false;
  m_currentProxyID = "";

  for(unsigned int i = 0 ; 
      i < m_currentlyBuiltProxy->m_proxyFeatures.length() ; 
      ++i ) {
    delete_these.insert(m_currentlyBuiltProxy->m_proxyFeatures[i]);
  }
  
  _deleteFeatures(delete_these);

  m_currentlyBuiltProxy = std::auto_ptr<BindingData::BindingProxy>(NULL);
  m_sourceIDAddress = "";
}


BindingData::FeaturePointer
AbstractMonitor::addSourceIDToCurrentProxy() {
  BindingFeatures::SourceID sourceID;
  if(m_sourceID == "") { // locally stored source ID
    throw BindingException("AbstractMonitor::m_sourceID not initialized. This should be done in the implemented monitor.");
  }
  sourceID.m_sourceID = CORBA::string_dup(m_sourceID.c_str());
  sourceID.m_monitorID = CORBA::string_dup(getProcessIdentifier().c_str());
  return addFeatureToCurrentProxy(sourceID);
}

BindingData::FeaturePointer 
AbstractMonitor::addOtherSourceIDToCurrentProxy(string _id, BindingFeaturesCommon::TruthValue _truthValue)
{
  BindingFeatures::OtherSourceID otherSourceID;
  otherSourceID.m_otherSourceID = CORBA::string_dup(_id.c_str());
  BindingData::FeaturePointer ret = addFeatureToCurrentProxy(otherSourceID,_truthValue);
  return ret;
}


BindingData::FeaturePointer
AbstractMonitor::addCreationTimeToCurrentProxy() 
{
  BindingFeatures::CreationTime creationTime;
  creationTime.m_creationTime = BALTTimer::getBALTTime();
  return addFeatureToCurrentProxy(creationTime);
}

string 
AbstractMonitor::storeCurrentProxy(bool _storeSystemFeatures) 	
{

  if(_storeSystemFeatures && !m_updatingProxy) {
    // 1st, store the SourceID since it's required. The inherited
    // method from AbstractMonitor can be used.
    addSourceIDToCurrentProxy().m_address;
    
    // Another simple thing to add is the creation time. Very
    // useful for debugging. Could be used to eliminate "old"
    // proxies too.
    addCreationTimeToCurrentProxy();
  }
  //log("AbstractMonitor::storeCurrentProxy");
  
  if(!m_updatingProxy) {
    //log("AbstractMonitor::storeCurrentProxy adding ThisProxyID");

    BindingFeatures::ThisProxyID thisProxyID;
    thisProxyID.m_thisProxyID = CORBA::string_dup(m_currentProxyID.c_str());
    addFeatureToCurrentProxy(thisProxyID);
  }


  //log("AbstractMonitor::storeCurrentProxy storing on wm");
  // store proxy on WM
  BindingData::BindingProxy* proxy_ptr = 
    new BindingData::BindingProxy(*m_currentlyBuiltProxy);
  
//   if(proxy_ptr->m_hypothetical) {
//     log("storeCurrentProxy, proxy_ptr is hypothetical");
//   }

  set<string> featureIDs;

  for(unsigned int i = 0; i < proxy_ptr->m_proxyFeatures.length() ; ++i) {
    featureIDs.insert(string(proxy_ptr->m_proxyFeatures[i].m_address));
  }
  string signature;
  foreach(string str, featureIDs) {
    signature += str;
  }
  proxy_ptr->m_featureSignature = CORBA::string_dup(signature.c_str());
  if(!hasBindTheseProxiesMonitorToken())
    acquireBindTheseProxiesMonitorToken();
  if(!hasBinderToken())
    acquireBinderToken();
  if(!m_updatingProxy) {
    proxy_ptr->m_proxyState = BindingData::NEW;
    addToWorkingMemory(m_currentProxyID, 
		       bindingSA(), 
		       //BindingLocalOntology::BINDING_PROXY_TYPE, 
		       proxy_ptr, 
		       BLOCKING);
  } else {
    
    bool overwritten = false;
    while(!overwritten) {
      try {
	proxy_ptr->m_proxyState = BindingData::UPDATED;
	overwriteWorkingMemory(m_currentProxyID, 
			       bindingSA(), 
			       //BindingLocalOntology::BINDING_PROXY_TYPE, 
			       proxy_ptr, 
			       BLOCKING);
	overwritten = true; // only reach if there is no exception
      } catch(const ConsistencyException _e) {
	getWorkingMemoryEntry<BindingData::BindingProxy>(m_currentProxyID,bindingSA());
      }
    }
  }
  // Now set the inports of the proxies that are related (if any)
  if(m_currentlyBuiltProxy->m_outPorts.m_ports.length() > 0)
    updateInports(m_currentlyBuiltProxy->m_outPorts);
  
  //log(string("Current proxy feature count: ") + lexical_cast<string>(m_currentlyBuiltProxy->m_proxyFeatures.length()));
  
  m_currentlyBuiltProxy = std::auto_ptr<BindingData::BindingProxy>(NULL);
  
  m_unboundProxyAddresses.insert(m_currentProxyID);
    
  string addr = m_currentProxyID;

  m_currentProxyID = "";

#warning testing to call bindNewProxies everytime an updated proxy is stored
//#warning NO... now call bindNewProxies everytime any proxy is stored
  if(m_updatingProxy) {
    bindNewProxies();
  }

  return addr;
}

string 
AbstractMonitor::addSimpleRelation(const string& _from, 
				   const string& _to, 
				   const string& _label) 
{
  assert(m_currentlyBuiltProxy.get() == NULL);
  
  // Now, try to add relation as a proxy too:
  startNewRelationProxy();
  RelationLabel label;
  label.m_label = CORBA::string_dup(_label.c_str());
  addFeatureToCurrentProxy(label);
  
  DebugString info;
  info.m_debugString = CORBA::string_dup(("obj1: " +  _from + " obj2: " + _to).c_str());
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
  assert(m_currentlyBuiltProxy->m_type == BindingData::RELATION); // only allow outports from relations, for now...
  BindingData::ProxyPort port;
  port.m_type = BindingData::PROXY; // for now... ok...
  port.m_label = CORBA::string_dup(_portLabel.c_str());
  port.m_proxyID = CORBA::string_dup(_proxyID.c_str());
  port.m_ownerProxyID = CORBA::string_dup(m_currentProxyID.c_str());
  m_currentlyBuiltProxy->m_outPorts.m_ports.length(m_currentlyBuiltProxy->m_outPorts.m_ports.length() + 1);
  m_currentlyBuiltProxy->m_outPorts.m_ports[m_currentlyBuiltProxy->m_outPorts.m_ports.length() - 1] = port;
}
  


void 
AbstractMonitor::updateInports(const BindingData::ProxyPorts& _ports)
{
  // disabled updateInports, this is now done in the Binder
  //  return;
  //for(unsigned int i = 0 ; i < _ports.m_ports.length() ; ++i) {
  //  string proxyID(_ports.m_ports[i].m_proxyID);
  //  log("updating inports of " + proxyID);
  //  try {
  //    map<string,string>::const_iterator inportsID_itr(m_proxyID2inportsID.find(proxyID));
  //    string inportsID;
  //    if(inportsID_itr == m_proxyID2inportsID.end()) {
  //shared_ptr<const CASTData<BindingData::BindingProxy> > data_ptr(getWorkingMemoryEntry<BindingData::BindingProxy>(proxyID,m_bindingSA));
  //shared_ptr<const BindingData::BindingProxy> from_proxy(data_ptr->getData());
  //inportsID = from_proxy->m_inPortsID;
  //m_proxyID2inportsID[proxyID] = inportsID;
  //    } else {
  //inportsID = inportsID_itr->second;
  //    }
  //    log("inportsID: " + inportsID);
  //          
  //    //      lockEntry(inportsID, m_bindingSA, cast::cdl::LOCKED_ODR);
  //    shared_ptr<const CASTData<BindingData::ProxyPorts> > 
  //inportsData = 
  //getWorkingMemoryEntry<BindingData::ProxyPorts>(inportsID, m_bindingSA);
  //    shared_ptr<const BindingData::ProxyPorts> inports = inportsData->getData();
  //    BindingData::ProxyPorts* new_inports = new BindingData::ProxyPorts(*inports);
  //    
  //    //ProxyPort[] new_inports = new ProxyPort[inports.m_ports.length + 1];
  //    //System.arraycopy(inports.m_ports, 0, new_inports, 0, inports.m_ports.length);
  //    //new_inports[inports.m_ports.length] = new ProxyPort(_portType,"",_relationID);
  //    new_inports->m_ports.length(inports->m_ports.length() + 1);
  //    BindingData::ProxyPort new_port;
  //    new_port.m_type = _ports.m_ports[i].m_type;
  //    new_port.m_label = CORBA::string_dup(_ports.m_ports[i].m_label);
  //    new_port.m_proxyID = CORBA::string_dup(_ports.m_ports[i].m_ownerProxyID);
  //    //cout << "new_port.m_proxyID " << new_port.m_proxyID << endl;
  //    new_port.m_ownerProxyID = CORBA::string_dup(_ports.m_ports[i].m_proxyID);
  //    
  //    new_inports->m_ports[inports->m_ports.length()] = new_port;
  //    
  //    overwriteWorkingMemory(inportsID, 
  //		     m_bindingSA,
  //		     //BindingLocalOntology::PROXY_PORTS_TYPE,
  //		     new_inports,
  //		     BLOCKING);
  //    //insert the proxy to the unbound list.
  //    m_unboundProxyAddresses.insert(proxyID);
  //  } catch(const DoesNotExistOnWMException& _e) {
  //    log("could not update inports of " + proxyID + " : " + _e.what());
  //  }
  //}
}

void 
AbstractMonitor::bindNewProxies()
{
  if(m_unboundProxyAddresses.empty()) {
    log("AbstractMonitor::bindNewProxies: m_unboundProxyAddresses.empty()");
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
    bindThese->m_proxyIDs.length(m_unboundProxyAddresses.size());
    set<string>::const_iterator unbound_itr = m_unboundProxyAddresses.begin();
    for(unsigned int i = 0 ; i < bindThese->m_proxyIDs.length() ; ++i,++unbound_itr) {
      m_ownedProxyIDs.insert(*unbound_itr);
      bindThese->m_proxyIDs[i] = CORBA::string_dup(unbound_itr->c_str());
      //log("bindThese->m_proxyIDs[" +lexical_cast<string>(i)+ "]: " + string(bindThese->m_proxyIDs[i]));
    }
    //  log("bindThese.m_proxyIDs.length(): " + lexical_cast<string>(bindThese->m_proxyIDs.length()));
    addToWorkingMemory(newDataID(), 
		       m_bindingSA, 
		       bindThese, 
		       BLOCKING);
    m_unboundProxyAddresses.clear();
    
    // now delete any features that have been replaced/removed
    _deleteFeatures(m_featuresToDelete);
    m_featuresToDelete.clear();
    
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

  set<string>::iterator owner_itr = m_ownedProxyIDs.find(_proxyAddr);
  assert(owner_itr != m_ownedProxyIDs.end()); // only delete what is owned by this proxy
  
  shared_ptr<const BindingData::BindingProxy> 
    proxy(getWorkingMemoryEntry<BindingData::BindingProxy>(_proxyAddr,m_bindingSA)->getData());

  //FeaturePointerSet delete_these;
  
  /*  for(unsigned int i = 0 ; i < proxy->m_proxyFeatures.length() ; ++i ) {
      delete_these.insert(proxy->m_proxyFeatures[i]);
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
    del_task->m_proxyID = CORBA::string_dup(_proxyAddr.c_str());
    
    string newID(newDataID());
    addToWorkingMemory(newID, 
		       m_bindingSA,
		       //BindingLocalOntology::BINDING_PROXY_DELETION_TASK_TYPE, 
		       del_task);
    
    m_ownedProxyIDs.erase(owner_itr);
    
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
    //log(string("Deleting feature task added for feature: " + string(feat->m_address)));
    //deleteFromWorkingMemory(string(feat->m_address));
    BindingData::ExplicitFeatureDeletionTask* del_task = 
      new BindingData::ExplicitFeatureDeletionTask();
    
    del_task->m_featureID = CORBA::string_dup(string(feat->m_address).c_str());
    
    assert(boost::regex_match(string(feat->m_address),boost::regex(".+:.+")));
    assert(string(feat->m_address) != "");
    
    addToWorkingMemory(newDataID(),
		       m_bindingSA, 
		       //BindingLocalOntology::BINDING_FEATURE_POINTER_DELETION_TASK_TYPE, 
		       del_task, 
		       cast::cdl::BLOCKING);
  }
}

void
AbstractMonitor::deleteFeatureFromCurrentProxy(const BindingData::FeaturePointer& _ptr)
{
  if(m_currentlyBuiltProxy.get() == NULL) {
    throw(BindingException("Attempt to delete feature from m_proxyFeatures before initiating it with startNewProxy"));
  }
  FeaturePointerSet deleted;
  unsigned int j = 0;
  for(unsigned int i = 0; i < m_currentlyBuiltProxy->m_proxyFeatures.length(); ++i) {
    if(string(m_currentlyBuiltProxy->m_proxyFeatures[i].m_address) == string(_ptr.m_address)) {
      assert(string(m_currentlyBuiltProxy->m_proxyFeatures[i].m_type) == string(_ptr.m_type));
      deleted.insert(_ptr);
    } else {
      m_currentlyBuiltProxy->m_proxyFeatures[j] = m_currentlyBuiltProxy->m_proxyFeatures[i];
      ++j;
    }
  }
  if(deleted.empty()) {
    throw(BindingException("Attempting to delete a feature that was not on the proxy"));
  }
  _deleteFeatures(deleted);
  m_currentlyBuiltProxy->m_proxyFeatures.length(m_currentlyBuiltProxy->m_proxyFeatures.length() - 1);
}


BindingData::FeaturePointer 
AbstractMonitor::addSalienceToCurrentProxy(const BindingFeaturesCommon::StartTime& _start,
					   const BindingFeaturesCommon::EndTime& _end)
{
  BindingFeatures::Salience salience;
  salience.m_start = _start;
  salience.m_end = _end;
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
  parent.m_truthValue = BindingFeaturesCommon::POSITIVE;
  parent.m_immediateProxyID = CORBA::string_dup(m_currentProxyID.c_str());
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
  a.m_subarchitecture = CORBA::string_dup(getBindingSA().c_str());
  a.m_id = CORBA::string_dup(BindingData::bindTheseProxiesMonitorTokenID);
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
  a.m_subarchitecture = CORBA::string_dup(getBindingSA().c_str());
  a.m_id = CORBA::string_dup(BindingData::updateProxyMonitorTokenID);
  return a;
}

/*void 
AbstractMonitor::makeProxiesUnavailableAdded(const cast::cdl::WorkingMemoryChange & _wmc)
{
  try{
    try{
      // mutex the entry first:
      lockEntry(_wmc.m_address,cast::cdl::LOCKED_ODR);
    } catch(const DoesNotExistOnWMException _e) { // ok, it was removed before this monitor could touch it
      return;
    }
    shared_ptr<const BindingQueries::MakeProxiesUnavailable> unavail = loadBindingDataFromWM<BindingQueries::MakeProxiesUnavailable>(_wmc);
    vector<BindingQueries::MakeProxyUnavailable> remaining;
    for(unsigned int i = 0; i < unavail->m_proxies.length() ; ++i) {
      if(string(unavail->m_proxies[i].m_ownerMonitorID) == getProcessIdentifier()) // or proxy should be deleted, aha!
	try{
	  makeProxyUnavailable(unavail->m_proxies[i]);
	} catch(DoesNotExistOnWMException _e) {
	  if(string(_e.address().m_id) == string(unavail->m_proxies[i].m_proxyID)) // ah, it was our proxy that was gone... it was already deleted, no probs
	    ; // noop
	  else // not our proxy, it was sth else that was rotten
	    throw(_e);
	}
      else
	remaining.push_back(unavail->m_proxies[i]);
    }
    if(remaining.size() == 0) { // all job is done
      deleteFromWorkingMemory(_wmc.m_address);
      return;
    }
    if(remaining.size() != unavail->m_proxies.length()) { // i.e. sth was processed
      BindingQueries::MakeProxiesUnavailable* new_unavail = new BindingQueries::MakeProxiesUnavailable();
      new_unavail->m_proxies.length(remaining.size());
      unsigned int j = 0;
      foreach(BindingQueries::MakeProxyUnavailable unav, remaining) {
	new_unavail->m_proxies[j++] = unav;
      }
      overwriteWorkingMemory(_wmc.m_address,new_unavail,cast::cdl::BLOCKING);
    }
  } catch(...) { // just in case
    unlockEntry(_wmc.m_address);
    throw;
  }
  unlockEntry(_wmc.m_address);
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
  string proxyID(unavail->m_proxyID);
  if(m_ownedProxyIDs.find(proxyID) == m_ownedProxyIDs.end()) // not my proxy, ignore
    return;
  // ok, it is my proxy, deal with it:
  // 1st, delete the entry...
  deleteFromWorkingMemory(_wmc.m_address);
  // 2nd, deal with the entry
  try{
    makeProxyUnavailable(*unavail);
  } catch(DoesNotExistOnWMException _e) {
    if(strcmp(_e.address().m_id,proxyID.c_str()) == 0) // ah, it was our proxy that was gone... it was already deleted, no probs
      /*noop*/;
    else // not our proxy, it was sth else that was rotten
      throw(_e);
  }
}

void 
AbstractMonitor::makeProxyUnavailable(const BindingQueries::MakeProxyUnavailable& _makeProxyUnavailable)
{
  deleteExistingProxy(string(_makeProxyUnavailable.m_proxyID));
}

} // namespace Binding
